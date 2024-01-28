#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(uart)

#define SW0_NODE DT_ALIAS(sw0)
#define PPS_NODE DT_ALIAS(pps)

static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});
static const struct gpio_dt_spec pps_dev = GPIO_DT_SPEC_GET_OR(PPS_NODE, gpios, {0});

static struct gpio_callback button_cb_data;
static struct gpio_callback pps_dev_cb_data;
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static const struct device *const dht22 = DEVICE_DT_GET_ONE(aosong_dht);

#define MSG_SIZE 256
#define PRIORITY_A 6
#define PRIORITY_B 7
#define PRIORITY_C 8
#define STACKSIZE 1024

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

#define SLEEP_TIME_MS 100

static int pps_cycle;
static int pps_cnt;
static double utc_time;
static double lat;
static double alt;
static double lon;
static int sat;

void pps_dev_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	pps_cycle = 0; // k_cycle_get_32();
	pps_cnt++;
}

int hex2int(char *c);
int checksum_valid(char *string);
int parse_comma_delimited_str(char *string, char **fields, int max_fields);

void parse_nwea_gps(char *buffer)
{
	char *field[20];
	if (checksum_valid(buffer))
	{
		if (buffer[0] == '$' && buffer[1] == 'G' && buffer[2] == 'P' &&
			buffer[3] == 'G' && buffer[4] == 'G' && buffer[5] == 'A')
		{
			int ret = parse_comma_delimited_str(buffer, field, 20);
			if (ret > 9)
			{
				utc_time = strtod(field[1], NULL);
				lat = strtod(field[2], NULL);
				lon = strtod(field[4], NULL);
				alt = strtod(field[9], NULL);
				sat = strtol(field[7], NULL, 10);
				if (field[3][0] == 'S')
					lat = -lat;
				if (field[5][0] == 'W')
					lon = -lon;
			}
		}
	}
}

int hexchar2int(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return -1;
}

int hex2int(char *c)
{
	int value;

	value = hexchar2int(c[0]);
	value = value << 4;
	value += hexchar2int(c[1]);

	return value;
}

int checksum_valid(char *string)
{
	char *checksum_str;
	int checksum;
	unsigned char calculated_checksum = 0;

	// Checksum is postcede by *
	checksum_str = strchr(string, '*');
	if (checksum_str != NULL)
	{
		// Remove checksum from string
		*checksum_str = '\0';
		// Calculate checksum, starting after $ (i = 1)
		for (int i = 1; i < strlen(string); i++)
		{
			calculated_checksum = calculated_checksum ^ string[i];
		}
		checksum = hex2int((char *)checksum_str + 1);
		// printk("Checksum Str [%s], Checksum %02X, Calculated Checksum %02X\r\n", (char *)checksum_str + 1, checksum, calculated_checksum);
		if (checksum == calculated_checksum)
		{
			return 1;
		}
	}
	else
	{
		// printf("Error: Checksum missing or NULL NMEA message\r\n");
		return 0;
	}
	return 0;
}

int parse_comma_delimited_str(char *string, char **fields, int max_fields)
{
	int i = 0;
	fields[i++] = string;

	while ((i < max_fields) && NULL != (string = strchr(string, ',')))
	{
		*string = '\0';
		fields[i++] = ++string;
	}

	return --i;
}

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev))
	{
		return;
	}

	if (!uart_irq_rx_ready(uart_dev))
	{
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(uart_dev, &c, 1) == 1)
	{
		if ((c == '\n' || c == '\r') && rx_buf_pos > 0)
		{
			/* terminate string */
			rx_buf[rx_buf_pos] = '\0';

			/* if queue is full, message is silently dropped */
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

			/* reset the buffer (it was copied to the msgq) */
			rx_buf_pos = 0;
		}
		else if (rx_buf_pos < (sizeof(rx_buf) - 1))
		{
			rx_buf[rx_buf_pos++] = c;
		}
		/* else: characters beyond buffer size are dropped */
	}
}

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++)
	{
		uart_poll_out(uart_dev, buf[i]);
	}
}

void thread_a_entry_point(void *dummy1, void *dummy2, void *dummy3)
{
	int ret;
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	if (!gpio_is_ready_dt(&button))
	{
		printk("Error: button device %s is not ready\n", button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	printk("Set up button at %s pin %d\n", button.port->name, button.pin);

	if (led.port && !device_is_ready(led.port))
	{
		printk("Error %d: LED device %s is not ready; ignoring it\n",
			   ret, led.port->name);
		led.port = NULL;
	}
	if (led.port)
	{
		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT);
		if (ret != 0)
		{
			printk("Error %d: failed to configure LED device %s pin %d\n",
				   ret, led.port->name, led.pin);
			led.port = NULL;
		}
		else
		{
			printk("Set up LED at %s pin %d\n", led.port->name, led.pin);
		}
	}

	printk("Press the button\n");
	if (led.port)
	{
		while (1)
		{

			/* If we have an LED, match its state to the button's. */
			int val = gpio_pin_get_dt(&button);

			if (val >= 0)
			{
				gpio_pin_set_dt(&led, val);
			}

			k_msleep(SLEEP_TIME_MS);
		}
	}
}

void thread_b_entry_point(void *dummy1, void *dummy2, void *dummy3)
{
	int ret;
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	char tx_buf[MSG_SIZE];

	if (!device_is_ready(uart_dev))
	{
		printk("UART device not found!");
		return;
	}

	/* configure interrupt and callback to receive data */
	ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0)
	{
		if (ret == -ENOTSUP)
		{
			printk("Interrupt-driven UART API support not enabled\n");
		}
		else if (ret == -ENOSYS)
		{
			printk("UART device does not support interrupt-driven API\n");
		}
		else
		{
			printk("Error setting UART callback: %d\n", ret);
		}
		return;
	}
	uart_irq_rx_enable(uart_dev);

	while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0)
	{
		int i;
		for (i = 0; i < MSG_SIZE; i++)
		{
			if (tx_buf[i] == '$')
				break;
		}
		if (i < MSG_SIZE)
		{
			parse_nwea_gps(&tx_buf[i]);
		}
	}
}
static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
			 h, min, s, ms);
	return buf;
}
void thread_c_entry_point(void *dummy1, void *dummy2, void *dummy3)
{

	if (!device_is_ready(dht22))
	{
		printf("Device %s is not ready\n", dht22->name);
		return;
	}

	while (true)
	{
		int rc = sensor_sample_fetch(dht22);

		if (rc != 0)
		{
			printf("Sensor fetch failed: %d\n", rc);
			k_sleep(K_SECONDS(1));
			continue;
		}

		struct sensor_value temperature;
		struct sensor_value humidity;
		/*temperature.val1=0;
		temperature.val2=0;
		humidity.val1=0;
		humidity.val2=0;
		*/
		rc = sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP,
								&temperature);
		if (rc == 0)
		{
			rc = sensor_channel_get(dht22, SENSOR_CHAN_HUMIDITY,
									&humidity);
		}
		if (rc != 0)
		{
			printf("get failed: %d\n", rc);
			k_sleep(K_SECONDS(1));
			continue;
		}

		int utc = (int)utc_time;
		int h = utc / 10000;
		utc -= h * 10000;
		int m = utc / 100;
		int s = utc - m * 100;
		printf("%d %6d %s [%02d:%02d:%02dZ] lat=%.6f, lon=%.6f, alt=%.1f Cel=%.1f RH=%.1f\n",
			   pps_cycle, pps_cnt, now_str(), h, m, s, lat, lon, alt,
			   sensor_value_to_double(&temperature),
			   sensor_value_to_double(&humidity));
		k_sleep(K_SECONDS(2));
	}
}

K_THREAD_DEFINE(thread_a, STACKSIZE,
				thread_a_entry_point, NULL, NULL, NULL,
				PRIORITY_A, 0, 0);
extern const k_tid_t thread_a;

K_THREAD_DEFINE(thread_b, STACKSIZE,
				thread_b_entry_point, NULL, NULL, NULL,
				PRIORITY_B, 0, 0);
extern const k_tid_t thread_b;

K_THREAD_DEFINE(thread_c, STACKSIZE,
				thread_c_entry_point, NULL, NULL, NULL,
				PRIORITY_C, 0, 0);
extern const k_tid_t thread_c;

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&pps_dev))
	{
		printk("Error: pps_dev device %s is not ready\n", pps_dev.port->name);
		return 0;
	}

	ret = gpio_pin_configure_dt(&pps_dev, GPIO_INPUT);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, pps_dev.port->name, pps_dev.pin);
		return 0;
	}

	ret = gpio_pin_interrupt_configure_dt(&pps_dev, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		printk("Error %d: failed to configure interrupt on %s pin %d\n",
			   ret, pps_dev.port->name, pps_dev.pin);
		return 0;
	}

	gpio_init_callback(&pps_dev_cb_data, pps_dev_pressed, BIT(pps_dev.pin));
	gpio_add_callback(pps_dev.port, &pps_dev_cb_data);

	printk("Set up pps_dev at %s pin %d\n", pps_dev.port->name, pps_dev.pin);

	return 0;
}
