#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <drivers/gpio.h>
//#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include <zephyr/zephyr.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
//#include <zephyr/device.h>
#include "../services/my_service.h"



#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define LED0_NODE DT_ALIAS(led0)

uint32_t number = 0;
//int16_t temp, press, humidity;
struct sensor_value temp, press, humidity;
const struct device *dev;
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

static K_SEM_DEFINE(ble_init_ok, 0, 1);

static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = 
{
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, MY_SERVICE_UUID),
};

struct bt_conn *my_connection;

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];

	my_connection = conn;

	if (err) 
	{
		printk("Connection failed (err %u)\n", err);
		return;
	}
	else if(bt_conn_get_info(conn, &info))
	{
		printk("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("Connection established!		\n\
		Connected to: %s					\n\
		Role: %u							\n\
		Connection interval: %u				\n\
		Slave latency: %u					\n\
		Connection supervisory timeout: %u	\n"
		, addr, info.role, info.le.interval, info.le.latency, info.le.timeout);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	//If acceptable params, return true, otherwise return false.
	return true; 
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];
	
	if(bt_conn_get_info(conn, &info))
	{
		printk("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("Connection parameters updated!	\n\
		Connected to: %s						\n\
		New Connection Interval: %u				\n\
		New Slave Latency: %u					\n\
		New Connection Supervisory Timeout: %u	\n"
		, addr, info.le.interval, info.le.latency, info.le.timeout);
	}
}

static struct bt_conn_cb conn_callbacks = 
{
	.connected				= connected,
	.disconnected   		= disconnected,
	.le_param_req			= le_param_req,
	.le_param_updated		= le_param_updated
};

static void bt_ready(int err)
{
	if (err) 
	{
		printk("BLE init failed with error code %d\n", err);
		return;
	}

	//Configure connection callbacks
	bt_conn_cb_register(&conn_callbacks);

	//Initalize services
	err = my_service_init();

	if (err) 
	{
		printk("Failed to init LBS (err:%d)\n", err);
		return;
	}

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) 
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");

	k_sem_give(&ble_init_ok);
}

static const struct device *get_bme280_device(void)
{
	const struct device *dev = DEVICE_DT_GET_ANY(bosch_bme280);

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return NULL;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

static void error(void)
{
	while (true) {
		printk("Error!\n");
		/* Spin for ever */
		k_sleep(K_MSEC(1000)); //1000ms
	}
}

/****************************************************************************************************
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
	// Forward the TWI event to the driver
	bme280_twi_evt_handler(p_event, p_context);
}

void bme280_handler(bme280_twi_evt_t const * p_event, void * p_context)
{
	switch (p_event->type) {
		case BME280_TWI_MEASUREMENT_FETCHED:
			m_measurement_fetched = true;
			break;
		default:
			break;	
	}
}
void twi_init(void)
{
	const nrf_drv_twi_config_t twi_config = {
		.scl                = ARDUINO_SCL_PIN,
		.sda                = ARDUINO_SDA_PIN,
		.frequency          = NRF_TWI_FREQ_100K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.clear_bus_init     = false
	};

	ret_code_t err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
	APP_ERROR_CHECK(err_code);

	nrf_drv_twi_enable(&m_twi);
}

void bme280_init(void) {
	const bme280_twi_config_t bme280_twi_config = {
		.addr = BME280_TWI_ADDR_0,
		.standby = BME280_TWI_STANDBY_250_MS,
		.filter = BME280_TWI_FILTER_OFF,
		.temp_oversampling = BME280_TWI_OVERSAMPLING_X4,
	};

	bme280_twi_init(&m_twi, &bme280_twi_config, bme280_handler, NULL);
	bme280_twi_enable();
}

static void log_temp(void)
{
	bme280_twi_data_t data;
	bme280_twi_measurement_get(&data);

	NRF_LOG_INFO("Temperature: " NRF_LOG_FLOAT_MARKER " degrees Celsius.\r\n",
			NRF_LOG_FLOAT(((float)data.temp)/100));
	NRF_LOG_FLUSH();
}

//****************************************************************************************************/

void main(void)
{
	
	int err = 0;

	printk("Starting Nordic BLE peripheral tutorial\n");

	
	err = bt_enable(bt_ready);

	if (err) 
	{
		printk("BLE initialization failed\n");
		error(); //Catch error
	}
	
	/* 	Bluetooth stack should be ready in less than 100 msec. 								\
																							\
		We use this semaphore to wait for bt_enable to call bt_ready before we proceed 		\
		to the main loop. By using the semaphore to block execution we allow the RTOS to 	\
		execute other tasks while we wait. */	
	err = k_sem_take(&ble_init_ok, K_MSEC(500));

	if (!err) 
	{
		printk("Bluetooth initialized\n");
	} else 
	{
		printk("BLE initialization did not complete in time\n");
		error(); //Catch error
	}

	err = my_service_init();

	int ret;

	if (!device_is_ready(led.port)) {
		return;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	dev = get_bme280_device();
	//int16_t temp, press, humidity;

	if (dev == NULL) {
		return;
	}


	for (;;) 
	{
		// Main loop
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("temp: %d.%06d; press: %d.%06d; humidity: %d.%06d\n",
		      temp.val1, temp.val2, press.val1, press.val2,
		      humidity.val1, humidity.val2);

		ret = gpio_pin_toggle_dt(&led);
		//my_service_send(my_connection, (uint8_t *)&number, sizeof(number));
		my_service_send(my_connection, (uint8_t *)&temp, sizeof(temp));
		my_service_send(my_connection, (uint8_t *)&press, sizeof(press));
		my_service_send(my_connection, (uint8_t *)&humidity, sizeof(humidity));
		number++;
		k_sleep(K_MSEC(1000)); // 1000ms
	}
}
