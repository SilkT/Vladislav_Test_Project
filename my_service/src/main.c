/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <soc.h>
//bme
//#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "../services/my_service.h"


#define DEVICE_NAME 		CONFIG_BT_DEVICE_NAME // Set in proj.conf
#define DEVICE_NAME_LEN        	(sizeof(DEVICE_NAME) - 1)

typedef struct{
	uint16_t temp;
	uint16_t pres;
	uint16_t humi;
}bme_meas_t;

static K_SEM_DEFINE(ble_init_ok, 0, 1);
struct bt_conn *my_connection;
static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = 
{
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, MY_SERVICE_UUID),
};
bme_meas_t bme_meas ={0};

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

	my_connection = NULL;
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


static struct bt_conn_cb conn_callbacks = {
	.connected		    = connected,
	.disconnected   	= disconnected,
	.le_param_req		= le_param_req,
	.le_param_updated	= le_param_updated
};

static void bt_ready(int err)
{
	if (err) {
		printk("BLE init failed with error code %d\n", err);
		return;
	}
	//Configure connection callbacks
	bt_conn_cb_register(&conn_callbacks);

	//Initalize services
	err = my_service_init();

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	/*
	err = bt_le_adv_start(BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_ONE_TIME | BT_LE_ADV_OPT_USE_NAME,
							160, 	// units of 0.625ms
							1600),	// units of 0.625ms
						ad, ARRAY_SIZE(ad),
						sd, ARRAY_SIZE(sd));
	*/
	if (err) 
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	k_sem_give(&ble_init_ok);
}

static void error(void)
{
	while (true) {
		printk("Error!\n");
		/* Spin for ever */
		k_sleep(K_MSEC(1000)); //1000ms
	}
}
/*
 * Get a device structure from a devicetree node with compatible
 * "bosch,bme280". (If there are multiple, just pick one.)
 */
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

void main(void)
{
	
	int err = 0;
	uint32_t number = 0;
	struct sensor_value temp, press, humidity;

	const struct device *dev = get_bme280_device();
	if (dev == NULL) {
		return;
	}
	
	printk("Starting Nordic BLE peripheral tutorial\n");
	err = bt_enable(bt_ready);

	if (err) 
	{
		printk("BLE initialization failed\n");
		error(); //Catch error
	}
	

	while (1) {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
		sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("temp: %d.%02d; press: %d.%02d; humidity: %d.%02d\n",
		      temp.val1, temp.val2, press.val1, press.val2,
		      humidity.val1, humidity.val2);
		bme_meas.temp = temp.val1;
		bme_meas.pres = press.val1;
		bme_meas.humi = humidity.val1;

		my_service_send(my_connection, &bme_meas, sizeof(bme_meas));
		//my_service_send(my_connection, &(uint16_t)temp, sizeof(2));
		//my_service_send(my_connection, &(uint16_t)press, sizeof(2));
		//my_service_send(my_connection, &(uint16_t)humidity, sizeof(2));

		k_sleep(K_MSEC(1000));
	}
}
