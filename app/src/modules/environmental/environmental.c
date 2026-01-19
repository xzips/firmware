/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/task_wdt/task_wdt.h>
#include <zephyr/sys/util.h>
#include <drivers/bme68x_iaq.h>
#include <date_time.h>
#include <zephyr/smf.h>

#include "mlx90632_zephyr.h"
#if IS_ENABLED(CONFIG_APP_LED)
#include "led_pwm.h"
#endif

#include "message_channel.h"
#include "modules_common.h"
#include "env_object_encode.h"

/* Register log module */
LOG_MODULE_REGISTER(environmental_module, CONFIG_APP_ENVIRONMENTAL_LOG_LEVEL);

/* Register subscriber */
ZBUS_MSG_SUBSCRIBER_DEFINE(environmental);

/* Observe trigger channel */
ZBUS_CHAN_ADD_OBS(TRIGGER_CHAN, environmental, 0);
ZBUS_CHAN_ADD_OBS(TIME_CHAN, environmental, 0);

#define MAX_MSG_SIZE (MAX(sizeof(enum trigger_type), sizeof(enum time_status)))

BUILD_ASSERT(CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS >
			CONFIG_APP_ENVIRONMENTAL_EXEC_TIME_SECONDS_MAX,
			"Watchdog timeout must be greater than maximum execution time");

static const struct device *const sensor_dev = DEVICE_DT_GET(DT_ALIAS(gas_sensor));

static bool ir_temp_ready;

/* Forward declarations */
static struct s_object s_obj;
static void sample(void);

/* State machine */

/* Defininig the module states.
 *
 * STATE_INIT: The environmental module is initializing and waiting for time to be available.
 * STATE_SAMPLING: The environmental module is ready to sample upon receiving a trigger.
 */
enum environmental_module_state {
	STATE_INIT,
	STATE_SAMPLING,
};

/* User defined state object.
 * Used to transfer data between state changes.
 */
struct s_object {
	/* This must be first */
	struct smf_ctx ctx;

	/* Last channel type that a message was received on */
	const struct zbus_channel *chan;

	/* Buffer for last zbus message */
	uint8_t msg_buf[MAX_MSG_SIZE];
};

/* Forward declarations of state handlers */
static enum smf_state_result state_init_run(void *o);
static enum smf_state_result state_sampling_run(void *o);

static struct s_object s_obj;
static const struct smf_state states[] = {
	[STATE_INIT] =
		SMF_CREATE_STATE(NULL, state_init_run, NULL,
				 NULL,	/* No parent state */
				 NULL), /* No initial transition */
	[STATE_SAMPLING] =
		SMF_CREATE_STATE(NULL, state_sampling_run, NULL,
				 NULL,
				 NULL),
};

/* State handlers */

static enum smf_state_result state_init_run(void *o)
{
	struct s_object *state_object = o;

	if (&TIME_CHAN == state_object->chan) {
		enum time_status time_status = MSG_TO_TIME_STATUS(state_object->msg_buf);

		if (time_status == TIME_AVAILABLE) {
			LOG_DBG("Time available, sampling can start");

			STATE_SET(STATE_SAMPLING);
			return SMF_EVENT_HANDLED;
		}
	}

	return SMF_EVENT_PROPAGATE;
}

static enum smf_state_result state_sampling_run(void *o)
{
	struct s_object *state_object = o;

	if (&TRIGGER_CHAN == state_object->chan) {
		enum trigger_type trigger_type = MSG_TO_TRIGGER_TYPE(state_object->msg_buf);

		if (trigger_type == TRIGGER_DATA_SAMPLE) {
			LOG_DBG("Data sample trigger received, getting environmental data");
			sample();
		}
	}

	return SMF_EVENT_PROPAGATE;
}

/* End of state handling */

static void task_wdt_callback(int channel_id, void *user_data)
{
	LOG_ERR("Watchdog expired, Channel: %d, Thread: %s",
		channel_id, k_thread_name_get((k_tid_t)user_data));

	SEND_FATAL_ERROR_WATCHDOG_TIMEOUT();
}

#if IS_ENABLED(CONFIG_APP_LED)
static float clamp01(float value)
{
	if (value < 0.0f) {
		return 0.0f;
	}
	if (value > 1.0f) {
		return 1.0f;
	}
	return value;
}

static void temp_to_rgb(double temp_c, uint8_t *r, uint8_t *g, uint8_t *b)
{
	float rf = 0.0f;
	float gf = 0.0f;
	float bf = 0.0f;

	if (temp_c <= -10.0) {
		rf = 0.0f;
		gf = 0.0f;
		bf = 1.0f;
	} else if (temp_c >= 60.0) {
		rf = 1.0f;
		gf = 0.0f;
		bf = 0.0f;
	} else if (temp_c < 25.0) {
		float t = (float)((temp_c + 10.0) / 35.0);
		rf = 0.0f;
		gf = t;
		bf = 1.0f - t;
	} else {
		float t = (float)((temp_c - 25.0) / 35.0);
		rf = t;
		gf = 1.0f - t;
		bf = 0.0f;
	}

	*r = (uint8_t)(clamp01(rf) * 255.0f);
	*g = (uint8_t)(clamp01(gf) * 255.0f);
	*b = (uint8_t)(clamp01(bf) * 255.0f);
}

static void update_temp_led(double temp_c)
{
	uint8_t r = 0;
	uint8_t g = 0;
	uint8_t b = 0;
	int ret;

	temp_to_rgb(temp_c, &r, &g, &b);
	ret = led_pwm_set_rgb(r, g, b);
	if (ret) {
		LOG_WRN("Failed to set temp LED color: %d", ret);
	}
}
#else
static void update_temp_led(double temp_c)
{
	ARG_UNUSED(temp_c);
}
#endif

static int ir_temp_init(void)
{
	const struct device *gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
	int ret;

	/* IR temp init: power the external Stemma connector before I2C access. */
	if (!device_is_ready(gpio0)) {
		LOG_ERR("GPIO0 not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure(gpio0, 3, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to enable EXPANSION_3V3_EN: %d", ret);
		return ret;
	}

	k_msleep(500);

	/* IR temp init: initialize the MLX90632 after power is on. */
	ret = (int)mlx90632_zephyr_init();
	if (ret < 0) {
		return ret;
	}

	ir_temp_ready = true;
	return 0;
}

static int ir_temp_sample(double *object_c)
{
	double ambient_c = 0.0;
	double object_c_local = 0.0;
	int ret;

	if (object_c == NULL) {
		return -EINVAL;
	}

	if (!ir_temp_ready) {
		return -EACCES;
	}

	/* IR temp measurement: read MLX90632 object temperature. */
	ret = (int)mlx90632_zephyr_sample(&ambient_c, &object_c_local);
	if (ret < 0) {
		return ret;
	}

	*object_c = object_c_local;
	return 0;
}

static void sample(void)
{
	int64_t system_time;
	struct payload payload = { 0 };
	struct sensor_value temp = { 0 };
	struct sensor_value press = { 0 };
	struct sensor_value humidity = { 0 };
	struct sensor_value iaq = { 0 };
	struct sensor_value co2 = { 0 };
	struct sensor_value voc = { 0 };
	struct sensor_value temp_reported = { 0 };
	struct env_object env_obj = { 0 };
	double ir_object_c = 0.0;
	double temp_reported_c;
	bool ir_ok = false;
	int ret;

	ret = sensor_sample_fetch(sensor_dev);
	__ASSERT_NO_MSG(ret == 0);
	ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	__ASSERT_NO_MSG(ret == 0);
	ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_PRESS, &press);
	__ASSERT_NO_MSG(ret == 0);
	ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_HUMIDITY, &humidity);
	__ASSERT_NO_MSG(ret == 0);
	ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_IAQ, &iaq);
	__ASSERT_NO_MSG(ret == 0);
	ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_CO2, &co2);
	__ASSERT_NO_MSG(ret == 0);
	ret = sensor_channel_get(sensor_dev, SENSOR_CHAN_VOC, &voc);
	__ASSERT_NO_MSG(ret == 0);

	temp_reported = temp;
	temp_reported_c = sensor_value_to_double(&temp);

	/* IR temp measurement: prefer MLX90632 object temperature over BME ambient. */
	if (ir_temp_ready) {
		ret = ir_temp_sample(&ir_object_c);
		if (ret == 0) {
			ir_ok = true;
			ret = sensor_value_from_double(&temp_reported, ir_object_c);
			if (ret == 0) {
				temp_reported_c = ir_object_c;
			} else {
				LOG_WRN("Failed to convert IR temperature: %d", ret);
				ir_ok = false;
				temp_reported = temp;
				temp_reported_c = sensor_value_to_double(&temp);
			}
		} else {
			LOG_WRN("MLX90632 sample failed: %d", ret);
		}
	}

	LOG_DBG("temp(%s): %d.%06d; press: %d.%06d; humidity: %d.%06d; iaq: %d; CO2: %d.%06d; "
		"VOC: %d.%06d",
		ir_ok ? "ir" : "bme",
		temp_reported.val1, temp_reported.val2, press.val1, press.val2, humidity.val1, humidity.val2,
		iaq.val1, co2.val1, co2.val2, voc.val1, voc.val2);

	ret = date_time_now(&system_time);
	if (ret) {
		LOG_ERR("Failed to convert uptime to unix time, error: %d", ret);
		return;
	}

	env_obj.temperature_m.bt = (int32_t)(system_time / 1000);
	/* Substitute IR temperature for cloud reporting (fallback to BME if needed). */
	env_obj.temperature_m.vf = temp_reported_c;
	env_obj.humidity_m.vf = sensor_value_to_double(&humidity);
	env_obj.pressure_m.vf = sensor_value_to_double(&press) / 100;
	env_obj.iaq_m.vi = iaq.val1;

	/* Update LED color based on the reported temperature. */
	update_temp_led(temp_reported_c);

	ret = cbor_encode_env_object(payload.buffer, sizeof(payload.buffer),
				     &env_obj, &payload.buffer_len);
	if (ret) {
		LOG_ERR("Failed to encode env object, error: %d", ret);
		SEND_FATAL_ERROR();
		return;
	}

	LOG_DBG("Submitting payload");

	int err = zbus_chan_pub(&PAYLOAD_CHAN, &payload, K_SECONDS(1));
	if (err) {
		LOG_ERR("zbus_chan_pub, error: %d", err);
		SEND_FATAL_ERROR();
		return;
	}
}

static void environmental_task(void)
{
	int err;
	int task_wdt_id;
	const uint32_t wdt_timeout_ms = (CONFIG_APP_ENVIRONMENTAL_WATCHDOG_TIMEOUT_SECONDS * MSEC_PER_SEC);
	const uint32_t execution_time_ms = (CONFIG_APP_ENVIRONMENTAL_EXEC_TIME_SECONDS_MAX * MSEC_PER_SEC);
	const k_timeout_t zbus_wait_ms = K_MSEC(wdt_timeout_ms - execution_time_ms);

	LOG_DBG("Environmental module task started");

	task_wdt_id = task_wdt_add(wdt_timeout_ms, task_wdt_callback, (void *)k_current_get());

	/* IR temp init: power/initialize the sensor once before sampling. */
	err = ir_temp_init();
	if (err) {
		LOG_WRN("MLX90632 init failed: %d", err);
	}

	STATE_SET_INITIAL(STATE_INIT);

	while (true) {
		err = task_wdt_feed(task_wdt_id);
		if (err) {
			LOG_ERR("task_wdt_feed, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = zbus_sub_wait_msg(&environmental, &s_obj.chan, s_obj.msg_buf, zbus_wait_ms);
		if (err == -ENOMSG) {
			continue;
		} else if (err) {
			LOG_ERR("zbus_sub_wait_msg, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}

		err = STATE_RUN();
		if (err) {
			LOG_ERR("handle_message, error: %d", err);
			SEND_FATAL_ERROR();
			return;
		}
	}
}

K_THREAD_DEFINE(environmental_task_id,
		CONFIG_APP_ENVIRONMENTAL_THREAD_STACK_SIZE,
		environmental_task, NULL, NULL, NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, 0);
