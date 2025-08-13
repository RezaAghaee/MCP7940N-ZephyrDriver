/*
 * Copyright (c) 2023 Alvaro Garcia Gomez <maxpowel@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <stdint.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/rtc/rtc_mcp7940.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(mcp7940);

#define DT_DRV_COMPAT microchip_mcp7940


/* Alarm channels */
#define ALARM0_ID			0
#define ALARM1_ID			1

/* Size of block when writing whole struct */
#define RTC_TIME_REGISTERS_SIZE		sizeof(struct mcp7940n_time_registers)
#define RTC_ALARM_REGISTERS_SIZE	sizeof(struct mcp7940n_alarm_registers)

/* Largest block size */
#define MAX_WRITE_SIZE                  (RTC_TIME_REGISTERS_SIZE)

/* Macro used to decode BCD to UNIX time to avoid potential copy and paste
 * errors.
 */
#define RTC_BCD_DECODE(reg_prefix) (reg_prefix##_one + reg_prefix##_ten * 10)

/* tm struct uses years since 1900 but unix time uses years since
 * 1970. MCP7940N default year is '1' so the offset is 69
 */
#define UNIX_YEAR_OFFSET		69


#if DT_ANY_INST_HAS_PROP_STATUS_OKAY(int_gpios) &&                                                \
	(defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_UPDATE))
/* The user may need only alarms but not interrupts so we will only
 * include all the interrupt code if the user configured it in the dts
 */
#define MCP7940N_INT_GPIOS_IN_USE 1
#endif

/* RTC alarm time fields supported by the MCP7940N */
#define MCP7940N_RTC_ALARM_TIME_MASK                                                         \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |    \
	 RTC_ALARM_TIME_MASK_WEEKDAY)

//#ifdef MCP7940N_INT_GPIOS_IN_USE
/* This work will run the user callback function */
static void mcp7940n_work_handler(struct k_work *work);
K_WORK_DEFINE(alarm_work, mcp7940n_work_handler);
//#endif

struct mcp7940n_config {
	//struct counter_config_info generic;
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec int_gpios;
	const struct gpio_dt_spec vcc_gpios;
};


struct mcp7940n_data {
	const struct device *mcp7940n;
	struct k_sem lock;
	struct mcp7940n_time_registers registers;
	struct mcp7940n_alarm_registers alm0_registers;
	struct mcp7940n_alarm_registers alm1_registers;

	struct k_work alarm_work;
	struct gpio_callback int_callback;

	rtc_alarm_callback alarm_handler[2];
	void *alarm_user_data[2];

	bool int_active_high;
};

/** @brief Convert bcd time in device registers to rtc_time time
 *
 * @param dev the MCP7940N device pointer.
 *
 * @retval returns rtc_time time.
 */
static struct rtc_time decode_rtc(const struct device *dev)
{
	struct mcp7940n_data *data = dev->data;
	struct rtc_time time = { 0 };

	time.tm_sec = RTC_BCD_DECODE(data->registers.rtc_sec.sec);
	time.tm_min = RTC_BCD_DECODE(data->registers.rtc_min.min);
	time.tm_hour = RTC_BCD_DECODE(data->registers.rtc_hours.hr);
	time.tm_mday = RTC_BCD_DECODE(data->registers.rtc_date.date);
	time.tm_wday = data->registers.rtc_weekday.weekday;
	/* tm struct starts months at 0, mcp7940n starts at 1 */
	time.tm_mon = RTC_BCD_DECODE(data->registers.rtc_month.month) - 1;
	/* tm struct uses years since 1900 but unix time uses years since 1970 */
	time.tm_year = RTC_BCD_DECODE(data->registers.rtc_year.year) +
		UNIX_YEAR_OFFSET;

	return time;
}

/** @brief Encode time struct tm into mcp7940n rtc registers
 *
 * @param dev the MCP7940N device pointer.
 * @param time_buffer rtc_time struct containing time to be encoded into mcp7940n
 * registers.
 *
 * @retval return 0 on success, or a negative error code from invalid
 * parameter.
 */
static int encode_rtc(const struct device *dev, struct rtc_time *time_buffer)
{
	struct mcp7940n_data *data = dev->data;
	uint8_t month;
	uint8_t year_since_epoch;

	/* In a rtc_time struct, months start at 0, mcp7940n starts with 1 */
	month = time_buffer->tm_mon + 1;

	if (time_buffer->tm_year < UNIX_YEAR_OFFSET) {
		return -EINVAL;
	}
	year_since_epoch = time_buffer->tm_year - UNIX_YEAR_OFFSET;

	/* Set external oscillator configuration bit */
	data->registers.rtc_sec.start_osc = 1;

	data->registers.rtc_sec.sec_one = time_buffer->tm_sec % 10;
	data->registers.rtc_sec.sec_ten = time_buffer->tm_sec / 10;
	data->registers.rtc_min.min_one = time_buffer->tm_min % 10;
	data->registers.rtc_min.min_ten = time_buffer->tm_min / 10;
	data->registers.rtc_hours.hr_one = time_buffer->tm_hour % 10;
	data->registers.rtc_hours.hr_ten = time_buffer->tm_hour / 10;
	data->registers.rtc_weekday.weekday = time_buffer->tm_wday;
	data->registers.rtc_date.date_one = time_buffer->tm_mday % 10;
	data->registers.rtc_date.date_ten = time_buffer->tm_mday / 10;
	data->registers.rtc_month.month_one = month % 10;
	data->registers.rtc_month.month_ten = month / 10;
	data->registers.rtc_year.year_one = year_since_epoch % 10;
	data->registers.rtc_year.year_ten = year_since_epoch / 10;

	return 0;
}

/** @brief Encode time struct rtc_time into mcp7940n alarm registers
 *
 * @param dev the MCP7940N device pointer.
 * @param time_buffer rtc_time struct containing time to be encoded into mcp7940n
 * registers.
 * @param alarm_id alarm ID, can be 0 or 1 for MCP7940N.
 *
 * @retval return 0 on success, or a negative error code from invalid
 * parameter.
 */
static int encode_alarm(const struct device *dev, struct rtc_time *time_buffer, uint8_t alarm_id)
{
	struct mcp7940n_data *data = dev->data;
	uint8_t month;
	struct mcp7940n_alarm_registers *alm_regs;

	if (alarm_id == ALARM0_ID) {
		alm_regs = &data->alm0_registers;
	} else if (alarm_id == ALARM1_ID) {
		alm_regs = &data->alm1_registers;
	} else {
		return -EINVAL;
	}
	/* In a rtc_time struct, months start at 0 */
	month = time_buffer->tm_mon + 1;

	alm_regs->alm_sec.sec_one = time_buffer->tm_sec % 10;
	alm_regs->alm_sec.sec_ten = time_buffer->tm_sec / 10;
	alm_regs->alm_min.min_one = time_buffer->tm_min % 10;
	alm_regs->alm_min.min_ten = time_buffer->tm_min / 10;
	alm_regs->alm_hours.hr_one = time_buffer->tm_hour % 10;
	alm_regs->alm_hours.hr_ten = time_buffer->tm_hour / 10;
	alm_regs->alm_weekday.weekday = time_buffer->tm_wday;
	alm_regs->alm_date.date_one = time_buffer->tm_mday % 10;
	alm_regs->alm_date.date_ten = time_buffer->tm_mday / 10;
	alm_regs->alm_month.month_one = month % 10;
	alm_regs->alm_month.month_ten = month / 10;

	return 0;
}

/** @brief Reads single register from MCP7940N
 *
 * @param dev the MCP7940N device pointer.
 * @param addr register address.
 * @param val pointer to uint8_t that will contain register value if
 * successful.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction.
 */
static int read_register(const struct device *dev, uint8_t addr, uint8_t *val)
{
	const struct mcp7940n_config *cfg = dev->config;

	int rc = i2c_write_read_dt(&cfg->i2c, &addr, sizeof(addr), val, 1);

	return rc;
}

/** @brief Read registers from device and populate mcp7940n_registers struct
 *
 * @param dev the MCP7940N device pointer.
 * @param tm_time pointer to rtc_time value that will contain time if
 * successful.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction.
 */
static int read_time(const struct device *dev, struct rtc_time *current_time)
{
	struct mcp7940n_data *data = dev->data;
	const struct mcp7940n_config *cfg = dev->config;
	uint8_t addr = REG_RTC_SEC;

	int rc = i2c_write_read_dt(&cfg->i2c, &addr, sizeof(addr), &data->registers,
				   RTC_TIME_REGISTERS_SIZE);

	if (rc >= 0) {
		*current_time = decode_rtc(dev);
	}

	return rc;
}

/** @brief Write a single register to MCP7940N
 *
 * @param dev the MCP7940N device pointer.
 * @param addr register address.
 * @param value Value that will be written to the register.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
static int write_register(const struct device *dev, enum mcp7940n_register addr, uint8_t value)
{
	const struct mcp7940n_config *cfg = dev->config;
	int rc = 0;

	uint8_t time_data[2] = {addr, value};

	rc = i2c_write_dt(&cfg->i2c, time_data, sizeof(time_data));

	return rc;
}

/** @brief Write a full time struct to MCP7940N registers.
 *
 * @param dev the MCP7940N device pointer.
 * @param addr first register address to write to, should be REG_RTC_SEC,
 * REG_ALM0_SEC or REG_ALM0_SEC.
 * @param size size of data struct that will be written.
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
static int write_data_block(const struct device *dev, enum mcp7940n_register addr, uint8_t size)
{
	struct mcp7940n_data *data = dev->data;
	const struct mcp7940n_config *cfg = dev->config;
	int rc = 0;
	uint8_t time_data[MAX_WRITE_SIZE + 1];
	uint8_t *write_block_start;

	if (size > MAX_WRITE_SIZE) {
		return -EINVAL;
	}

	if (addr >= REG_INVAL) {
		return -EINVAL;
	}

	if (addr == REG_RTC_SEC) {
		write_block_start = (uint8_t *)&data->registers;
	} else if (addr == REG_ALM0_SEC) {
		write_block_start = (uint8_t *)&data->alm0_registers;
	} else if (addr == REG_ALM1_SEC) {
		write_block_start = (uint8_t *)&data->alm1_registers;
	} else {
		return -EINVAL;
	}

	/* Load register address into first byte then fill in data values */
	time_data[0] = addr;
	memcpy(&time_data[1], write_block_start, size);

	rc = i2c_write_dt(&cfg->i2c, time_data, size + 1);

	return rc;
}

/** @brief Sets the correct weekday.
 *
 * If the time is never set then the device defaults to 1st January 1970
 * but with the wrong weekday set. This function ensures the weekday is
 * correct in this case.
 *
 * @param dev the MCP7940N device pointer.
 * @param tm_time pointer to time that will be used to work out the weekday
 *
 * @retval return 0 on success, or a negative error code from an I2C
 * transaction or invalid parameter.
 */
static int set_day_of_week(const struct device *dev, struct rtc_time *tm_time)
{
	struct mcp7940n_data *data = dev->data;
	int rc = 0;

	data->registers.rtc_weekday.weekday = tm_time->tm_wday;
	rc = write_register(dev, REG_RTC_WDAY, *((uint8_t *)(&data->registers.rtc_weekday)));

	return rc;
}

/** @brief Checks the interrupt pending flag (IF) of a given alarm.
 *
 * A callback is fired if an IRQ is pending.
 *
 * @param dev the MCP7940N device pointer.
 * @param alarm_id ID of alarm, can be 0 or 1 for MCP7940N.
 */
static void mcp7940n_handle_interrupt(const struct device *dev, uint8_t alarm_id)
{
	struct mcp7940n_data *data = dev->data;
	uint8_t alarm_reg_address;
	struct mcp7940n_alarm_registers *alm_regs;
	rtc_alarm_callback cb;
	bool fire_callback = false;

	if (alarm_id == ALARM0_ID) {
		alarm_reg_address = REG_ALM0_WDAY;
		alm_regs = &data->alm0_registers;
	} else if (alarm_id == ALARM1_ID) {
		alarm_reg_address = REG_ALM1_WDAY;
		alm_regs = &data->alm1_registers;
	} else {
		return;
	}

	k_sem_take(&data->lock, K_FOREVER);

	/* Check if this alarm has a pending interrupt */
	read_register(dev, alarm_reg_address, (uint8_t *)&alm_regs->alm_weekday);

	if (alm_regs->alm_weekday.alm_if) {
		/* Clear interrupt */
		alm_regs->alm_weekday.alm_if = 0;
		write_register(dev, alarm_reg_address,
			*((uint8_t *)(&alm_regs->alm_weekday)));

		/* Fire callback */
		if (data->alarm_handler[alarm_id]) {
			cb = data->alarm_handler[alarm_id];
			fire_callback = true;
		}
	}

	k_sem_give(&data->lock);

	if (fire_callback) {
		cb(data->mcp7940n, alarm_id, data->alarm_user_data[alarm_id]);
	}
}

static void mcp7940n_work_handler(struct k_work *work)
{
	struct mcp7940n_data *data =
		CONTAINER_OF(work, struct mcp7940n_data, alarm_work);

	/* Check interrupt flags for both alarms */
	mcp7940n_handle_interrupt(data->mcp7940n, ALARM0_ID);
	mcp7940n_handle_interrupt(data->mcp7940n, ALARM1_ID);
}

static void mcp7940n_init_cb(const struct device *dev,
				 struct gpio_callback *gpio_cb, uint32_t pins)
{
	struct mcp7940n_data *data =
		CONTAINER_OF(gpio_cb, struct mcp7940n_data, int_callback);

	ARG_UNUSED(pins);

	k_work_submit(&data->alarm_work);
}


int mcp7940n_rtc_set_time(const struct device *dev, const struct rtc_time *new_time)
{
	struct mcp7940n_data *data = dev->data;
	int rc = 0;

	k_sem_take(&data->lock, K_FOREVER);

	/* Encode time */
	rc = encode_rtc(dev, new_time);
	if (rc < 0) {
		goto out;
	}

	/* Write to device */
	rc = write_data_block(dev, REG_RTC_SEC, RTC_TIME_REGISTERS_SIZE);

out:
	k_sem_give(&data->lock);

	return rc;
}

int mcp7940n_rtc_get_time(const struct device *dev, struct rtc_time *dest_time)
{
	struct mcp7940n_data *data = dev->data;
	struct rtc_time current_time;
	int rc;

	k_sem_take(&data->lock, K_FOREVER);

	/* Get time */
	rc = read_time(dev, &current_time);

	/* Convert time to ticks */
	if (rc >= 0) {
		*dest_time = current_time;
	}

	k_sem_give(&data->lock);

	return rc;
}



#ifdef CONFIG_RTC_ALARM

static int mcp7940n_rtc_alarm_get_supported_fields(const struct device *dev, uint16_t id,
					      uint16_t *mask)
{
	ARG_UNUSED(dev);

	/* This device only has two channel*/
	if ((id != ALARM0_ID) && (id != ALARM1_ID)) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	*mask = MCP7940N_RTC_ALARM_TIME_MASK;

	return 0;
}

static int mcp7940n_alarm_set_time(const struct device *dev, uint16_t alarm_id, uint16_t mask,
				  const struct rtc_time *timeptr)
{
	struct mcp7940n_data *data = dev->data;
	uint8_t alarm_base_address;
	struct mcp7940n_alarm_registers *alm_regs;
	int rc = 0;
	
	k_sem_take(&data->lock, K_FOREVER);

	if (alarm_id == ALARM0_ID) {
		alarm_base_address = REG_ALM0_SEC;
		alm_regs = &data->alm0_registers;
	} else if (alarm_id == ALARM1_ID) {
		alarm_base_address = REG_ALM1_SEC;
		alm_regs = &data->alm1_registers;
	} else {
		rc = -EINVAL;
		goto out;
	}
	
	/* Set alarm trigger mask and alarm enable flag */
	if (alarm_id == ALARM0_ID) {
		data->registers.rtc_control.alm0_en = 1;
	} else if (alarm_id == ALARM1_ID) {
		data->registers.rtc_control.alm1_en = 1;
	}

	/* Set alarm to match with second, minute, hour, day of week, day of
	 * month and month
	 */
	if((mask & RTC_ALARM_TIME_MASK_SECOND) == RTC_ALARM_TIME_MASK_SECOND) {
		alm_regs->alm_weekday.alm_msk = MCP7940N_ALARM_TRIGGER_SECONDS;
	} 
	if((mask & RTC_ALARM_TIME_MASK_MINUTE) == RTC_ALARM_TIME_MASK_MINUTE) {
		alm_regs->alm_weekday.alm_msk |= MCP7940N_ALARM_TRIGGER_MINUTES;
	}
	if((mask & RTC_ALARM_TIME_MASK_HOUR) == RTC_ALARM_TIME_MASK_HOUR) {
		alm_regs->alm_weekday.alm_msk |= MCP7940N_ALARM_TRIGGER_HOURS;
	}
	if((mask & RTC_ALARM_TIME_MASK_WEEKDAY) == RTC_ALARM_TIME_MASK_WEEKDAY) {
		alm_regs->alm_weekday.alm_msk |= MCP7940N_ALARM_TRIGGER_WDAY;
	}
	if(((mask & RTC_ALARM_TIME_MASK_MONTHDAY) == RTC_ALARM_TIME_MASK_MONTHDAY) || 
	   ((mask & RTC_ALARM_TIME_MASK_MONTH) == RTC_ALARM_TIME_MASK_MONTH) ||
	   ((mask & RTC_ALARM_TIME_MASK_YEAR) == RTC_ALARM_TIME_MASK_YEAR) ||
	   ((mask & RTC_ALARM_TIME_MASK_YEARDAY) == RTC_ALARM_TIME_MASK_YEARDAY) ){
		alm_regs->alm_weekday.alm_msk |= MCP7940N_ALARM_TRIGGER_DATE;
	}
	
	/* Write time to alarm registers */
	encode_alarm(dev, timeptr, alarm_id);
	rc = write_data_block(dev, alarm_base_address, RTC_ALARM_REGISTERS_SIZE);
	if (rc < 0) {
		goto out;
	}
	
	/* Enable alarm */
	rc = write_register(dev, REG_RTC_CONTROL,
		*((uint8_t *)(&data->registers.rtc_control)));
	if (rc < 0) {
		goto out;
	}
	
	
out:
	k_sem_give(&data->lock);

	return rc;
}

static int mcp7940n_alarm_get_time(const struct device *dev, uint16_t alarm_id, uint16_t *mask,
				  struct rtc_time *timeptr)
{
	struct mcp7940n_data *data = dev->data;
	const struct mcp7940n_config *cfg = dev->config;
	uint8_t alarm_base_address;
	struct mcp7940n_alarm_registers *alm_regs;
	int rc = 0;
	
	k_sem_take(&data->lock, K_FOREVER);

	if (alarm_id == ALARM0_ID) {
		alarm_base_address = REG_ALM0_SEC;
		alm_regs = &data->alm0_registers;
	} else if (alarm_id == ALARM1_ID) {
		alarm_base_address = REG_ALM1_SEC;
		alm_regs = &data->alm1_registers;
	} else {
		rc = -EINVAL;
		return rc;
	}
	
	rc = i2c_write_read_dt(&cfg->i2c, alarm_base_address, RTC_ALARM_REGISTERS_SIZE, &data->registers,
				   RTC_TIME_REGISTERS_SIZE);

	if (rc >= 0) {
		*timeptr = decode_rtc(dev);
	}
	
	return rc;
}

static int mcp7940n_alarm_is_pending(const struct device *dev, uint16_t alarm_id)
{
	struct mcp7940n_data *data = dev->data;
	uint32_t interrupt_pending = 0;
	int rc;

	k_sem_take(&data->lock, K_FOREVER);

	if (alarm_id == ALARM0_ID) {
		/* Check interrupt flag for alarm 0 */
		rc = read_register(dev, REG_ALM0_WDAY,
			(uint8_t *)&data->alm0_registers.alm_weekday);
		if (rc < 0) {
			goto out;
		}
	
		if (data->alm0_registers.alm_weekday.alm_if) {
			/* Clear interrupt */
			data->alm0_registers.alm_weekday.alm_if = 0;
			rc = write_register(dev, REG_ALM0_WDAY,
				*((uint8_t *)(&data->alm0_registers.alm_weekday)));
			if (rc < 0) {
				goto out;
			}
			interrupt_pending |= (1 << ALARM0_ID);
		}
	} else if (alarm_id == ALARM1_ID) {
		/* Check interrupt flag for alarm 1 */
		rc = read_register(dev, REG_ALM1_WDAY,
			(uint8_t *)&data->alm1_registers.alm_weekday);
		if (rc < 0) {
			goto out;
		}

		if (data->alm1_registers.alm_weekday.alm_if) {
			/* Clear interrupt */
			data->alm1_registers.alm_weekday.alm_if = 0;
			rc = write_register(dev, REG_ALM1_WDAY,
				*((uint8_t *)(&data->alm1_registers.alm_weekday)));
			if (rc < 0) {
				goto out;
			}
			interrupt_pending |= (1 << ALARM1_ID);
		}
	}

out:
	k_sem_give(&data->lock);

	if (rc) {
		interrupt_pending = 0;
	}
	return (interrupt_pending);
}
#endif



static int mcp7940n_alarm_set_callback(const struct device *dev, uint16_t id,
				      rtc_alarm_callback callback, void *user_data)
{
	struct mcp7940n_data *data = dev->data;
	const struct mcp7940n_config *cfg = dev->config;
	int rc = 0;

	if ((id != ALARM0_ID) && (id != ALARM1_ID)) {
		LOG_ERR("invalid ID %d", id);
		return -EINVAL;
	}

	data->alarm_handler[id] = callback;
	data->alarm_user_data[id] = user_data;
	data->mcp7940n = dev;

	return rc;
}

static const struct rtc_driver_api mcp7940n_driver_api = {
	.set_time = mcp7940n_rtc_set_time,
	.get_time = mcp7940n_rtc_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = mcp7940n_rtc_alarm_get_supported_fields,
	.alarm_set_time = mcp7940n_alarm_set_time,
	.alarm_get_time = mcp7940n_alarm_get_time,
	.alarm_is_pending = mcp7940n_alarm_is_pending,
#endif
//#ifdef MCP7940N_INT_GPIOS_IN_USE
	.alarm_set_callback = mcp7940n_alarm_set_callback,
//#endif
};


int mcp7940n_init(const struct device *dev)
{
	const struct mcp7940n_config *cfg = dev->config;
	struct mcp7940n_data *data = dev->data;
	int rc = 0;
	struct rtc_time current_time;
	
	/* Initialize and take the lock */
	k_sem_init(&data->lock, 0, 1);

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C device %s is not ready", cfg->i2c.bus->name);
		rc = -ENODEV;
		goto out;
	}
	
	/* Configure vcc gpio */
	if (cfg->vcc_gpios.port != NULL) {

		if (!gpio_is_ready_dt(&cfg->vcc_gpios)) {
			LOG_ERR("Port device %s is not ready",
				cfg->vcc_gpios.port->name);
			rc = -ENODEV;
			goto out;
		}
		
		rc = gpio_pin_configure_dt(&cfg->vcc_gpios, GPIO_OUTPUT_HIGH);
		if (rc < 0) {
			goto out;
		}
	}
	
	rc = read_time(dev, &current_time);
	if (rc < 0) {
		goto out;
	}
	
	rc = set_day_of_week(dev, &current_time);
	if (rc < 0) {
		goto out;
	}

	/* Set 24-hour time */
	data->registers.rtc_hours.twelve_hr = false;
	rc = write_register(dev, REG_RTC_HOUR,
		*((uint8_t *)(&data->registers.rtc_hours)));
	if (rc < 0) {
		goto out;
	}
	
	/* Configure alarm interrupt gpio */
	if (cfg->int_gpios.port != NULL) {

		if (!gpio_is_ready_dt(&cfg->int_gpios)) {
			LOG_ERR("Port device %s is not ready",
				cfg->int_gpios.port->name);
			rc = -ENODEV;
			goto out;
		}

		data->mcp7940n = dev;
		k_work_init(&data->alarm_work, mcp7940n_work_handler);
		
		rc = gpio_pin_configure_dt(&cfg->int_gpios, GPIO_INPUT);
		if (rc < 0) {
			goto out;
		}

		rc = gpio_pin_interrupt_configure_dt(&cfg->int_gpios,
							GPIO_INT_LEVEL_ACTIVE);//GPIO_INT_EDGE_TO_ACTIVE);//
		if (rc < 0) {
			goto out;
		}


		gpio_init_callback(&data->int_callback, mcp7940n_init_cb, BIT(cfg->int_gpios.pin));
		gpio_add_callback(cfg->int_gpios.port, &data->int_callback);
	
		/* Configure interrupt polarity */
		if ((cfg->int_gpios.dt_flags & GPIO_ACTIVE_LOW) == GPIO_ACTIVE_LOW) {
			data->int_active_high = false;
		} else {
			data->int_active_high = true;
		}
		data->alm0_registers.alm_weekday.alm_if = 0;
		data->alm0_registers.alm_weekday.alm_pol = data->int_active_high;
		data->alm1_registers.alm_weekday.alm_if = 0;
		data->alm1_registers.alm_weekday.alm_pol = data->int_active_high;
		rc = write_register(dev, REG_ALM0_WDAY,
					*((uint8_t *)(&data->alm0_registers.alm_weekday)));
		rc = write_register(dev, REG_ALM1_WDAY,
					*((uint8_t *)(&data->alm1_registers.alm_weekday)));
					
		LOG_DBG("Alarm set");
	}
	
out:
	k_sem_give(&data->lock);
	
	return 0;
}

#define MCP7940N_INIT(inst)                                                                         \
	static const struct mcp7940n_config mcp7940n_config_##inst = {                               \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.vcc_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, vcc_gpios, {0}),						 \
		/*IF_ENABLED(MCP7940N_INT_GPIOS_IN_USE,*/                                              \
		/*(*/.int_gpios = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0})/*))*/                         \
		};                                                                                 \
                                                                                                   \
	static struct mcp7940n_data mcp7940n_data_##inst;                                            \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &mcp7940n_init, NULL,                                           \
			      &mcp7940n_data_##inst, 		\
				  &mcp7940n_config_##inst, 		\
				  POST_KERNEL,           \
			      CONFIG_RTC_INIT_PRIORITY, 	\
				  &mcp7940n_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MCP7940N_INIT)
