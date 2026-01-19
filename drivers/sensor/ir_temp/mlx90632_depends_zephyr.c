#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>

#include <errno.h>

#include "mlx90632_depends.h"

#if DT_HAS_ALIAS(mlx90632)
#define MLX90632_USE_I2C_DT_SPEC 1
static const struct i2c_dt_spec mlx90632_i2c = I2C_DT_SPEC_GET(DT_ALIAS(mlx90632));
#else
/* Fallback (keeps builds working even if overlay wasn't picked up yet):
 * use Thingy:91 X's i2c2 bus and the typical MLX90632 address (0x3a).
 */
#define MLX90632_USE_I2C_DT_SPEC 0
#if DT_NODE_EXISTS(DT_NODELABEL(i2c2))
static const struct device *const mlx90632_bus = DEVICE_DT_GET(DT_NODELABEL(i2c2));
#define MLX90632_I2C_ADDR 0x3a
#else
static const struct device *const mlx90632_bus = NULL;
#define MLX90632_I2C_ADDR 0x3a
#endif
#endif

int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value)
{
	if (value == NULL) {
		return -EINVAL;
	}

	uint8_t reg_be[2] = {
		(uint8_t)((uint16_t)register_address >> 8),
		(uint8_t)((uint16_t)register_address & 0xFF),
	};
	uint8_t rx[2];

	int ret;
#if MLX90632_USE_I2C_DT_SPEC
	if (!device_is_ready(mlx90632_i2c.bus)) {
		return -ENODEV;
	}
	ret = i2c_write_read_dt(&mlx90632_i2c, reg_be, sizeof(reg_be), rx, sizeof(rx));
#else
	if (mlx90632_bus == NULL || !device_is_ready(mlx90632_bus)) {
		return -ENODEV;
	}
	ret = i2c_write_read(mlx90632_bus, MLX90632_I2C_ADDR, reg_be, sizeof(reg_be), rx, sizeof(rx));
#endif
	if (ret != 0) {
		return ret;
	}

	*value = sys_get_be16(rx);
	return 0;
}

int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value)
{
	uint8_t tx[4] = {
		(uint8_t)((uint16_t)register_address >> 8),
		(uint8_t)((uint16_t)register_address & 0xFF),
		(uint8_t)((uint16_t)value >> 8),
		(uint8_t)((uint16_t)value & 0xFF),
	};

#if MLX90632_USE_I2C_DT_SPEC
	if (!device_is_ready(mlx90632_i2c.bus)) {
		return -ENODEV;
	}
	return i2c_write_dt(&mlx90632_i2c, tx, sizeof(tx));
#else
	if (mlx90632_bus == NULL || !device_is_ready(mlx90632_bus)) {
		return -ENODEV;
	}
	return i2c_write(mlx90632_bus, tx, sizeof(tx), MLX90632_I2C_ADDR);
#endif
}

void usleep(int min_range, int max_range)
{
	int usec = min_range;
	if (max_range > min_range) {
		usec = (min_range + max_range) / 2;
	}
	if (usec < 0) {
		usec = 0;
	}
	k_usleep((uint32_t)usec);
}

void msleep(int msecs)
{
	if (msecs < 0) {
		msecs = 0;
	}
	k_msleep((uint32_t)msecs);
}
