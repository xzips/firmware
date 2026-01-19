#include <zephyr/kernel.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include "mlx90632.h"
#include "mlx90632_depends.h"
#include "mlx90632_zephyr.h"

struct mlx90632_calib {
	int32_t P_T;
	int32_t P_R;
	int32_t P_G;
	int32_t P_O;
	int16_t Gb;

	int16_t Ka;
	int32_t Ea;
	int32_t Eb;
	int32_t Ga;
	int32_t Fa;
	int32_t Fb;
	int16_t Ha;
	int16_t Hb;
};

static struct mlx90632_calib calib;
static bool calib_ready;

static int32_t read_s16(uint16_t reg, int16_t *out)
{
	uint16_t v;
	int32_t ret = mlx90632_i2c_read((int16_t)reg, &v);
	if (ret < 0) {
		return ret;
	}
	*out = (int16_t)v;
	return 0;
}

static int32_t read_s32(uint16_t reg, int32_t *out)
{
	uint16_t lo;
	uint16_t hi;
	int32_t ret = mlx90632_i2c_read((int16_t)reg, &lo);
	if (ret < 0) {
		return ret;
	}
	ret = mlx90632_i2c_read((int16_t)(reg + 1), &hi);
	if (ret < 0) {
		return ret;
	}

	uint32_t u = ((uint32_t)hi << 16) | (uint32_t)lo;
	*out = (int32_t)u;
	return 0;
}

static int32_t mlx90632_read_calibration(struct mlx90632_calib *c)
{
	int32_t ret;

	ret = read_s32(MLX90632_EE_P_T, &c->P_T);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_P_R, &c->P_R);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_P_G, &c->P_G);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_P_O, &c->P_O);
	if (ret < 0) return ret;
	ret = read_s16(MLX90632_EE_Gb, &c->Gb);
	if (ret < 0) return ret;

    //printk("MLX90632 halfway through read calibration\n");

	ret = read_s16(MLX90632_EE_Ka, &c->Ka);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_Ea, &c->Ea);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_Eb, &c->Eb);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_Ga, &c->Ga);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_Fa, &c->Fa);
	if (ret < 0) return ret;
	ret = read_s32(MLX90632_EE_Fb, &c->Fb);
	if (ret < 0) return ret;
	ret = read_s16(MLX90632_EE_Ha, &c->Ha);
	if (ret < 0) return ret;
	ret = read_s16(MLX90632_EE_Hb, &c->Hb);
	if (ret < 0) return ret;

	return 0;
}

int32_t mlx90632_zephyr_init(void)
{
    //printk("MLX90632 init entry\n");

	int32_t ret = mlx90632_init();
	if (ret < 0) {
		return ret;
	}


   //printk("MLX90632 init reading calibration\n");
	ret = mlx90632_read_calibration(&calib);
	if (ret < 0) {
		return ret;
	}

    //printk("MLX90632 init setting emissivity\n");

	mlx90632_set_emissivity(1.0);
	calib_ready = true;
	return 0;
}

int32_t mlx90632_zephyr_sample(double *ambient_c, double *object_c)
{
	if (ambient_c == NULL || object_c == NULL) {
		return -EINVAL;
	}
	if (!calib_ready) {
		return -EACCES;
	}

	int16_t amb_new;
	int16_t amb_old;
	int16_t obj_new;
	int16_t obj_old;

	int32_t ret = mlx90632_read_temp_raw(&amb_new, &amb_old, &obj_new, &obj_old);
	if (ret < 0) {
		return ret;
	}

	double amb_raw = mlx90632_preprocess_temp_ambient(amb_new, amb_old, calib.Gb);
	double obj_raw = mlx90632_preprocess_temp_object(obj_new, obj_old, amb_new, amb_old, calib.Ka);

	*ambient_c = mlx90632_calc_temp_ambient(amb_new, amb_old, calib.P_T, calib.P_R, calib.P_G, calib.P_O, calib.Gb);
	*object_c = mlx90632_calc_temp_object((int32_t)obj_raw, (int32_t)amb_raw,
					calib.Ea, calib.Eb, calib.Ga, calib.Fa, calib.Fb, calib.Ha, calib.Hb);

	return 0;
}
