#ifndef MLX90632_ZEPHYR_H
#define MLX90632_ZEPHYR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Initialize MLX90632 (checks device + reads calibration constants). */
int32_t mlx90632_zephyr_init(void);

/** Sample ambient/object temperature.
 *
 * @param[out] ambient_c Ambient temperature in degrees Celsius
 * @param[out] object_c  Object temperature in degrees Celsius
 */
int32_t mlx90632_zephyr_sample(double *ambient_c, double *object_c);

#ifdef __cplusplus
}
#endif

#endif /* MLX90632_ZEPHYR_H */
