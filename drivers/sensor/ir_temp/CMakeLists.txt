# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(
	mlx90632.c
	mlx90632_extended_meas.c
	mlx90632_depends_zephyr.c
	mlx90632_zephyr.c
)
zephyr_include_directories(.)
