// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for Maxim MAX25014
 *
 * Copyright (C) 2024 GOcontroll B.V.
 *      Maud Spierings <maudspierings@gocontroll.com>
 */

#ifndef _MAX25014_H
#define _MAX25014_H

/**
 * struct max25014_platform_data
 * @name : Backlight driver name. If it is not defined, default name is set.
 * @initial_brightness : Initial value of the backlight brightness.
 * @iset : Value of the iset field which scales the amperage/limits it.
 * @strings : Which, out of four, led strings are in use.
 */
struct max25014_platform_data {
	const char *name;
	uint32_t initial_brightness;
	uint32_t iset;
	uint32_t strings[4];
};

#endif
