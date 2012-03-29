/*
 * Copyright (C) 2012  Lothar Wassmann <LW@KARO-electronics.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
enum MXS_LRADC_TS_STATE {
	STATE_DISABLE,
	STATE_DETECT,
	STATE_PENDOWN,
	STATE_IDLE,
	STATE_MEASURE_X,
	STATE_MEASURE_Y,
	STATE_MEASURE_P,
	STATE_DONE,
};

