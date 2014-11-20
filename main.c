/*
 * This file is part of StormLoader, the Storm Bootloader
 *
 * StormLoader is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * StormLoader is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with StormLoader.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2014, Michael Andersen <m.andersen@eecs.berkeley.edu>
 */

#include <ioport.h>
extern void bl_init(void);
extern void bl_testloop(void);
extern void bl_loop_poll(void);
extern void bl_spi_test_loop(void);

#include <asf.h>
#include <board.h>
#include <conf_board.h>
#include <wdt_sam4l.h>
#include <sysclk.h>

#include "ASF/common/services/ioport/sam/ioport_gpio.h"
#include "ASF/common/services/ioport/ioport.h"
void board_init(void)
{
	struct wdt_dev_inst wdt_inst;
	struct wdt_config   wdt_cfg;
	wdt_get_config_defaults(&wdt_cfg);
	wdt_init(&wdt_inst, WDT, &wdt_cfg);
	wdt_disable(&wdt_inst);

	/* Initialize IOPORT */
	ioport_init();

    /* There is a problem with SB.01 relating to the LCDA power rail, 
     * this is a workaround. See http://storm.pm/msg/SB-001
     */
	ioport_set_pin_dir(PIN_PA14, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(PIN_PA14, IOPORT_PIN_LEVEL_HIGH);//
	ioport_set_pin_mode(PIN_PA14,IOPORT_MODE_DRIVE_STRENGTH);
	
	ioport_set_pin_dir(PIN_PA19, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(PIN_PB06, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(PIN_PB06, IOPORT_MODE_PULLUP | IOPORT_MODE_GLITCH_FILTER);
	sysclk_init();	
}

extern void jump_into_user_code(void)  __attribute__((noreturn));

int main (void)
{
    board_init();
    // Verify BL policy
    uint32_t active = 0;
    uint32_t inactive = 0;
    uint32_t samples = 10000;
    while(samples)
    {
        if (ioport_get_pin_level(PIN_PB06) == 0) active++;
        else inactive++;
        samples--;
    }
    
    if (active > inactive)
    {
        bl_init();
        while (1) 
        {
            bl_loop_poll();
        }
    }
    else
    {
        ioport_set_pin_dir(PIN_PB08, IOPORT_DIR_OUTPUT);
        ioport_set_pin_level(PIN_PB08, 0);
        ioport_set_pin_level(PIN_PB08, 1);
        ioport_set_pin_level(PIN_PB08, 0);
        jump_into_user_code();
        ioport_set_pin_level(PIN_PB08, 0);
        ioport_set_pin_level(PIN_PB08, 1);
        ioport_set_pin_level(PIN_PB08, 0);
    }
}
