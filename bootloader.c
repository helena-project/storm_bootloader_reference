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
 
#include <usart.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <flashcalw.h>
#include <spi.h>

#include "info.h"

#include "bootloader.h"

#include "ASF/common/services/clock/sam4l/sysclk.h"
#include "ASF/common/services/ioport/ioport.h"

uint8_t byte_escape;
uint8_t tx_stage_ram [TXBUFSZ];
uint16_t tx_ptr;
uint16_t tx_left;
uint8_t rx_stage_ram [RXBUFSZ];
uint16_t rx_ptr;

const sam_usart_opt_t bl_settings = {
     115200,
     US_MR_CHRL_8_BIT,
     US_MR_PAR_NO, //TODO change
     US_MR_NBSTOP_1_BIT,
     US_MR_CHMODE_NORMAL
};
    


void bl_init(void)
{
    byte_escape = 0;

    tx_ptr = tx_left = 0;
    tx_ptr = tx_left = 0;
    rx_ptr = 0;
    
    //Enable BL USART
    ioport_set_pin_mode(PIN_PB10A_USART3_TXD, MUX_PB10A_USART3_TXD);
    ioport_disable_pin(PIN_PB10A_USART3_TXD);
    ioport_set_pin_mode(PIN_PB09A_USART3_RXD, MUX_PB09A_USART3_RXD);
    ioport_disable_pin(PIN_PB09A_USART3_RXD);
    sysclk_enable_peripheral_clock(USART3);
    usart_reset(USART3);
    usart_init_rs232(USART3, &bl_settings, sysclk_get_main_hz());
    usart_enable_tx(USART3);
    usart_enable_rx(USART3);

    //Enable flash SPI
    ioport_set_pin_mode(PIN_PC05A_SPI_MOSI, MUX_PC05A_SPI_MOSI);
    ioport_disable_pin(PIN_PC05A_SPI_MOSI);
    ioport_set_pin_mode(PIN_PC04A_SPI_MISO, MUX_PC04A_SPI_MISO);
    ioport_disable_pin(PIN_PC04A_SPI_MISO);
    ioport_set_pin_mode(PIN_PC06A_SPI_SCK, MUX_PC06A_SPI_SCK);
    ioport_disable_pin(PIN_PC06A_SPI_SCK);
    ioport_set_pin_mode(PIN_PC03A_SPI_NPCS0, MUX_PC03A_SPI_NPCS0); //FL CS
    ioport_disable_pin(PIN_PC03A_SPI_NPCS0);
    ioport_set_pin_mode(PIN_PC01A_SPI_NPCS3, MUX_PC01A_SPI_NPCS3); //RAD CS
    ioport_disable_pin(PIN_PC01A_SPI_NPCS3);
    spi_enable_clock(SPI);
    spi_reset(SPI);
    spi_set_master_mode(SPI);
    spi_disable_mode_fault_detect(SPI);
    spi_disable_loopback(SPI);
    spi_set_peripheral_chip_select_value(SPI, spi_get_pcs(0));
    //spi_set_fixed_peripheral_select(SPI);
    spi_set_variable_peripheral_select(SPI);
    spi_disable_peripheral_select_decode(SPI);
    spi_set_delay_between_chip_select(SPI, 0);
    spi_set_transfer_delay(SPI, 0, FLASH_POSTCS_DELAY, FLASH_IXF_DELAY);
    spi_set_bits_per_transfer(SPI, 0, 8);
    spi_set_baudrate_div(SPI, 0, spi_calc_baudrate_div(FLASH_BAUD_RATE, sysclk_get_cpu_hz()));
    spi_configure_cs_behavior(SPI, 0, SPI_CS_KEEP_LOW);
    spi_set_clock_polarity(SPI, 0, 1); //SPI mode 3
    spi_set_clock_phase(SPI, 0, 0);
    spi_enable(SPI);
    
    //Test pin toggle
    ioport_set_pin_dir(PIN_PB08, IOPORT_DIR_OUTPUT);
    
    
}


uint16_t bl_spi_xfer(uint8_t* send, uint8_t* recv, uint16_t len)
{
    uint16_t i;
    uint16_t recvd = 0;
    uint32_t timeout;
    //Throw away existing buffer
    while(spi_is_rx_full(SPI))
    {
        spi_get(SPI);
    }
    for (i = 0; i < len; i++)
    {
        for(timeout = 100000; timeout > 0 && !spi_is_tx_ready(SPI);timeout--);
        spi_write(SPI, send[i], spi_get_pcs(0), i==(len-1));
        for(timeout = 100000; timeout > 0 && !spi_is_rx_ready(SPI);timeout--);
        uint16_t dw;
        dw = spi_get(SPI);
        recv[recvd] = (uint8_t) dw;
        recvd ++;
    }

    return recvd;
}

uint16_t bl_spi_tx(uint8_t* send, uint16_t len, uint8_t do_end)
{
    uint16_t i;
    uint16_t recvd = 0;
    uint32_t timeout;
    for (i = 0; i < len; i++)
    {
        for(timeout = 100000; timeout > 0 && !spi_is_tx_empty(SPI);timeout--);
        spi_write(SPI, send[i], spi_get_pcs(0), (i==(len-1))&&do_end);
    }
    for(timeout = 100000; timeout > 0 && !spi_is_tx_ready(SPI);timeout--);

    return recvd;
}

uint16_t bl_spi_rx(uint8_t* recv, uint16_t len)
{
    uint16_t i;
    uint16_t recvd = 0;
    uint32_t timeout;
    //Throw away existing buffer
    while(spi_is_rx_full(SPI))
    {
        spi_get(SPI);
    }
    for (i = 0; i < len; i++)
    {
        for(timeout = 100000; timeout > 0 && !spi_is_tx_ready(SPI);timeout--);
        spi_write(SPI, 0x00, spi_get_pcs(0), i==(len-1));
        for(timeout = 100000; timeout > 0 && !spi_is_rx_ready(SPI);timeout--);
        uint16_t dw;
        dw = spi_get(SPI);
        recv[recvd] = (uint8_t) dw;
        recvd ++;
    }
    return recvd;
}

void bl_loop_poll(void)
{
    if (usart_is_rx_ready(USART3))
    {
        
        uint32_t ch;
        usart_getchar(USART3, &ch);
        if (rx_ptr == RXBUFSZ)
        {
            tx_ptr = 0;
            tx_left = 1;
            tx_stage_ram[0] = RES_OVERFLOW;
        }
        else
        {
            bl_rxb(ch);
        }
    }
    if (usart_is_tx_ready(USART3))
    {
        if (tx_left > 0)
        {
            bl_txb(tx_stage_ram[tx_ptr++]);
            tx_left--;
        }
    }
}

void bl_txb(uint8_t b)
{
    usart_putchar(USART3, b);
}

void bl_rxb(uint8_t b)
{
    if (byte_escape && b == ESCAPE_CHAR)
    {
        byte_escape = 0;
        rx_stage_ram[rx_ptr++] = b;
    } 
    else if (byte_escape)
    {
        bl_cmd(b);
        byte_escape = 0;
    }
    else if (b == ESCAPE_CHAR)
    {
        byte_escape = 1;
    }
    else
    {
        rx_stage_ram[rx_ptr++] = b;
    }
    
}

void bl_cmd(uint8_t b)
{
    switch(b)
    {
        case CMD_PING:
            bl_c_ping();
            break;
        case CMD_INFO:
            bl_c_info();
            break;
        case CMD_ID:
            bl_c_id();
            break;
        case CMD_RESET:
            bl_c_reset();
            break;
        case CMD_WPAGE:
            bl_c_wpage();
            break;
        case CMD_EPAGE:
            bl_c_epage();
            break;
        case CMD_XEBLOCK:
            bl_c_xeblock();
            break;
        case CMD_XWPAGE:
            bl_c_xwpage();
            break;
        case CMD_CRCRX:
            bl_c_crcrx();
            break;
        case CMD_RRANGE:
            bl_c_rrange();
            break;
        case CMD_XRRANGE:
            bl_c_xrrange();
            break;
        case CMD_SATTR:
            bl_c_sattr();
            break;
        case CMD_GATTR:
            bl_c_gattr();
            break;
        case CMD_CRCIF:
            bl_c_crcif();
            break;
        case CMD_CRCEF:
            bl_c_crcef();
            break;
        case CMD_XEPAGE:
            bl_c_xepage();
            break;        
        case CMD_XFINIT:
            bl_c_xfinit();
            break;
        default:
            bl_c_unknown();
            break;
    }
}


