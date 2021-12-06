/*
 * deca_device_ex.c
 *
 *  Created on: Jan 25, 2018
 *      Author: milad
 */

// compile this file instead of deca_device.c

#include "deca_device.c"

void dwt_setrxautoreenable(int enable)
{
    // Config system register
    pdw1000local->sysCFGreg = dwt_read32bitreg(SYS_CFG_ID) ; // Read sysconfig register

    // Disable smart power configuration
    if(enable)
    {
    	pdw1000local->sysCFGreg |= SYS_CFG_RXAUTR ;
    }
    else
    {
    	pdw1000local->sysCFGreg &= ~(SYS_CFG_RXAUTR) ;
    }

    dwt_write32bitreg(SYS_CFG_ID,pdw1000local->sysCFGreg) ;
}
