/*

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// *************************************************************************************************
//
//      Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
//       
//       
//        Redistribution and use in source and binary forms, with or without 
//        modification, are permitted provided that the following conditions 
//        are met:
//      
//          Redistributions of source code must retain the above copyright 
//          notice, this list of conditions and the following disclaimer.
//       
//          Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the 
//          documentation and/or other materials provided with the   
//          distribution.
//       
//          Neither the name of Texas Instruments Incorporated nor the names of
//          its contributors may be used to endorse or promote products derived
//          from this software without specific prior written permission.
//      
//        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//        LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//        A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//        OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//        SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//        LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//        DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//        THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//        (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//        OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "project.h"

#ifdef CONFIG_GATE

// driver
#include "altitude.h"
#include "display.h"
#include "vti_ps.h"
#include "ports.h"
#include "timer.h"

#include "stopwatch.h"

// logic
#include "user.h"
#include "radio.h"
#include "gate.h"

#include "menu.h"

#include "./simpliciti/Components/mrfi/radios/family5/mrfi_radio_interface.h"


/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */
#define GATE_ENABLE_SYNC_PIN_INT()                  (RF1AIE |= BV(9))
#define GATE_DISABLE_SYNC_PIN_INT()                 (RF1AIE &= ~BV(9))
#define GATE_CLEAR_SYNC_PIN_INT_FLAG()              (RF1AIFG &= ~BV(9))
#define GATE_SYNC_PIN_INT_IS_ENABLED()              (RF1AIE & BV(9))
#define GATE_SYNC_PIN_IS_HIGH()                     (RF1AIN & BV(9))
#define GATE_SYNC_PIN_INT_FLAG_IS_SET()             (RF1AIFG & BV(9))


#define GATE_CLEAR_PAPD_PIN_INT_FLAG()              (RF1AIFG &= ~BV(0))
#define GATE_PAPD_PIN_IS_HIGH()                     (RF1AIN & BV(0))
#define GATE_PAPD_INT_FLAG_IS_SET()                 (RF1AIFG & BV(0))


/* RSSI valid signal is available on the GDO_1 */
#define GATE_RSSI_VALID_WAIT()  while(!(RF1AIN & BV(1)));


/* Abstract radio interface calls. Could use these later to
 * merge code from similar radio but different interface.
 */
#define GATE_STROBE(cmd)                      mrfiRadioInterfaceCmdStrobe(cmd)
#define GATE_RADIO_REG_READ(reg)              mrfiRadioInterfaceReadReg(reg)
#define GATE_RADIO_REG_WRITE(reg, value)      mrfiRadioInterfaceWriteReg(reg, value)
#define GATE_RADIO_WRITE_TX_FIFO(pData, len)  mrfiRadioInterfaceWriteTxFifo(pData, len)
#define GATE_RADIO_READ_RX_FIFO(pData, len)   mrfiRadioInterfaceReadRxFifo(pData, len)


#define GATE_STROBE_IDLE_AND_WAIT()              \
{                                                \
  GATE_STROBE( SIDLE );                          \
    /* Wait for XOSC to be stable and radio in IDLE state */ \
      while (GATE_STROBE( SNOP ) & 0xF0) ;           \
 }

/* ------------------------------------------------------------------------------------------------
 *                                    Local Constants
 * ------------------------------------------------------------------------------------------------
 */
const uint8_t gateRadioCfg[][2] = {
  /* internal radio configuration */
  {IOCFG0, GATE_SETTING_IOCFG0},	/* Configure GDO_0 to output PA_PD signal (low during TX, high otherwise). */
  {IOCFG1, GATE_SETTING_IOCFG1},	/* Configure GDO_1 to output RSSI_VALID signal (high when RSSI is valid, low otherwise). */
  {MCSM1, GATE_SETTING_MCSM1},	/* CCA mode, RX_OFF_MODE and TX_OFF_MODE */
  {MCSM0, GATE_SETTING_MCSM0},	/* AUTO_CAL and XOSC state in sleep */
  {PKTLEN, GATE_SETTING_PKTLEN},
  {PKTCTRL0, GATE_SETTING_PKTCTRL0},
  {FIFOTHR, GATE_SETTING_FIFOTHR},

/* imported SmartRF radio configuration */

  {FSCTRL1, GATE_SETTING_FSCTRL1},
  {FSCTRL0, GATE_SETTING_FSCTRL0},
  {FREQ2, GATE_SETTING_FREQ2},
  {FREQ1, GATE_SETTING_FREQ1},
  {FREQ0, GATE_SETTING_FREQ0},
  {MDMCFG4, GATE_SETTING_MDMCFG4},
  {MDMCFG3, GATE_SETTING_MDMCFG3},
  {MDMCFG2, GATE_SETTING_MDMCFG2},
  {MDMCFG1, GATE_SETTING_MDMCFG1},
  {MDMCFG0, GATE_SETTING_MDMCFG0},
  {DEVIATN, GATE_SETTING_DEVIATN},
  {FOCCFG, GATE_SETTING_FOCCFG},
  {BSCFG, GATE_SETTING_BSCFG},
  {AGCCTRL2, GATE_SETTING_AGCCTRL2},
  {AGCCTRL1, GATE_SETTING_AGCCTRL1},
  {AGCCTRL0, GATE_SETTING_AGCCTRL0},
  {FREND1, GATE_SETTING_FREND1},
  {FREND0, GATE_SETTING_FREND0},
  {FSCAL3, GATE_SETTING_FSCAL3},
  {FSCAL2, GATE_SETTING_FSCAL2},
  {FSCAL1, GATE_SETTING_FSCAL1},
  {FSCAL0, GATE_SETTING_FSCAL0},
  {TEST2, GATE_SETTING_TEST2},
  {TEST1, GATE_SETTING_TEST1},
  {TEST0, GATE_SETTING_TEST0},
};


static void Gate_RxModeOff(void)
{
  /* disable receive interrupts */
  GATE_DISABLE_SYNC_PIN_INT();

  /* turn off radio */
  GATE_STROBE_IDLE_AND_WAIT();

  /* flush the receive FIFO of any residual data */
  GATE_STROBE( SFRX );

  /* clear receive interrupt */
  GATE_CLEAR_SYNC_PIN_INT_FLAG();
}


/**************************************************************************************************
 * @fn          GATE_Transmit
 *
 * @brief       Transmit a packet using CCA algorithm.
 *
 * @param       pPacket - pointer to packet to transmit
 *
 * @return      Return code indicates success or failure of transmit:
 *                  GATE_TX_RESULT_SUCCESS - transmit succeeded
 *                  GATE_TX_RESULT_FAILED  - transmit failed because CCA failed
 **************************************************************************************************
 */
uint8_t
GATE_Transmit (u8 * packet, u8 len)
{
  /* Turn off reciever. We can ignore/drop incoming packets during transmit. */
  Gate_RxModeOff ();

  /* compute number of bytes to write to transmit FIFO */

  /* ------------------------------------------------------------------
   *    Write packet to transmit FIFO
   *   --------------------------------
   */
  GATE_RADIO_WRITE_TX_FIFO (packet, len);


  /* ------------------------------------------------------------------
   *    Immediate transmit
   *   ---------------------
   */
  /* Issue the TX strobe. */
  GATE_STROBE (STX);

  /* Wait for transmit to complete */
  while (!GATE_SYNC_PIN_INT_FLAG_IS_SET ());

  /* Clear the interrupt flag */
  GATE_CLEAR_SYNC_PIN_INT_FLAG ();
  return 0;
}

void
gate_init ()
{
  /* Initialize the radio interface */
  mrfiRadioInterfaceInit ();

  /* Strobe Reset: Resets the radio and puts it in SLEEP state. */
  GATE_STROBE (SRES);

  /* Put radio in Idle state */
  GATE_STROBE_IDLE_AND_WAIT ();

  uint8_t i;

  for (i = 0; i < (sizeof (gateRadioCfg) / sizeof (gateRadioCfg[0])); i++)
    {
      GATE_RADIO_REG_WRITE (gateRadioCfg[i][0], gateRadioCfg[i][1]);
    }


   while( !(RF1AIFCTL1 & RFINSTRIFG));
   RF1AINSTRB = RF_SNOP;
   while( !(RF1AIFCTL1 & RFINSTRIFG));
   RF1AINSTRW = 0x7E00;                    /* PA Table write (burst) */
   while( !(RF1AIFCTL1 & RFINSTRIFG));
//   RF1AINSTRW = 0x7E50;                    /* PA Table write (burst) */
   RF1AINSTRW = 0x7EC6;                    /* PA Table write (burst) */
   while( !(RF1AIFCTL1 & RFINSTRIFG));
   RF1AINSTRB = RF_SNOP;
   while( !(RF1AIFCTL1 & RFINSTRIFG));


}

void
gate_tick ()
{
  display_gate (0, 0);
}

u8
is_gate (void)
{
//  return ((ptrMenu_L2 == &menu_L2_gate));
    return 0;
}

void
update_gate_timer ()
{
  /* TA0CCR2 = TA0CCR2 + STOPWATCH_1HZ_TICK; */
}

void
start_gate ()
{

  /* // Init CCR register with current time */
  /* TA0CCR2 = TA0R; */

  /* // Load CCR register with next capture time */
  /* update_gate_timer(); */

  /* // Reset IRQ flag     */
  /* TA0CCTL2 &= ~CCIFG;  */

  /* // Enable timer interrupt     */
  /* TA0CCTL2 |= CCIE;  */


  display_symbol (LCD_ICON_RECORD, SEG_ON);
}

void
stop_gate ()
{
  /* // Clear timer interrupt enable    */
  /* TA0CCTL2 &= ~CCIE;  */

  display_symbol (LCD_ICON_RECORD, SEG_OFF);

  // Call draw routine immediately
  display_gate (LINE2, DISPLAY_LINE_UPDATE_FULL);
}

u8 open[GATE_SETTING_PKTLEN] = { 0x92, 0xD9, 0x25, 0xB6, 0x58};
u8 blank[GATE_SETTING_PKTLEN] = { 0,0,0,0,0};

void
sx_gate (u8 line)
{
  u16 i;
  if (button.flag.down)
    {
	display_symbol (LCD_ICON_BEEPER1, SEG_ON);
	open_radio();
	gate_init();
	for(i=0;i<50;i++) {
	    GATE_Transmit(open, GATE_SETTING_PKTLEN);
	    Timer0_A4_Delay(CONV_MS_TO_TICKS(11));
	}
	close_radio();
	display_symbol (LCD_ICON_BEEPER1, SEG_OFF);
    }
  else
    {
    }

}

void
mx_gate (u8 line)
{
}

void
display_gate (u8 line, u8 update)
{
     display_chars(LCD_SEG_L2_5_0, (u8*) "  GATE", SEG_ON);
}

void
reset_gate (void)
{
}

#endif /* CONFIG_gate */
