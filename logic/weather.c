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

#ifdef CONFIG_WEATHER

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
#include "weather.h"

#include "menu.h"


void
weather_init ()
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
