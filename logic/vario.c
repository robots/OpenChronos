/*
    Vario function for ez430 chronos watch.
    Copyright (C) 2010 Marc Poulhi�s <dkm@kataplop.net>

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
//	Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
//	 
//	 
//	  Redistribution and use in source and binary forms, with or without 
//	  modification, are permitted provided that the following conditions 
//	  are met:
//	
//	    Redistributions of source code must retain the above copyright 
//	    notice, this list of conditions and the following disclaimer.
//	 
//	    Redistributions in binary form must reproduce the above copyright
//	    notice, this list of conditions and the following disclaimer in the 
//	    documentation and/or other materials provided with the   
//	    distribution.
//	 
//	    Neither the name of Texas Instruments Incorporated nor the names of
//	    its contributors may be used to endorse or promote products derived
//	    from this software without specific prior written permission.
//	
//	  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//	  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//	  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//	  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//	  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//	  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//	  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//	  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//	  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//	  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//	  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "project.h"

#ifdef CONFIG_VARIO

// driver
#include "altitude.h"
#include "display.h"
#include "vti_ps.h"
#include "ports.h"
#include "timer.h"

#include "stopwatch.h"

// logic
#include "user.h"
#include "vario.h"

#include "menu.h"

struct vario svario;

void hist_add(s16 alt);
u8 hist_ready(void);

#define HIST_GET_OLD() svario.hist_alts[(svario.hist_pos+1)%VARIO_HIST_SIZE]
#define HIST_GET_NEW() svario.hist_alts[svario.hist_pos]

/**
 * called every sec
 */
void vario_tick()
{
  if (is_altitude_measurement()){
    hist_add(sAlt.altitude);
    if (!hist_ready()){
        display_symbol(LCD_ICON_RECORD, SEG_ON);
    } else {
        display_symbol(LCD_ICON_RECORD, SEG_ON_BLINK_ON);
    }
  }
  display_vario(0, 0);
}

u8 is_vario(void)
{
  return (svario.state == VARIO_RUN &&  (ptrMenu_L2 == &menu_L2_Vario));
}

void update_vario()
{
}

static u8 fast = 0;

void start_vario()
{
  fast = !fast;
  svario.state = VARIO_RUN;


  if(fast) {
   ps_start_fast();
  } else {
   ps_start();
  }

}

void stop_vario()
{
  display_symbol(LCD_ICON_HEART, SEG_OFF);
  reset_vario();

  // Call draw routine immediately
  display_vario(LINE2, DISPLAY_LINE_UPDATE_FULL);
  if(is_altitude_measurement()) ps_start(); // restore low power mode
}

void sx_vario(u8 line)
{
  if (button.flag.down)
    {
      if (svario.state == VARIO_STOP){
        start_vario();
      } else {
        stop_vario();
      }
    }
}

void mx_vario(u8 line)
{
}

void display_vario(u8 line, u8 update)
{

  if (svario.state == VARIO_STOP) {
    display_chars(LCD_SEG_L2_5_0, (u8*) " VARIO", SEG_ON);
  } else if (is_altitude_measurement()){
    if (!hist_ready()) {
      if(fast)
        display_chars(LCD_SEG_L2_5_0, (u8*) "  FAST", SEG_ON);
          else
        display_chars(LCD_SEG_L2_5_0, (u8*) "  SLOW", SEG_ON);
    } else {
      u8 *str;

      s16 diff = HIST_GET_OLD() - HIST_GET_NEW();
      u8 is_neg = 0;
      u8 i;

      if (diff == 0){
        display_chars(LCD_SEG_L2_5_0, (u8*) "     0", SEG_ON);
      } else {
        if (diff < 0){
          is_neg = 1;
          diff = diff*(-1);
        }
        diff = diff / VARIO_HIST_SIZE;

        str = itoa(diff, 6, 7);

        for (i=0; i<7; i++){
          if (str[i] == '0' || str[i] == ' '){
            if (is_neg)
              str[i] = '-';
            else
              str[i] = ' ';
          } else {
            break;
          }
        }
        display_chars(LCD_SEG_L2_5_0, str, SEG_ON);
      }
    }
  } else {
    display_chars(LCD_SEG_L2_5_0, (u8*) " NOALT", SEG_ON);
  }
}

u8 hist_ready(void) {
  return (svario.hist_count == VARIO_HIST_SIZE);
}

/* s8 hist_size(void) { */
/*   if (svario.previous_end == -1) return 0; */
/*   if (svario.previous_end == svario.previous_start) return VARIO_HIST_SIZE; */
/*   return ((svario.previous_end-svario.previous_start)>0 ? VARIO_HIST_SIZE-svario.previous_end+svario.previous_start : svario.previous_start-svario.previous_end); */
/* } */

void hist_add(s16 alt) {
  if (svario.hist_count != VARIO_HIST_SIZE) svario.hist_count++;

  svario.hist_alts[svario.hist_pos] = alt;
  svario.hist_pos = (svario.hist_pos+1)%VARIO_HIST_SIZE;
}

void reset_vario(void)
{
  svario.state = VARIO_STOP;
  svario.hist_pos = svario.hist_count = 0;
  display_symbol(LCD_ICON_RECORD, SEG_OFF);
}

#endif /* CONFIG_VARIO */
