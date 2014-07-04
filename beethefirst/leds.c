/* Copyright (c) 2011-2014 BEEVC - Electronic Systems        */
/*
 * This file is part of BEESOFT software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version. BEESOFT is
 * distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public
 * License for more details. You should have received a copy of the
 * GNU General Public License along with BEESOFT. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

#include "leds.h"

void led_tick(){
  int output =0;
  if(led_mode==0){
      pwm_set_duty_cycle(2, 0);
      pwm_set_enable(2);
  }else if(led_mode==1){
      pwm_set_duty_cycle(2, 100);
      pwm_set_enable(2);
  }else if(led_mode==2){
      freq_counter++;
      pwm_set_duty_cycle(2, 100);
      pwm_set_enable(2);
      if(freq_counter >= 20){
          led_mode=3;
          freq_counter = 0;
      }
  }else if(led_mode==3){
      freq_counter++;
      pwm_set_duty_cycle(2, 0);
      pwm_set_enable(2);
      if(freq_counter >= 20){
          led_mode=2;
          freq_counter = 0;
      }
  }else if(led_mode>=173){
      //pwm_set_duty_cycle(2, 0);
      //pwm_set_enable(2);
      led_mode=24;

  }else if((led_mode)>93&&(led_mode)<163){
      output = 183-led_mode;
      pwm_set_duty_cycle(2, output);
      pwm_set_enable(2);
      led_mode+=1;
}else{
      output = led_mode-3;
      pwm_set_duty_cycle(2, output);
      pwm_set_enable(2);
      led_mode+=1;
}

}
void set_led_mode(int mode){
  if(mode<0){
      return;
  }else{
      led_mode = mode;
  }
}
