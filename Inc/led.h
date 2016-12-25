/*
 * led.h
 *
 *  Created on: 2016/12/24
 *      Author: Yuki
 */

#ifndef LED_H_
#define LED_H_
#ifdef __cplusplus
 extern "C" {
#endif


typedef enum
{
  LED3 = 1 << 0,
  LED4 = 1 << 1,
  LED5 = 1 << 2,
  LED6 = 1 << 3
} Led_TypeDef;

void led_on(Led_TypeDef led);
void led_off(Led_TypeDef led);
void led_toggle(Led_TypeDef led);

#ifdef __cplusplus
}
#endif
#endif /* LED_H_ */
