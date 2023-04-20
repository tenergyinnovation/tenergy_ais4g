/***********************************************************************
 * File         :     tiny32_v3_Lib.h
 * Description  :     Library for Hardware config and function for tiny32_v3 module
 * Author       :     Tenergy Innovation Co., Ltd.
 * Date         :     23 Nov 2021
 * Revision     :     1.0
 * Rev1.0       :     Original
 * website      :     http://www.tenergyinnovation.co.th
 * Email        :     uten.boonliam@innovation.co.th
 * TEL          :     089-140-7205
 ***********************************************************************/

#include "Arduino.h"

/**************************************/
/*           GPIO define              */
/**************************************/
#define RXD2    16
#define TXD2    17
#define RXD3    27
#define TXD3    26
#define SW1     34
#define SW2     35
#define RELAY1  25
#define RELAY2  32
#define LED_ONBOARD 15
#define SLID_SW 05
#define BUZZER 04
#define SW_IO0  0
#define SW_E18  18

static const uint8_t daysInMonth [] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };


 /***********************************************************************
 * FUNCTION:    void TickBuildinLED_blink(void){
 * DESCRIPTION: ON-OFF Buildin LED
 * PARAMETERS:  bool state
 * RETURNED:    nothing
 ***********************************************************************/
void TickBuildinLED_blink(void){
        bool _state = digitalRead(LED_ONBOARD);
        digitalWrite(BUILTIN_LED, !_state);
}