#ifdef DEBUG
#define F_CPU 1000000L
#include "dbginclude.c"
#include <util/delay.h>
#endif

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>

#define BUZZER_PIN PB2

#define FALSE 0
#define TRUE  1

#define SENSOR_DELAY 38 // 38 * 8s ~ 5mn

#define TOO_HOT_OFFSET 2
#define TOO_HOT_COUNT  2 // 2 * 5mn = 10mn

#define ADC_SAMPLES 32

#define LOW_BATTERY 402 // = 1.1v / 2.8v * 1024

#define TEMPERATURE_MASK  (_BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0) | _BV(REFS1))
#define BATTERY_MASK      (_BV(MUX3) | _BV(MUX2) )

#define Modify_Watchdog() WDTCR |= _BV(WDCE) | _BV(WDE)

EMPTY_INTERRUPT(WDT_vect) ;

EMPTY_INTERRUPT(ADC_vect) ;


void Set_Delay(uint8_t p_wdp) {
  if (p_wdp & 8) p_wdp = _BV(WDP3) | (p_wdp & 7) ;
  Modify_Watchdog() ;
  WDTCR &= ~(_BV(WDP3) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0)) ;
  Modify_Watchdog() ;
  WDTCR |= p_wdp ;
}


void Delay_Sleep(void) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN) ;
  WDTCR |= _BV(WDIE) | _BV(WDE) ;
  sei() ;
  sleep_mode() ;
  cli() ;
  Modify_Watchdog() ;
  WDTCR &= ~(_BV(WDCE) |_BV(WDE)) ;
}


void Beep_Sleep(void) {
  PORTB |= _BV(BUZZER_PIN) ;
  Delay_Sleep() ;
  PORTB &= ~_BV(BUZZER_PIN) ;
}


uint16_t Read_ADC_Mux(uint8_t p_mask) {
  uint16_t l_adcsum ;
  uint8_t l_count ;

  set_sleep_mode(SLEEP_MODE_ADC) ;
  ADMUX = p_mask ;
  ADCSRA |= _BV(ADEN) | _BV(ADIE) ; // ADC enable, interrupt
  sei() ;
  for (l_count = 0 ; l_count <= ADC_SAMPLES ; l_count++) {
    ADCSRA |= _BV(ADSC) ; // Start new conversion
    sleep_mode() ; // Sleep until ADC interrupt
    if (l_count)
      l_adcsum += ADCL | (ADCH << 8) ; // Drop first results, sum next
    else {
      l_adcsum = ADCH ; // Drop first result
      l_adcsum = 0 ;
    }
  }
  cli() ;
  ADCSRA &= ~(_BV(ADEN) | _BV(ADIE)) ; // ADC disable, no interrupt
  return l_adcsum / ADC_SAMPLES ; // Average result
}


int main(void) {
  uint8_t l_elapsed = 0 ;
  uint8_t l_hotcount = 0 ;
  uint16_t l_lowtemp = 0xFFFF ;
  uint16_t l_curtemp ;

  // IO port init
  DDRB = _BV(BUZZER_PIN) ; // Output pin for buzzer
  PORTB = ~_BV(BUZZER_PIN) ; // All other pins as input with pull-up
  // ADC init
  ADCSRA = _BV(ADPS1) | _BV(ADPS0) ; // Prescaler 8

  Set_Delay(WDTO_500MS) ; Delay_Sleep() ;

  // Warn device ready
  Set_Delay(WDTO_60MS) ;
  Beep_Sleep() ; Delay_Sleep() ; Beep_Sleep() ;

#ifdef DEBUG
OSCCAL += 10 ;
#endif

  for(;;) {

    Set_Delay(WDTO_8S) ; Delay_Sleep() ;

    // Check temperature and battery every 5mn
    if (l_elapsed % SENSOR_DELAY == 0) {
      l_curtemp = Read_ADC_Mux(TEMPERATURE_MASK) ;
      if (l_curtemp >= l_lowtemp + TOO_HOT_OFFSET)
        l_hotcount++ ;
      else {
        if (l_curtemp < l_lowtemp) l_lowtemp = l_curtemp ;
        l_hotcount = 0 ;
      }
      if (Read_ADC_Mux(BATTERY_MASK) >= LOW_BATTERY) { // Battery level too low
        Set_Delay(WDTO_250MS) ; Beep_Sleep() ;
      }
    }

    if (l_hotcount >= TOO_HOT_COUNT) { // Too hot for too long
      Set_Delay(WDTO_1S) ; Beep_Sleep() ;
      l_hotcount = TOO_HOT_COUNT ;
    }

    l_elapsed++ ;

#ifdef DEBUG
    Serial_Debug_Init() ;

    l_curtemp = Read_ADC_Mux(TEMPERATURE_MASK) ;
cli() ;
    Serial_Debug_Send(0xF1) ;
    _delay_ms(500) ;
    l_hotcount = (uint8_t) (l_curtemp >>8) ;
    Serial_Debug_Send(l_hotcount) ;
    _delay_ms(1000) ;
    l_hotcount = (uint8_t) (l_curtemp & 0xFF) ;
    Serial_Debug_Send(l_hotcount) ;
    _delay_ms(1000) ;

    l_curtemp = Read_ADC_Mux(BATTERY_MASK) ;
cli() ;
    Serial_Debug_Send(0xF2) ;
    _delay_ms(500) ;
    l_hotcount = (uint8_t) (l_curtemp >>8) ;
    Serial_Debug_Send(l_hotcount) ;
    _delay_ms(1000) ;
    l_hotcount = (uint8_t) (l_curtemp & 0xFF) ;
    Serial_Debug_Send(l_hotcount) ;
    _delay_ms(2000) ;
#endif

  }
}
