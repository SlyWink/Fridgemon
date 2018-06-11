#define F_CPU 1000000L

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

#define ADC_SAMPLES_COUNT  24
#define ADC_SAMPLES_REJECT 8

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
  WDTCR &= ~(_BV(WDCE) |_BV(WDE) | _BV(WDIE)) ;
}


void Beep_Sleep(void) {
  PORTB |= _BV(BUZZER_PIN) ;
  Delay_Sleep() ;
  PORTB &= ~_BV(BUZZER_PIN) ;
}


void Sort_Samples(uint16_t *p_val) {
  uint8_t l_count ;
  uint16_t l_tmp ;

  do {
    l_tmp = 0xFFFF ;
    for (l_count=0 ; l_count<ADC_SAMPLES_COUNT-1 ; l_count++)
      if (p_val[l_count] > p_val[l_count+1]) {
        l_tmp = p_val[l_count] ;
        p_val[l_count] = p_val[l_count+1] ;
        p_val[l_count+1] = l_tmp ;
      }
  } while (l_tmp < 0xFFFF) ; // ADC value is only 0x3FF max
}


uint16_t Read_ADC_Mux(uint8_t p_mask) {
  static uint16_t l_samples[ADC_SAMPLES_COUNT] ;
  static uint16_t l_adcval ;
  static int8_t l_count ;

  set_sleep_mode(SLEEP_MODE_ADC) ;
  ADMUX = p_mask ;
  ADCSRA |= _BV(ADEN) | _BV(ADIE) ; // ADC enable, interrupt
  sei() ;
  for (l_count = -1 ; l_count < ADC_SAMPLES_COUNT ; l_count++) {
    ADCSRA |= _BV(ADSC) ; // Start new conversion
    sleep_mode() ; // Sleep until ADC interrupt
    l_adcval = ADCL | (ADCH << 8) ;
    if (l_count >= 0) l_samples[l_count] = l_adcval ; // Drops first sample
  }
  cli() ;
  ADCSRA &= ~(_BV(ADEN) | _BV(ADIE)) ; // ADC disable, no interrupt

  Sort_Samples(l_samples) ;
  l_adcval = 0 ;
  // Reject min and max values and average the remaining
  for (l_count = ADC_SAMPLES_REJECT/2 ; l_count < ADC_SAMPLES_COUNT-ADC_SAMPLES_REJECT/2 ; l_count++)
    l_adcval += l_samples[l_count] ;
  return l_adcval / (ADC_SAMPLES_COUNT-ADC_SAMPLES_REJECT) ; // Average result
}


int main(void) {
  uint8_t l_elapsed = 0 ;
  uint8_t l_hotcount = 0 ;
  uint16_t l_lowtemp = 0xFF00 ; // Not 0xFFFF to keep some place for first comparison
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

  }
}
