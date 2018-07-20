#define F_CPU 1000000L

#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define BUZZER_PIN PB0
#define BUTTON_PIN PB3

#define BOOLEAN uint8_t
#define FALSE 0
#define TRUE  1

#define SENSOR_DELAY 38 // 38 * 8s ~ 5mn

#define TOO_HOT_DELAY      4 // 4 * 8s ~ 30s
#define TOO_HOT_COUNT      2 // 2 * 5mn = 10mn
#define TOO_HOT_BEEPS      3
#define TOO_HOT_MIN_OFFSET 2 // Max degrees over average temperature
#define TOO_HOT_MAX_OFFSET (TOO_HOT_MIN_OFFSET + 4)

#define MAX_TEMPERATURE_COUNT 0x3F // Too keep temperature sum in 16 bits range

#define ADC_SAMPLES_COUNT  24
#define ADC_SAMPLES_REJECT 8

#define LOW_BATTERY 402 // = 1.1v / 2.8v * 1024

#define TEMPERATURE_MASK   (_BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0) | _BV(REFS1))
#define BATTERY_MASK       (_BV(MUX3) | _BV(MUX2) )

#define BUTTON_NONE  0
#define BUTTON_SHORT 1
#define BUTTON_LONG  2
#define BUTTON_LONG_DELAY 50 // 50 * 60MS ~ 3s

#define Enable_Button() GIMSK |= _BV(PCIE)
#define Disable_Button() GIMSK &= ~_BV(PCIE)
#define Button_Released() (PINB & _BV(BUTTON_PIN))

#define Modify_Watchdog() WDTCR |= _BV(WDCE) | _BV(WDE)

EMPTY_INTERRUPT(WDT_vect) ;

EMPTY_INTERRUPT(ADC_vect) ;

EMPTY_INTERRUPT(PCINT0_vect) ;


void Set_Delay(uint8_t p_wdp) {
  if (p_wdp & 8) p_wdp = _BV(WDP3) | (p_wdp & 7) ;
  Modify_Watchdog() ; WDTCR &= ~(_BV(WDP3) | _BV(WDP2) | _BV(WDP1) | _BV(WDP0)) ;
  Modify_Watchdog() ; WDTCR |= p_wdp ;
}


void Delay_Sleep(void) {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN) ;
  WDTCR |= _BV(WDIE) | _BV(WDE) ;
  sei() ;
  sleep_mode() ;
  cli() ;
  Modify_Watchdog() ; WDTCR &= ~(_BV(WDCE) |_BV(WDE) | _BV(WDIE)) ;
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

  power_adc_enable() ;
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
  power_adc_disable() ;

  Sort_Samples(l_samples) ;
  l_adcval = 0 ;
  // Reject min and max values and average the remaining
  for (l_count = ADC_SAMPLES_REJECT/2 ; l_count < ADC_SAMPLES_COUNT-ADC_SAMPLES_REJECT/2 ; l_count++)
    l_adcval += l_samples[l_count] ;
  return l_adcval / (ADC_SAMPLES_COUNT-ADC_SAMPLES_REJECT) ; // Average result
}


uint16_t Average_Temperature(uint16_t p_temp) {
  static uint16_t l_final = 0 ;
  static uint16_t l_tempsum = 0 ;
  static uint8_t l_tempcount = 0 ;
  uint16_t l_average ;

  if (l_final) return l_final ; // Already have a reliable average temperature
  l_tempsum += p_temp ; l_tempcount++ ;
  l_average = l_tempsum / l_tempcount ;
  if ((l_tempsum % l_tempcount) > (l_tempcount + 1) / 2) l_average++ ;
  if (l_tempcount == MAX_TEMPERATURE_COUNT) l_final = l_average ; // Stop computing average temperature
  return l_average ;
}


uint8_t Get_Hot_Offset(BOOLEAN p_next) {
  static uint8_t l_offset = TOO_HOT_MIN_OFFSET;

  if (p_next)
    if (++l_offset > TOO_HOT_MAX_OFFSET) l_offset = TOO_HOT_MIN_OFFSET ;
  return l_offset ;
}


void Beep_Repeat(uint8_t p_repeat, uint8_t p_wdpon, uint8_t p_wdpoff) {
  uint8_t l_count ;

  for (l_count=p_repeat ; ; l_count--) {
    Set_Delay(p_wdpon) ; Beep_Sleep() ;
    if (l_count == 1) break ;
    Set_Delay(p_wdpoff) ; Delay_Sleep() ;
  }
}


#define Beep_Too_Hot() Beep_Repeat(TOO_HOT_BEEPS,WDTO_250MS,WDTO_120MS)

#define Beep_Offset(p_offset) Beep_Repeat(p_offset,WDTO_120MS,WDTO_120MS)

#define Beep_Reset() Beep_Repeat(5,WDTO_120MS,WDTO_60MS)


uint8_t Read_Button(void) {
  uint8_t l_count = 0 ;

  Set_Delay(WDTO_60MS) ;
  while (!Button_Released()) {
    Delay_Sleep() ; // First call is also for debouncing
    if (++l_count >= BUTTON_LONG_DELAY) return BUTTON_LONG ;
  }
  return (l_count) ? BUTTON_SHORT : BUTTON_NONE ;
}


void Reset_Avr(void) {
  Set_Delay(WDTO_15MS) ;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN) ;
  WDTCR |= _BV(WDE) ;
  sleep_mode() ;
}


void Manage_Sensors(void) {
  static uint8_t l_elapsed = 1 ;
  static uint16_t l_curtemp ;
  static uint8_t l_hotcount = 0 ;
  static uint16_t l_avertemp = 0xFF00 ; // Not 0xFFFF to keep some place for first comparison

  // Check temperature and battery every 5mn
  if (l_elapsed % SENSOR_DELAY == 0) {
    l_curtemp = Read_ADC_Mux(TEMPERATURE_MASK) ;
    if (l_hotcount < TOO_HOT_COUNT)
      l_avertemp = Average_Temperature(l_curtemp) ;
    if (l_curtemp >= l_avertemp + Get_Hot_Offset(FALSE))
      l_hotcount++ ;
    else
      l_hotcount = 0 ;
    if (Read_ADC_Mux(BATTERY_MASK) >= LOW_BATTERY) { // Battery level too low
      Set_Delay(WDTO_250MS) ; Beep_Sleep() ;
    }
  }

  if ((l_hotcount >= TOO_HOT_COUNT) && (l_elapsed % TOO_HOT_DELAY == 0)) { // Too hot for too long -> beep
    Beep_Too_Hot() ;
    l_hotcount = TOO_HOT_COUNT ;
  }

  l_elapsed++ ;
}


int main(void) {

  // Disable watchdog
  MCUSR &= ~_BV(WDRF) ;
  Modify_Watchdog() ; WDTCR &= ~_BV(WDE) ;
  // IO port init
  DDRB = _BV(BUZZER_PIN) ; // Output pin
  PORTB = ~(_BV(BUZZER_PIN)) ; // All other pins as input with pull-up
  // ADC init
  ADCSRA = _BV(ADPS1) | _BV(ADPS0) ; // Prescaler 8
  // Button pin mask
  PCMSK = _BV(BUTTON_PIN) ;

  power_all_disable() ;

  Set_Delay(WDTO_500MS) ; Delay_Sleep() ;

  // Warn device ready
  Beep_Offset(Get_Hot_Offset(FALSE)) ;

  for(;;) {

    Enable_Button() ;
    Set_Delay(WDTO_8S) ; Delay_Sleep() ;
    Disable_Button() ;

    switch (Read_Button()) {

      case BUTTON_SHORT:
        Beep_Offset(Get_Hot_Offset(TRUE)) ;
        Set_Delay(WDTO_250MS) ; Delay_Sleep() ;
        break ;

      case BUTTON_LONG:
        Beep_Reset() ;
        Set_Delay(WDTO_2S) ; Delay_Sleep() ;
        Reset_Avr() ;
        break ;

      default:
        Manage_Sensors() ;
        break ;
    }

  }
}
