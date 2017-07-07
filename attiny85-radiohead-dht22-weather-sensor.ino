/*
 * RadioHead DHT22 Weather Station
 * 
 * Copyright (c) 2017 Peter Müller <peter@crycode.de> (https://crycode.de)
 */

/**************
 *   config   *
 **************/
 
// pin with the LED connected
#define LED_PIN 4

// blink time for the LED
#define LED_TIME 200

// pin of the DHT22 sensor
#define DHT_PIN 1

// the own RadioHead address
#define RH_OWN_ADDR 0xca // 202

// the server RadioHead address
#define RH_SERVER_ADDR 0x01

// RadioHead bitrate in bit/s
#define RH_SPEED 2000

// pins for the radio hardware
#define RH_RX_PIN 5 // not used, set to a non-existens pin
#define RH_TX_PIN 3
#define RH_PTT_PIN 5 // not used, set to a non-existens pin

// time until the watchdog wakes the mc in seconds
#define WATCHDOG_TIME 8 // 1, 2, 4 or 8

// after how many watchdog wakeups we should collect and send the data
#define WATCHDOG_WAKEUPS_TARGET 7 // 8 * 7 = 56 seconds between each data collection

// after how many data collections we should get the battery status
#define BAT_CHECK_INTERVAL 30

// min max values for the ADC to calculate the battery percent
#define BAT_ADC_MIN 40  // ~0,78V
#define BAT_ADC_MAX 225 // ~4,41V

/**************
 * end config *
 **************/

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#if BAT_ADC_MIN >= BAT_ADC_MAX
  #error BAT_ADC_MAX must be greater than BAT_ADC_MIN!
#endif

// DHT22 lib
// not using the current adafruit dht library, because it eats too many memory
#include "dht22.h"
dht22 dht;

// buffer for RadioHead messages
// rh_buf[0] - control byte
// control byte 0x00
//   start message
// control byte 0x01
//   rh_buf[1-4] - temperature as float
//   rh_buf[5-8] - humidity as float
// control byte 0x02
//   rh_buf[1] - battery status in %
//   rh_buf[2] - battery raw ADC value (0 to 255)
// control byte 0xEE
//   error reading temperature/humidity
#define RH_BUF_LEN 9
uint8_t rh_buf[RH_BUF_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// reduce the RadioHead max message length to save memory
#define RH_ASK_MAX_MESSAGE_LEN RH_BUF_LEN

// RadioHead
#include <RH_ASK.h>
#include <RHDatagram.h>
#include <RHReliableDatagram.h> // only needed for some constants

RH_ASK rh_driver(RH_SPEED, RH_RX_PIN, RH_TX_PIN, RH_PTT_PIN);
RHDatagram rh_manager(rh_driver, RH_OWN_ADDR);

// some mcs (e.g atmega328) have WDTCSR instead of WTDCR
#if !defined WDTCR and defined WDTCSR
  #define WDTCR WDTCSR
#endif

// header ID for the RadioHead message
uint8_t rh_id = 0;

// counter for the battery check, starting at BAT_CHECK_INTERVAL to transmit the battery status on first loop
uint8_t bat_check_count = BAT_CHECK_INTERVAL;

void enableWatchdog()
{
  cli();
  
  // clear the reset flag
  MCUSR &= ~(1<<WDRF);
  
  // set WDCE to be able to change/set WDE
  WDTCR |= (1<<WDCE) | (1<<WDE);

  // set new watchdog timeout prescaler value
  #if WATCHDOG_TIME == 1
    WDTCR = 1<<WDP1 | 1<<WDP2;
  #elif WATCHDOG_TIME == 2
    WDTCR = 1<<WDP0 | 1<<WDP1 | 1<<WDP2;
  #elif WATCHDOG_TIME == 4
    WDTCR = 1<<WDP3;
  #elif WATCHDOG_TIME == 8
    WDTCR = 1<<WDP0 | 1<<WDP3;
  #else
    #error WATCHDOG_TIME must be 1, 2, 4 or 8!
  #endif
  
  // enable the WD interrupt to get an interrupt instead of a reset
  WDTCR |= (1<<WDIE);
  
  sei();
}

// function to go to sleep
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   /* EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
  sleep_enable();
  
  /* Now enter sleep mode. */
  sleep_mode();
  
  /* The program will continue from here after the WDT timeout*/
  sleep_disable(); /* First thing to do is disable sleep. */
}

void setup() {
  // setup the LED pin
  pinMode(LED_PIN, OUTPUT);

  // setup the ADC
  ADMUX =
    (1 << ADLAR) | // left shift result
    (0 << REFS1) | // Sets ref. voltage to VCC, bit 1
    (0 << REFS0) | // Sets ref. voltage to VCC, bit 0
    (0 << MUX3)  | // use ADC1 for input (PB2), MUX bit 3
    (0 << MUX2)  | // use ADC1 for input (PB2), MUX bit 2
    (0 << MUX1)  | // use ADC1 for input (PB2), MUX bit 1
    (1 << MUX0);   // use ADC1 for input (PB2), MUX bit 0
  ADCSRA =
    (1 << ADEN)  | // enable ADC
    (1 << ADPS2) | // set prescaler to 64, bit 2
    (1 << ADPS1) | // set prescaler to 64, bit 1
    (0 << ADPS0);  // set prescaler to 64, bit 0

  // disable ADC for powersaving
  ADCSRA &= ~(1<<ADEN);

  // disable analog comperator for powersaving
  ACSR |= (1<<ACD);

  // init RadioHead
  if(!rh_manager.init()){
    // init failed, blink 10 times, then go to sleep
    for(uint8_t i=0; i<10; i++){
      digitalWrite(LED_PIN, HIGH);
      _delay_ms(LED_TIME);
      digitalWrite(LED_PIN, LOW);
      _delay_ms(LED_TIME);
    }
    enterSleep();
  }

  // init the DHT22
  dht_init(&dht, DHT_PIN);

  // send start message
  rh_buf[0] = 0x00;
  rh_send(1);

  // blink 3 times
  for(uint8_t i=0; i<3; i++){
    digitalWrite(LED_PIN, HIGH);
    _delay_ms(LED_TIME);
    digitalWrite(LED_PIN, LOW);
    _delay_ms(LED_TIME);
  }

  // enable the watchdog
  enableWatchdog();
}

// main loop
void loop() {
  // turn on the LED
  digitalWrite(LED_PIN, HIGH);

  // battery check/status
  bat_check_count++;
  if(bat_check_count >= BAT_CHECK_INTERVAL){
    bat_check();
    bat_check_count = 0;
  }

  // temperature and humidity
  float t = 0;
  float h = 0;

  // read from the sensor and check if it was successfull
  if(dht_read_data(&dht, &t, &h) == 1){
    // normal message
    rh_buf[0] = 0x01;
  
    // copy to buffer
    memcpy(&rh_buf[1],&t,4);
    memcpy(&rh_buf[5],&h,4);
  
    rh_send(RH_BUF_LEN);
    
  }else{
    // error message
    rh_buf[0] = 0xee;
    rh_send(1);
    
  }
  
  // turn off the LED
  digitalWrite(LED_PIN, LOW);
  
  // deep sleep
  for(uint8_t i=0;i < WATCHDOG_WAKEUPS_TARGET;i++){
    enterSleep();
  }
}

// function to read and send the battery status
void bat_check(){
  // enable the ADC
  ADCSRA |= (1<<ADEN);

  // short delay
  _delay_ms(10);

  ADCSRA |= (1 << ADSC); // start ADC measurement
  while ( ADCSRA & (1 << ADSC) ); // wait till conversion complete
  
  int16_t adc = ADCH;

  // clear the ADIF bit by writing 1 to it
  ADCSRA|=(1<<ADIF);

  // disable the ADC
  ADCSRA &= ~(1<<ADEN);

  // write the raw value into the buffer
  rh_buf[2] = adc;
  
  // calc the battery status in percent, respecting min and max
  adc = 100 * ( adc - BAT_ADC_MIN ) / (BAT_ADC_MAX - BAT_ADC_MIN);
  if(adc > 100){
    adc = 100;
  }else if(adc < 0){
    adc = 0;
  }

  // write it into the buffer
  rh_buf[1] = adc;

  // set command byte to 0x02 for battery status
  rh_buf[0] = 0x02;

  // send 3 bytes from the buffer
  rh_send(3);
}

// function to send RadioHead messages from the buffer
void rh_send(uint8_t len){
  // set header ID
  rh_id++;
  rh_manager.setHeaderId(rh_id);

  // set/reset flags
  rh_manager.setHeaderFlags(RH_FLAGS_NONE, RH_FLAGS_ACK);

  // send the data
  rh_manager.sendto(rh_buf, len, RH_SERVER_ADDR);

  // wait until sending data is done
  rh_manager.waitPacketSent();
}

// watchdog ISR
ISR(WDT_vect){
  // nothing to do here, just wake up
}
