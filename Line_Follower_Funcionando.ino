#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// function prototypes
void init();                                       // initialization
void play();                                       // move
void readSensors();                                // read sensor values and store them
void setM1Speed(float speed);                      // set speed of right motor
void setM2Speed(float speed);                      // set speed of left motor
void InitTimerCounter1();                          // motors
void InitADC();                                    // single read mode
uint16_t ReadADCSingleConversion(uint8_t channel); // reading sensors

// global variables
float set_speed_right, set_speed_left;
uint16_t right_sensor, left_sensor;
uint16_t pot_value = 0; // potentiometer
float offset = 0;       // difference between sensors
float T = 500;          // value where we consider black line (adjust according to hardware)
float kKp = 1.0/300.0;      // 1/max difference between sensors (Iadjust according to hardware)
float m_deviation = 0.87;

int main(){
  init();
  while(1){
    readSensors();
    play();
   
  }
}

void init(){
  sei();
  InitTimerCounter1();
  InitADC();
  DDRB |= ((1<<PB0)|(1<<PB1)|(1<<PB2)|(1<<PB3)); //set motors as outputs
}

void play(){
  // error
  int16_t error = right_sensor - left_sensor;
  offset = kKp * static_cast<float>(error);      // choose kKp so [-1.0 <= offset <= 1.0]
  set_speed_right =60;
  set_speed_left = 60*m_deviation;
  // acceleration
  if(right_sensor >= T || left_sensor >= T){      // on black line, going forward using P-control of the two motors
    setM1Speed((1.0 + offset)*set_speed_right);    // offset<-kKp*T: accelerate right motor
    setM2Speed((1.0 - offset)*set_speed_left);     // offset<-kKp*T: slow down left motor
  }else if(right_sensor < T && left_sensor < T){  // lost black line, going backward with fixed speed
    setM1Speed(-set_speed_right);
    setM2Speed(-set_speed_left);
  }else{                                          // unexpected situation
    setM1Speed(0);
    setM2Speed(0);
  }
}

void readSensors(){
  right_sensor = ReadADCSingleConversion(1<<PC0); // values between 0 and 1023
  left_sensor = ReadADCSingleConversion(1<<PC1);

}

void setM1Speed(float speed){
  if(speed>0){PORTB |= (1<<PB0);}
  if(speed<0){PORTB &= (0<<PB0); speed *= -1;}
  OCR1A = static_cast<uint16_t>(speed*(ICR1/100.0));
  
}

void setM2Speed(float speed){
  if(speed>0){PORTB |= (1<<PB3);}
  if(speed<0){PORTB &= (0<<PB3); speed *= -1;}
  OCR1B = static_cast<uint16_t>(speed*(ICR1/100.0));
  
}

void InitTimerCounter1(){
  TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<WGM11); // Set A and B on TOP, clear at compare match.
  TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<WGM13);    // Fast PWM mode, 8 prescaler.
  ICR1 = 15999; 
}

void InitADC(){
  ADMUX = (1<<REFS0);                           // internal Vcc (+5 V) as REF voltage
  ADCSRA = (1<<ADPS2) | (1<<ADPS1)| (1<<ADPS0); // prescale divison factor 128, ADC clock: 125 kHz, sampling rate: 9.6 KHz
  ADCSRA |= (1<<ADEN);                          // endable ADC
  ADCSRA |= (1<<ADSC);                          // start single conversion, do first ADC run (initialisation) to warm up
  do{}while(ADCSRA&(1<<ADSC));                  // wait until first conversion is finished, ADSC bit resets to 0 on ADC complete
  uint16_t garbage = ADC;                       // read ADC register to clean ADC
}

uint16_t ReadADCSingleConversion(uint8_t channel){
  const uint8_t kMuxMask = ((1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<MUX3));
  ADMUX &= ~kMuxMask;             // clear MUX
  ADMUX |= channel;               // set input channel
  ADCSRA |= (1<<ADSC);            // start single conversion
  do {} while(ADCSRA&(1<<ADSC));  // wait until it is finished
  return ADC;                     // return result
}
