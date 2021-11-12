#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <avr/power.h>     // CPU power save


#define  LEFT_FOOT   0
#define RIGHT_FOOT   1

///// Initial BLE ///
//#define Initial
 
#define FOOT_SEL     LEFT_FOOT   
//#define FOOT_SEL     RIGHT_FOOT 

#define USE_2X
#define F_CPU 8000000

#if defined(Initial)
  #define BAUD  38400
#else
  #define BAUD  38400
#endif

#include <util/setbaud.h>
#include <util/delay.h>

#include <stdlib.h>
//////////// BATTERY MEASUREMENT ////////
#define  Vmax     4.0
#define  Vref     3.3
#define  R1       330000.0 // 
#define  R2       470000.0

#define  ADC_VMAX_BATT     (((R2/(R1+R2))*Vmax)*1023.0)/Vref
#define  ADC_VMIN_BATT     (((R2/(R1+R2))*Vref)*1023.0)/Vref

#define Calculate_Percent_BaTT(X)   ((X-ADC_VMIN_BATT)/(ADC_VMAX_BATT-ADC_VMIN_BATT))*100.0

//////////// TIMER //////////////// 
volatile uint16_t timer1_second = 0; // counter 
volatile uint16_t timer1_counter;

/////////// BLE AT COMMAND ////////////// 
char * name_R = "AT+NAMESurasole Lite R";
char * name_L = "AT+NAMESurasole Lite L";
char * rst  = "AT+RST";

//char * tx_powr = "AT+POWR8";
char * AT_BAUD  = "AT+BAUD2"; // 38400
char * AT_ADVIN = "AT+ADVIN0";
char * AT_SLEEP = "AT+SLEEP2"; // Deep sleep
//////////// SLEEP ////////////
volatile uint16_t Wait_sleep = 25;// Second
bool on_connected = false;
volatile bool _en_send = false;
volatile bool _equal    = false;

/////////// Function Prototype //////
uint16_t calcrc_multiple(uint8_t * ptr,int index);
void enable_sleep();
void transmit_string(char ch[]);
void stop_timer();
void initial_chip();
void goToSleep(void);
int ADCsingleREAD(uint8_t adctouse);

volatile int data_count = 0;
volatile bool rst_ble = false;
char data[40];
char str_1[20];
char str_2[20];
char comma = ',';
char _or   = '|';

uint16_t threshold = 80;
//uint8_t * data_p;

uint16_t adc_raw[8];
uint16_t per_batt;
//uint16_t crc_chk=0;
//uint16_t adc_batt=0;

/////////// EMA FILTER ////////////////////
float ema_a = 0.25;

uint16_t ema_ema_high = 0;
uint16_t ema_high = 0;

uint16_t EMA_function_high(float alpha,uint16_t latest,uint16_t stored)
{
  return round(alpha*latest) + round((1-alpha)*stored);
}
///////// Varible to Stop Loop ////////////
volatile int clr_ere = 0;
#define loop_count 100 //Same 100ms

/////////////// Interrupt Timer 0 >> 1kHz( 1 ms) /////////
ISR(TIMER0_COMPA_vect)
{
   clr_ere++; // Increase clr_err to stop loop
   if(clr_ere==20)
   {
	     clr_ere=0;
		 _en_send = true;
	}
}
void init_timer_0()
{

  TCCR0A = 0;  
  TCCR0B = 0; 
  TCNT0  = 0;                                
  OCR0A = 124;// Formula >> (8MHz)/(1k*8) - 1  //// set compare match register for 1khz increments
  
  TCCR0A |= (1 << WGM01); // turn on CTC mode

  //TCCR0B |= (1 << CS01);      // Set prescaler equal
  TCCR0B |= (1 << CS01) | (1 << CS00);      // Set prescaler 1024
  TIMSK0 &= (0 << OCIE0A);   // Disable timer compare interrupt

}
/////////////// Func Enable Timer 0 CTC Mode //////////////
void ena_timer_0()
{
  TIMSK0 |= (1 << OCIE0A);   // Enable timer compare interrupt
}
void disa_timer_0()
{
  TIMSK0 &= (0 << OCIE0A);   // Disable timer compare interrupt
}
//////////////// UART Function //////////////////
void init_usart()
{
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;

	#if USE_2X
	UCSR0A |= (1 << U2X0);
	#else
	UCSR0A &= ~(1 << U2X0);
	#endif

 UCSR0C &= ~(_BV(UMSEL01) | _BV(UMSEL00)); // enable asynchronous USART

 // Set frame format to 8 data bits, Even parity, 1 stop bit
//UCSR0C |= (1<<UPM01)|(1<<UCSZ01)|(1<<UCSZ00);

 // Set frame format to 8 data bits, None parity, 1 stop bit
 UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
 //enable reception and RC complete interrupt
 UCSR0B |= (1 << TXEN0)|(1<<RXEN0)|(1<<RXCIE0);  
 
 //sei();  

}

ISR(USART_RX_vect) // INTERRUPT SERVICE ROUTINE
{
  
}
////////// Change NAME NLE ///////////
void ble_cmd(char cmd[])
{
  PORTB = 0b00001000; // PB3 >> Mosfet BLE // PB2 >> PWRC
  _delay_ms(200);
  int tr_n;
  for(tr_n=0;tr_n<strlen(cmd);)
  { 
 		while ( !( UCSR0A & (1<<UDRE0))); 
  		UDR0 = cmd[tr_n];
		tr_n++;
  }
   		while ( !( UCSR0A & (1<<UDRE0))); 
  		UDR0 = 0x0d;
	    while ( !( UCSR0A & (1<<UDRE0))); 
  		UDR0 = 0x0a;
  _delay_ms(1000);
  PORTB = 0b00001100;
}
void transmit_string(char ch[])
{
  int tr;
  int _num = strlen(ch);
  for(tr=0;tr<_num;) //strlen(ch)
  { 
 		while ( !( UCSR0A & (1<<UDRE0))); 
  		UDR0 = ch[tr];
		tr++;
  }
}
////////////// TIMER INTERRUPT ////////////////
void init_timer()
{
  TCCR1A = 0; //
  TCCR1B = 0; //
  timer1_counter = 17143;   // preload timer 65536-16MHz/256/1Hz >> 34286
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler  

}

ISR(TIMER1_OVF_vect) // 
{
  TCNT1 = timer1_counter;   // preload timer
  timer1_second++;   
  if(timer1_second == Wait_sleep) 
  { 
       stop_timer();
       timer1_second = 0;
       goToSleep();
  }

}

void start_timer() //
{
     TIMSK1 |= (1 << TOIE1);   
}

void stop_timer() // 
{
     TIMSK1 &= (0 << TOIE1);   
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
   asm volatile ("  jmp 0");  
}  
void goToSleep(void)
{
    timer1_second=0;
    uint8_t adcsra = ADCSRA;          //save the ADC Control and Status Register A
	//cli();                     //stop interrupts to ensure the BOD timed sequence executes as required
    
	ADCSRA = 0;                    //disable the ADC

    //EICRA = _BV(ISC01);            //configure INT0 to trigger on falling edge
    EICRA |= _BV(ISC00);             //configure INT0 to trigger on any logical change
    EIMSK |= _BV(INT0);             //enable INT0

	PORTB = 0b00000100; // SLEEP PWRC(PB2) >> HIGH

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    cli();                     //stop interrupts to ensure the BOD timed sequence executes as required
    sleep_enable();

    //disable brown-out detection while sleeping (20-25�A)
    uint8_t mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE);
    uint8_t mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1;
    MCUCR = mcucr2;
    sleep_bod_disable();           //for AVR-GCC 4.3.3 and later, this is equivalent to the previous 4 lines of code

    PORTB = (0 << PB3)|(1 << PB2);
    sei();                         //ensure interrupts enabled so we can wake up again
    sleep_cpu();                   //go to sleep
    sleep_disable();               //wake up here
    ADCSRA = adcsra;               //restore ADCSRA
}

ISR(INT0_vect)
{

	        cli();
		    EIMSK &= (0 << INT0); //DISABLE INT0                
			timer1_second=0;
	        PORTB = 0b00001100;
			init_usart();
		    init_timer();
		    start_timer();

		    init_timer_0();
		    ena_timer_0();
    
			sei();

}
//read internal voltage
float readVcc() 
{
  long result;
  //double curvolt;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  _delay_ms(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV

  //double curvolt = double(result)/1000;

  return (double)result/1000.0;
}
/////////////// READ ADC //////////////////////// 
int ADCsingleREAD(uint8_t adctouse)
{
    int ADCval;
    ADMUX = adctouse;         // use #1 ADC
    ADMUX |= (1 << REFS0);    // use AVcc as the reference
    ADMUX &= ~(1 << ADLAR);   // clear for 10 bit resolution   
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);    // 128 prescaler for 16Mhz
    ADCSRA |= (1 << ADEN);    // Enable the ADC
    ADCSRA |= (1 << ADSC);    // Start the ADC conversion
    while(ADCSRA & (1 << ADSC));      // Thanks T, this line waits for the ADC to finish 
    ADCval = ADCL;
    ADCval = (ADCH << 8) + ADCval;    // ADCH is read so ADC can be updated again
    return ADCval;
}

////////////// Watchdog ///////////////////
void WDT_Init(void) //initialize watchdog
{
//disable interrupts
//cli();
//reset watchdog
wdt_reset();
//set up WDT interrupt
WDTCSR = (1<<WDCE)|(1<<WDE);
//Start watchdog timer with 4s prescaller
WDTCSR = (1<<WDIE)|(1<<WDE)|(1<<WDP3);
//Enable global interrupts
//sei();
}

void WDT_off(void)
{
//disable interrupts
//cli();
wdt_reset();
/* Clear WDRF in MCUSR */
MCUSR &= ~(1<<WDRF);
/* Write logical one to WDCE and WDE */
/* Keep old prescaler setting to prevent unintentional time-out */
WDTCSR |= (1<<WDCE) | (1<<WDE);
/* Turn off WDT */
WDTCSR = 0x00;
 
//Enable global interrupts
//sei();
}

void initial_chip()
{
     //////// BLE STATUS /////////////
   // DDRD &= ~(1 << PD3); // PD2 as INPUT

    DDRB |= 0b00001100; // PB2 >> PWRC // PB3 >> ON BLE
    //PORTB = 0b00001000;
	_delay_ms(20);
    PORTB = 0b00001100;
     init_usart();
   _delay_ms(10);

   char * name;
   ///////// SEL NAME //////
   switch(FOOT_SEL)
   {
      case 0: name = name_L; break; // LEFT
      case 1: name = name_R; break; // RIGHT
   }

     #if defined (Initial) 
      ble_cmd(AT_ADVIN);
      ble_cmd(AT_BAUD);
      ble_cmd(name);
	  ble_cmd(rst);
     #endif
    //cli();
    init_timer();
    start_timer();

    init_timer_0();
    ena_timer_0(); 

     EIMSK &= (0 << INT0); //DISABLE INT0
      
    //WDT_Init();  //initialize watchdog
    //wdt_enable(WDTO_2S);
     sei();
}

int main()
{
 /////////////// CONTROL BLE ///////////  
     _delay_ms(100);

     DDRB  |= 0b00001100; // PB2 >> PWRC // PB3 >> ON BLE
     PORTB = 0b00001100; // TURNOFF BLE
	 CLKPR = 0x80;
  per_batt = Calculate_Percent_BaTT(ADCsingleREAD(3)); // Percent Batterry
   if(per_batt>100)per_batt = 100;

  if(per_batt<1)
   {
	  goToSleep();
   }
  else
   {
       //_delay_ms(1000); // Wait 3 sec
	   initial_chip();   
   }

 while(1)
  {
  //  int var = PIND & (1 << PIND3);
      // wdt_reset();
	  
	        
	  ///////////// R /////////////////
	  if(_en_send)
	  {
     	  disa_timer_0();
		  if(FOOT_SEL==RIGHT_FOOT)
		  {
		  		      adc_raw[0] = ADCsingleREAD(0); // FSR 0
		   	  adc_raw[1] = ADCsingleREAD(1); // FSR 1
		  	  adc_raw[2] = ADCsingleREAD(2); // FSR 2
		 	  adc_raw[3] = ADCsingleREAD(5); // FSR 3
		  	  adc_raw[4] = ADCsingleREAD(4); // FSR 4
			  		     ////////////////////////////// CLEAR TIMER ///////////////////////////
				     if(adc_raw[0]>= threshold || adc_raw[1]>= threshold || adc_raw[2]>= threshold|| adc_raw[3]>= threshold|| adc_raw[4]>= threshold)
					  {
					     timer1_second=0;
					  }	 
		      	  per_batt = Calculate_Percent_BaTT(ADCsingleREAD(3)); // Percent Batterry
		      if(per_batt>100)per_batt = 100;
			  sprintf(data,"%d%c%d%c%d%c%d%c%d%c%d%c",adc_raw[1],comma,adc_raw[0],comma,adc_raw[2],comma,adc_raw[4],comma,adc_raw[3],comma,per_batt,_or);  // LEFT
			  		  if(per_batt<1)
					  {
						  goToSleep();
					  }
		  
		   }
		   else
		   {
		      adc_raw[0] = ADCsingleREAD(0); // FSR 0
		   	  adc_raw[1] = ADCsingleREAD(1); // FSR 1
		  	  adc_raw[2] = ADCsingleREAD(2); // FSR 2
		 	  adc_raw[3] = ADCsingleREAD(5); // FSR 3
		  	  adc_raw[4] = ADCsingleREAD(4); // FSR 4
			  		     ////////////////////////////// CLEAR TIMER ///////////////////////////
				     if(adc_raw[0]>= threshold || adc_raw[1]>= threshold || adc_raw[2]>= threshold|| adc_raw[3]>= threshold|| adc_raw[4]>= threshold)
					  {
					     timer1_second=0;
					  }	 
			    per_batt = Calculate_Percent_BaTT(ADCsingleREAD(3)); // Percent Batterry
		      if(per_batt>100)per_batt = 100;
			  sprintf(data,"%d%c%d%c%d%c%d%c%d%c%d%c",adc_raw[4],comma,adc_raw[2],comma,adc_raw[3],comma,adc_raw[1],comma,adc_raw[0],comma,per_batt,_or);
			  	 if(per_batt<1)
					  {
						  goToSleep();
					  }	
		   }
		///////////////// CHECK BUFFER SIZE //////////////////
		int length_str = (int)strlen(data);
		if(length_str > 19) //more Than 20 Byte
		   {

		        memset(str_1,'\0',sizeof(str_1));
		        memcpy(str_1,data,19);
		        memset(str_2,'\0',sizeof(str_2));
		        memcpy(str_2,data+19,length_str-19);
                transmit_string(str_1);
				transmit_string(str_2); 
				_en_send = false;
		   }
		     else 
		   {
		        memset(str_1,'\0',sizeof(str_1));
		        memcpy(str_1,data,strlen(data));
				transmit_string(str_1);
				_en_send = false;	  
		   }

	     init_timer_0();
         ena_timer_0();
      }
	
  }
  return 0;
}
