#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "./hd44780.h"
//#include <i2cmaster.h>


enum {
    tr  = 10,    // reset pulse width (microseconds)
    trs = 80,    // reset-to-strobe (microseconds)
    to  = 40,    // output settling time (microseconds)
    tss = 1800,  // strobe-to-strobe (microseconds; much, much higher than the mininum)
};

void setup(){
  lcd_init();
  lcd_clrscr();
  //i2c_init();

  // Analog Input
  ADMUX = 0x05;
  ADMUX |= (1<<REFS0);
  ADMUX |= (1<<ADLAR);

  ADCSRA |= (1<<ADEN);

  //Strobe
  DDRD |= (1<<PD4);

  //Reset
  DDRD |= (1<<PD3);
}


void msgeq7_reset(void) {
  DDRD |=(1 << PD4); //make sure that STORBE pin is high before RESET
  _delay_us(100);

  DDRD |=(1 << PD3);
  _delay_us(100);

  DDRD &= ~(1 << PD3);
  _delay_us(100);

}

void msgeq7_strobe(void) {
  // Set strobe and wait for pulse to complete.
  PORTD |= (1<<PD4);
  _delay_us(tr);

  //Clear strobe pin and wait.
  PORTD &= ~(1<<PD4);
  _delay_us(trs);
}

int msgeq7_read(void) {
  int ADC_value;

  while(ADCSRA & (1 << ADSC)); // Wait for ADC to read.
  ADCSRA |= (1<<ADSC);
  ADC_value = ADCL;
  return ADC_value;
}

int main(void) {
  setup();
  unsigned int ADC_value;
  unsigned char ADC_string[7];

  unsigned int i;
  unsigned char count[4];

  msgeq7_reset();
  for (i=0;i<1000;i++) {
    /*itoa(i, count, 10);
    lcd_goto(0);
    lcd_puts(count);
    _delay_ms(50);
    lcd_goto(40);
    msgeq7_strobe();
    ADC_value = msgeq7_read();
    itoa(ADC_value, ADC_string, 10);
    lcd_puts("ADC_value = ");
    lcd_puts(ADC_string);*/

    lcd_command(_BV(LCD_CGRAM)+0*8); //The 0 on this line may be 0-7
    lcd_putc(0b00000); //5x8 bitmap of character, in this example a backslash
    lcd_putc(0b00000);
    lcd_putc(0b00000);
    lcd_putc(0b00000);
    lcd_putc(0b00000);
    lcd_putc(0b00000);
    lcd_putc(0b11111);
    lcd_putc(0b11111);
    lcd_goto(0);
    lcd_putc(0);
  }
  return 0;
}
