#include <avr/io.h>
#define INPUT 0x0
#define OUTPUT 0x1
#define LOW 0
#define HIGH 1
#define _BV(bit) (1 << (bit))
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define PRTA 0
#define PRTB 1
#define PRTC 2
#define PRTD 3
#define PORTB_ADDRESS 0x05
#define PORTC_ADDRESS 0x08
#define PORTD_ADDRESS 0x0B
#define PINB_ADDRESS 0x03
#define PINC_ADDRESS 0x06
#define PIND_ADDRESS 0x09
#define DDRB_ADDRESS 0x04
#define DDRC_ADDRESS 0x07
#define DDRD_ADDRESS 0x0A
void types(String a) { Serial.println("it's a String"); }
void types(int a) { Serial.println("it's an int"); }
void types(char *a) { Serial.println("it's a char*"); }
void types(float a) { Serial.println("it's a float"); }
void types(bool a) { Serial.println("it's a bool"); }

uint8_t avr_digitalPinToBitMask(uint8_t pin)
{
    if (pin >= 0 && pin <= 7)
    {
        return _BV(pin);
    }
    else if (pin >= 8 && pin <= 13)
    {
        return _BV(pin - 8);
    }
    else if (pin >= A0 && pin <= A5)
    {
        return _BV(pin - 14);
    }
}
uint8_t avr_digitalPinToPort(uint8_t pin)
{
    if (pin >= 0 && pin <= 7)
    {
        return PRTD;
    }
    else if (pin >= 8 && pin <= 13)
    {
        return PRTB;
    }
    else if (pin >= A0 && pin <= A5)
    {
        return PRTC;
    }
}
uint8_t *avr_portModeRegister(uint8_t port)
{
    if (port == PRTB)
    {
        return (volatile uint8_t *)(DDRB_ADDRESS + 0x20);
    }
    else if (port == PRTC)
    {
        return (volatile uint8_t *)(DDRC_ADDRESS + 0x20);
    }
    else if (port == PRTD)
    {
        return (volatile uint8_t *)(DDRD_ADDRESS + 0x20);
    }
}

uint8_t *avr_portInputRegister(uint8_t port)
{
    if (port == PRTB)
    {
        return (volatile uint8_t *)(PINB_ADDRESS + 0x20);
    }
    else if (port == PRTC)
    {
        return (volatile uint8_t *)(PINC_ADDRESS + 0x20);
    }
    else if (port == PRTD)
    {
        return (volatile uint8_t *)(PIND_ADDRESS + 0x20);
    }
}
void avr_pinMode(uint8_t pin, uint8_t mode)
{
    uint8_t bit = avr_digitalPinToBitMask(pin);
    uint8_t port = avr_digitalPinToPort(pin);
    volatile uint8_t *reg, *out;

    // JWS: can I let the optimizer do this?
    reg = avr_portModeRegister(port);

    if (mode == INPUT)
    {
        *reg &= ~bit;
    }
    else
    {
        *reg |= bit;
    }
}
uint8_t *avr_portOutputRegister(uint8_t port)
{
    if (port == PRTB)
    {
        return (volatile uint8_t *)(PORTB_ADDRESS + 0x20);
    }
    else if (port == PRTC)
    {
        return (volatile uint8_t *)(PORTC_ADDRESS + 0x20);
    }
    else if (port == PRTD)
    {
        return (volatile uint8_t *)(PORTD_ADDRESS + 0x20);
    }
}
void avr_digitalWrite(uint8_t pin, uint8_t val)
{
    uint8_t bit = avr_digitalPinToBitMask(pin);
    uint8_t port = avr_digitalPinToPort(pin);
    volatile uint8_t *out;
    out = avr_portOutputRegister(port);
    if (val == LOW)
    {
        *out &= ~bit;
    }
    else
    {
        *out |= bit;
    }
}

int avr_digitalRead(uint8_t pin)
{
    uint8_t bit = avr_digitalPinToBitMask(pin);
    uint8_t port = avr_digitalPinToPort(pin);
    Serial.println(PIND);
    Serial.println(*avr_portInputRegister(port));
    if (*avr_portInputRegister(port) & bit)
        return HIGH;
    return LOW;
}

int avr_analogRead(uint8_t ch)
{
    uint8_t pin = ch - 14;
    ADMUX = 1 << REFS0;
    ADCSRA |= (1 << ADEN);
    pin &= 0b111;
    ADMUX = (ADMUX & 0xF8) | pin;
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC))
        ;
    return ADC;
}

uint8_t avr_analogWrite(uint8_t apin, uint8_t cycle)
{

    if (apin == 3)
    {
        DDRD |= _BV(DDD3);
        TCCR2A |= (_BV(COM2B1) | _BV(WGM21) | _BV(WGM20));
        TCCR2B |= (_BV(CS22) | _BV(CS20));
        OCR2B = cycle;
        return (0);
    }
    else if (apin == 5)
    {
        DDRD |= _BV(DDD5);
        TCCR0A |= (_BV(COM0B1) | _BV(WGM21) | _BV(WGM20));
        TCCR0B |= (_BV(CS21) | _BV(CS20));
        OCR0B = cycle;
        return (0);
    }
    else if (apin == 6)
    {
        DDRD |= _BV(DDD6);
        TCCR0A |= (_BV(COM0A1) | _BV(COM0A0) | _BV(WGM21) | _BV(WGM20));
        TCCR0B |= (_BV(CS21) | _BV(CS20));
        OCR0A = cycle;
        return (0);
    }
    else if (apin == 9)
    {
        DDRB |= _BV(DDB1);
        TCCR1A |= (_BV(COM1A1) | _BV(WGM10));
        TCCR1B |= (_BV(WGM12) | _BV(CS11) | _BV(CS10));
        OCR1A = cycle;
        return (0);
    }
    else if (apin == 10)
    {
        DDRB |= _BV(DDB2);
        TCCR1A |= (_BV(COM1B1) | _BV(WGM10));
        TCCR1B |= (_BV(WGM12) | _BV(CS11) | _BV(CS10));
        OCR1B = cycle;
        return (0);
    }
    else if (apin == 11)
    {
        DDRB |= _BV(DDB3);
        TCCR2A |= (_BV(COM2A1) | _BV(WGM21) | _BV(WGM20));
        TCCR2B |= (_BV(CS22) | _BV(CS20));
        OCR2A = cycle;
        return (0);
    }
    else
    {
        return (1);
    }
}
