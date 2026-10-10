#define F_CPU 8000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#define RELAY_PIN     PB7
#define VREF          3.3     
#define DIVIDER       161.0   
#define PEAK_FACTOR   1.414   
#define SENSITIVITY   0.100    
#define SAMPLES       384     
#define TRIP_COUNT    3       

uint16_t EEMEM ee_vmax;     
uint16_t EEMEM ee_imax10;    


int vmax = 250;           
int imax10 = 100;              

int relay_on = 0;
int fault = 0;              
int stream_on = 1;

float zero_counts = 512.0;    

char rx_buffer[20];
volatile int rx_len = 0;
volatile int cmd_ready = 0;

// ---------- UART ----------
void uart_init(void)
{
    UBRR0 = 51;                                  
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_char(char c)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
}

void uart_print(const char *s)
{
    while (*s) {
        uart_char(*s);
        s++;
    }
}

void uart_long(long n)
{
    char buf[12];
    ltoa(n, buf, 10);
    uart_print(buf);
}

void uart_float(float f)          
{
    if (f < 0) {
        uart_char('-');
        f = -f;
    }
    long whole = (long)f;
    long frac = (long)((f - whole) * 1000.0 + 0.5);
    if (frac >= 1000) {
        whole++;
        frac -= 1000;
    }
    uart_long(whole);
    uart_char('.');
    if (frac < 100) uart_char('0');
    if (frac < 10)  uart_char('0');
    uart_long(frac);
}

ISR(USART_RX_vect)
{
    char c = UDR0;

    if (cmd_ready) return;                     

    if (c == '\r' || c == '\n') {
        if (rx_len > 0) {
            rx_buffer[rx_len] = 0;
            cmd_ready = 1;
            uart_char('\r');
            uart_char('\n');
        }
    }
    else if (c == 8 || c == 127) {              
        if (rx_len > 0) {
            rx_len--;
            uart_char(8);
            uart_char(' ');
            uart_char(8);
        }
    }
    else if (rx_len < 19) {
        rx_buffer[rx_len] = toupper(c);
        rx_len++;
        uart_char(c);                            
    }
}

void adc_init(void)
{
    ADMUX = (1 << REFS0);                                    // AVCC reference
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);      // clock /64
}

int adc_read(int channel)
{
    ADMUX = (1 << REFS0) | channel;
    _delay_us(10);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

float adc_read_volts(int channel)
{
    long sum = 0;
    int i;

    adc_read(channel);                          
    for (i = 0; i < 16; i++) {
        sum = sum + adc_read(channel);
    }
    return ((float)sum / 16.0) * VREF / 1024.0;
}

// line voltage in VAC
float read_vac(void)
{
    float adc_volts = adc_read_volts(1);       
    float hvdc = adc_volts * DIVIDER;            
    return hvdc / PEAK_FACTOR;                  
}

float sample_current(float zero, float *mean)
{
    float sum = 0;
    float sumsq = 0;
    int i;
    int x;

    ADMUX = (1 << REFS0);                        
    _delay_us(20);
    ADCSRB = 0;
    ADCSRA |= (1 << ADATE) | (1 << ADSC);       

    for (i = 0; i <= SAMPLES; i++) {
        while (!(ADCSRA & (1 << ADIF)));
        ADCSRA |= (1 << ADIF);
        x = ADC;
        if (i == 0) continue;                    
        sum = sum + x;
        sumsq = sumsq + (x - zero) * (x - zero);
    }

    ADCSRA &= ~(1 << ADATE);                      
    while (ADCSRA & (1 << ADSC));
    ADCSRA |= (1 << ADIF);

    *mean = sum / SAMPLES;
    return sqrt(sumsq / SAMPLES);
}

void calibrate_zero(void)
{
    float mean;
    sample_current(512.0, &mean);
    zero_counts = mean;
}


float read_current(void)
{
    float mean;
    float rms_counts = sample_current(zero_counts, &mean);
    float amps = (rms_counts * VREF / 1024.0) / SENSITIVITY;

    if (amps < 0.04) amps = 0; 
    return amps;
}


void relay_set(int on)
{
    relay_on = on;
    if (on) PORTB |= (1 << RELAY_PIN);
    else    PORTB &= ~(1 << RELAY_PIN);
}

void load_settings(void)
{
    vmax = eeprom_read_word(&ee_vmax);
    imax10 = eeprom_read_word(&ee_imax10);

    if (vmax < 50 || vmax > 300) vmax = 250;      // empty eeprom -> default
    if (imax10 < 1 || imax10 > 200) imax10 = 100;
}

void save_settings(void)
{
    eeprom_update_word(&ee_vmax, vmax);
    eeprom_update_word(&ee_imax10, imax10);
}

void print_status(float vac, float amps)
{
    uart_print("V=");
    uart_long((long)(vac + 0.5));
    uart_print(" VAC, I=");
    uart_float(amps);
    uart_print(" A | Vmax=");
    uart_long(vmax);
    uart_print(" VAC, Imax=");
    uart_float(imax10 / 10.0);
    uart_print(" A | RELAY=");
    if (relay_on) uart_print("ON");
    else          uart_print("OFF");
    uart_print(" | FAULT=");
    if (fault == 0) uart_print("NONE");
    if (fault == 1) uart_print("OVERVOLTAGE");
    if (fault == 2) uart_print("OVERCURRENT");
    if (fault == 3) uart_print("MANUAL CUTOFF");
    uart_print("\r\n");
}

void do_command(char *cmd)
{
    float x;

    if (strcmp(cmd, "ON") == 0) {
        if (fault != 0) {
            uart_print("REFUSED: fault latched, send RESET first\r\n");
        }
        else if (read_vac() > vmax || read_current() > imax10 / 10.0) {
            uart_print("REFUSED: value above limit\r\n");
        }
        else {
            relay_set(1);
            uart_print("RELAY ON\r\n");
        }
    }
    else if (strcmp(cmd, "OFF") == 0) {
        relay_set(0);
        fault = 3;
        uart_print("RELAY OFF (manual)\r\n");
    }
    else if (strcmp(cmd, "RESET") == 0) {
        if (read_vac() > vmax) {
            uart_print("REFUSED: voltage still above limit\r\n");
        }
        else {
            fault = 0;
            uart_print("FAULT CLEARED (send ON to close relay)\r\n");
        }
    }
    else if (strncmp(cmd, "SETV", 4) == 0) {
        x = atof(cmd + 4);
        if (x >= 50 && x <= 300) {
            vmax = (int)(x + 0.5);
            save_settings();
            uart_print("Over-voltage cutoff set to ");
            uart_long(vmax);
            uart_print(" VAC\r\n");
        }
        else {
            uart_print("ERROR: usage SETV <50 - 300>\r\n");
        }
    }
    else if (strncmp(cmd, "SETI", 4) == 0) {
        x = atof(cmd + 4);
        if (x >= 0.1 && x <= 20) {
            imax10 = (int)(x * 10 + 0.5);
            save_settings();
            uart_print("Over-current cutoff set to ");
            uart_float(imax10 / 10.0);
            uart_print(" A\r\n");
        }
        else {
            uart_print("ERROR: usage SETI <0.1 - 20>\r\n");
        }
    }
    else if (strcmp(cmd, "ZERO") == 0) {
        if (relay_on) {
            uart_print("REFUSED: send OFF first (no load current allowed)\r\n");
        }
        else {
            calibrate_zero();
            uart_print("Zero calibrated\r\n");
        }
    }
    else if (strcmp(cmd, "ADC") == 0) {
        uart_print("PC0 zero = ");
        uart_float(zero_counts * VREF / 1024.0 * 1000.0);
        uart_print(" mV, PC1 = ");
        uart_float(adc_read_volts(1) * 1000.0);
        uart_print(" mV\r\n");
    }
    else if (strcmp(cmd, "STATUS") == 0) {
        print_status(read_vac(), read_current());
    }
    else if (strcmp(cmd, "STREAM") == 0) {
        if (stream_on) stream_on = 0;
        else           stream_on = 1;
        uart_print("Stream toggled\r\n");
    }
    else if (strcmp(cmd, "?") == 0) {
        uart_print("Commands: ON, OFF, RESET, SETV <VAC>, SETI <amps>, ZERO, ADC, STATUS, STREAM\r\n");
    }
    else {
        uart_print("Unknown command, send ? for help\r\n");
    }
}

int main(void)
{
    float vac;
    float amps;
    int over_v = 0;
    int over_i = 0;
    int report = 0;
    char cmd[20];

    DDRB |= (1 << RELAY_PIN);                     
    relay_set(0);                                

    uart_init();
    adc_init();
    load_settings();
    sei();

    _delay_ms(300);                               
    calibrate_zero();                           
    uart_print("\r\nVoltage / Current Protector\r\n");
    uart_print("Send ? for commands\r\n");

    if (read_vac() <= vmax) {
        relay_set(1);
        uart_print("Relay ON (auto)\r\n");
    }
    else {
        fault = 1;
        uart_print("Voltage above limit - relay kept OFF\r\n");
    }

    while (1) {

       
        if (cmd_ready) {
            strcpy(cmd, rx_buffer);
            rx_len = 0;
            cmd_ready = 0;
            do_command(cmd);
        }

        
        vac = read_vac();
        amps = read_current();

       
        if (relay_on) {
            if (vac > vmax) over_v++;
            else            over_v = 0;

            if (amps > imax10 / 10.0) over_i++;
            else                      over_i = 0;

            if (over_v >= TRIP_COUNT) {
                relay_set(0);
                fault = 1;
                over_v = 0;
                over_i = 0;
                uart_print("!!! TRIP: OVERVOLTAGE - relay OFF\r\n");
                print_status(vac, amps);
            }
            else if (over_i >= TRIP_COUNT) {
                relay_set(0);
                fault = 2;
                over_v = 0;
                over_i = 0;
                uart_print("!!! TRIP: OVERCURRENT - relay OFF\r\n");
                print_status(vac, amps);
            }
        }
        else {
            over_v = 0;
            over_i = 0;
        }

        
        report++;
        if (report >= 20) {
            report = 0;
            if (stream_on && rx_len == 0 && !cmd_ready) {
                print_status(vac, amps);
            }
        }

        _delay_ms(10);
    }

    return 0;
}