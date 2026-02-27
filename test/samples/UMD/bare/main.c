#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

/*
 * Pin mapping (ATmega328P port/pin):
 *   AIN1  -> PB1 (D9)
 *   AIN2  -> PB0 (D8)
 *   PWM   -> PD5 (D5)  - OC0B (Timer0 PWM)
 *   STBY  -> PD6 (D6)
 *   TRIG  -> PB3 (D11)
 *   ECHO  -> PB4 (D12)
 */

#define AIN1_PORT   PORTB
#define AIN1_DDR    DDRB
#define AIN1_PIN    PB1

#define AIN2_PORT   PORTB
#define AIN2_DDR    DDRB
#define AIN2_PIN    PB0

#define STBY_PORT   PORTD
#define STBY_DDR    DDRD
#define STBY_PIN    PD6

#define PWM_DDR     DDRD
#define PWM_PIN     PD5   // OC0B

#define TRIG_PORT   PORTB
#define TRIG_DDR    DDRB
#define TRIG_PIN    PB3

#define ECHO_DDR    DDRB
#define ECHO_INPUT  PINB
#define ECHO_PIN    PB4

// USART
void uart_init(void);
void uart_putchar(char c);
void uart_print(const char *s);
void uart_print_double(double val);

// PWM (Timer0, Fast PWM, OC0B = PD5)
void pwm_init(void);
void pwm_set(uint8_t duty);

// TB6612 Motor Control 
void motor_init(void);

// speed: -255 to +255
void motor_drive(int16_t speed);
void motor_brake(void);

// HC-SR04
void hcsr04_init(void);

// Returns distance in cm, or -1.0 on timeout
double hcsr04_measure(void);

int main(void) {
    uart_init();
    motor_init();
    hcsr04_init();

    while (1) {
        double distance = hcsr04_measure();

        uart_print("Distance: ");
        uart_print_double(distance);
        uart_print(" cm\r\n");

        if (distance > 30.0 || distance < 0.0) {
            motor_drive(200);
        } else {
            motor_brake();
        }

        _delay_ms(200);
    }

    return 0;
}

// USART
void uart_init(void) {
    // 9600 baud @ 16MHz: UBRR = (F_CPU / (16 * BAUD)) - 1 = 103
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);                        // TX enable only
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);      // 8N1
}

void uart_putchar(char c) {
    while (!(UCSR0A & (1 << UDRE0)));             // wait for empty buffer
    UDR0 = c;
}

void uart_print(const char *s) {
    while (*s) uart_putchar(*s++);
}

void uart_print_double(double val) {
    char buf[16];
    int whole = (int)val;
    int frac  = (int)((val - whole) * 100);
    if (frac < 0) frac = -frac;
    snprintf(buf, sizeof(buf), "%d.%02d", whole, frac);
    uart_print(buf);
}

// PWM (Timer0, Fast PWM, OC0B = PD5)
void pwm_init(void) {
    PWM_DDR |= (1 << PWM_PIN);
    // Fast PWM, non-inverting on OC0B, prescaler /64 => ~980Hz
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01)   | (1 << CS00);
    OCR0B  = 0;
}

void pwm_set(uint8_t duty) {   // 0–255
    OCR0B = duty;
}

// TB6612 Motor Control 
void motor_init(void) {
    AIN1_DDR  |= (1 << AIN1_PIN);
    AIN2_DDR  |= (1 << AIN2_PIN);
    STBY_DDR  |= (1 << STBY_PIN);
    STBY_PORT |= (1 << STBY_PIN);   // take out of standby
    pwm_init();
}

// speed: -255 to +255
void motor_drive(int16_t speed) {
    if (speed > 0) {
        AIN1_PORT |=  (1 << AIN1_PIN);
        AIN2_PORT &= ~(1 << AIN2_PIN);
        pwm_set((uint8_t)speed);
    } else if (speed < 0) {
        AIN1_PORT &= ~(1 << AIN1_PIN);
        AIN2_PORT |=  (1 << AIN2_PIN);
        pwm_set((uint8_t)(-speed));
    } else {
        pwm_set(0);
    }
}

void motor_brake(void) {
    AIN1_PORT |= (1 << AIN1_PIN);
    AIN2_PORT |= (1 << AIN2_PIN);
    pwm_set(255);   // full brake (both inputs HIGH = brake on TB6612)
}

// HC-SR04
void hcsr04_init(void) {
    TRIG_DDR  |=  (1 << TRIG_PIN);   // trigger = output
    ECHO_DDR  &= ~(1 << ECHO_PIN);   // echo    = input
    TRIG_PORT &= ~(1 << TRIG_PIN);   // trigger low
}

// Returns distance in cm, or -1.0 on timeout
double hcsr04_measure(void) {
    // Send 10us trigger pulse
    TRIG_PORT |=  (1 << TRIG_PIN);
    _delay_us(10);
    TRIG_PORT &= ~(1 << TRIG_PIN);

    // Wait for echo HIGH (with timeout)
    uint16_t timeout = 0;
    while (!(ECHO_INPUT & (1 << ECHO_PIN))) {
        if (++timeout > 60000) return -1.0;   // no object detected
    }

    // Count how long echo stays HIGH
    // Each iteration ~4 cycles @ 16MHz = 0.25us
    uint16_t count = 0;
    while (ECHO_INPUT & (1 << ECHO_PIN)) {
        count++;
        _delay_us(1);
        if (count > 23200) return -1.0;   // > 400cm, bail
    }

    // Sound: 343 m/s = 0.0343 cm/us
    // distance = (count * 1us * 0.0343) / 2
    return (double)count * 0.01715;
}
