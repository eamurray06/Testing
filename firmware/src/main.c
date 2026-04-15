#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "../step_detector.h"

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define UART_BAUD 115200UL
#define UBRR_VALUE ((F_CPU / (16UL * UART_BAUD)) - 1)

#define MPU_ADDR 0x68
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_CONFIG 0x1A
#define MPU_ACCEL_CONFIG 0x1C

static volatile uint32_t g_millis = 0;

ISR(TIMER0_COMPA_vect) {
    g_millis++;
}

static uint32_t millis(void) {
    uint32_t now;
    uint8_t old_sreg = SREG;
    cli();
    now = g_millis;
    SREG = old_sreg;
    return now;
}

static void timer0_init(void) {
    // 1 ms tick @ 16 MHz with prescaler 64: 16e6/64 = 250k, OCR0A=249 => 1 kHz
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
}

static void uart_init(void) {
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)(UBRR_VALUE & 0xFF);
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

static void uart_write_char(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {
    }
    UDR0 = c;
}

static void uart_write_str(const char *s) {
    while (*s) {
        uart_write_char(*s++);
    }
}

static int uart_printf_putchar(char c, FILE *stream) {
    (void)stream;
    if (c == '\n') {
        uart_write_char('\r');
    }
    uart_write_char(c);
    return 0;
}

static FILE uart_stdout = FDEV_SETUP_STREAM(uart_printf_putchar, NULL, _FDEV_SETUP_WRITE);

static void i2c_init(void) {
    TWSR = 0x00;
    TWBR = 72;  // ~100 kHz @ 16 MHz
    TWCR = (1 << TWEN);
}

static uint8_t i2c_start(uint8_t address_rw) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
    if ((TWSR & 0xF8) != 0x08 && (TWSR & 0xF8) != 0x10) return 0;

    TWDR = address_rw;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }

    uint8_t status = TWSR & 0xF8;
    if (address_rw & 1) return status == 0x40;
    return status == 0x18;
}

static void i2c_stop(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static uint8_t i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
    return (TWSR & 0xF8) == 0x28;
}

static uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT))) {
    }
    return TWDR;
}

static uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {
    }
    return TWDR;
}

static uint8_t mpu_write(uint8_t reg, uint8_t value) {
    if (!i2c_start((MPU_ADDR << 1) | 0)) return 0;
    if (!i2c_write(reg)) return 0;
    if (!i2c_write(value)) return 0;
    i2c_stop();
    return 1;
}

static uint8_t mpu_read_bytes(uint8_t start_reg, uint8_t *buffer, uint8_t len) {
    if (!i2c_start((MPU_ADDR << 1) | 0)) return 0;
    if (!i2c_write(start_reg)) return 0;

    if (!i2c_start((MPU_ADDR << 1) | 1)) return 0;
    for (uint8_t i = 0; i < len; i++) {
        buffer[i] = (i < (len - 1)) ? i2c_read_ack() : i2c_read_nack();
    }
    i2c_stop();
    return 1;
}

static uint8_t mpu_init(void) {
    _delay_ms(100);
    if (!mpu_write(MPU_PWR_MGMT_1, 0x00)) return 0;
    _delay_ms(10);
    if (!mpu_write(MPU_CONFIG, 0x03)) return 0;
    if (!mpu_write(MPU_ACCEL_CONFIG, 0x00)) return 0;  // +-2 g
    return 1;
}

static uint8_t imu_read_g(float *ax, float *ay, float *az) {
    uint8_t raw[6];
    if (!mpu_read_bytes(MPU_ACCEL_XOUT_H, raw, 6)) return 0;

    int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t y = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t z = (int16_t)((raw[4] << 8) | raw[5]);

    const float scale = 16384.0f;
    *ax = x / scale;
    *ay = y / scale;
    *az = z / scale;
    return 1;
}

int main(void) {
    uart_init();
    stdout = &uart_stdout;

    i2c_init();
    timer0_init();
    sei();

    StepDetector detector;
    step_detector_init(&detector);

    printf("BOOT,ATmega328PB step counter\n");
    if (!mpu_init()) {
        printf("ERROR,MPU init failed\n");
        while (1) {
            _delay_ms(500);
        }
    }

    uint32_t last_sample = 0;
    const uint16_t sample_interval_ms = 20;  // 50 Hz

    while (1) {
        uint32_t now = millis();
        if ((uint32_t)(now - last_sample) < sample_interval_ms) {
            continue;
        }
        last_sample = now;

        float ax, ay, az, threshold;
        if (!imu_read_g(&ax, &ay, &az)) {
            printf("WARN,imu_read_failed\n");
            continue;
        }

        uint8_t is_step = step_detector_update(&detector, now, ax, ay, az, &threshold);

        // CSV for laptop parser
        printf("%lu,%.4f,%.4f,%.4f\n", (unsigned long)now, ax, ay, az);

        if (is_step) {
            printf("STEP,%lu,%lu,thr=%.3f\n", (unsigned long)now, (unsigned long)detector.steps, threshold);
        }
    }
}
