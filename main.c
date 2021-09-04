#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>

#define RCC_USART RCC_USART2
#define USART         USART2
#define USART_GPIO GPIO2
#define USART_AF GPIO_AF7
#define USART_BAUD 115200

#define I2C_SLAVE_ADDR 0x69  // 105d


void sys_tick_handler(void);

static volatile uint64_t _millis = 0;

static void systick_setup(void) {
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    STK_CVR = 0;
    systick_set_reload(rcc_ahb_frequency / 1000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

static void rcc_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_USART);
}


static void usart_setup(void) {
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_GPIO);
    gpio_set_af(GPIOA, USART_AF, USART_GPIO);
    usart_set_baudrate(USART, USART_BAUD);
    usart_set_databits(USART, 8);
    usart_set_parity(USART, USART_PARITY_NONE);
    usart_set_stopbits(USART, USART_CR2_STOPBITS_1);
    usart_set_mode(USART, USART_MODE_TX);
    usart_set_flow_control(USART, USART_FLOWCONTROL_NONE);
    usart_enable(USART);
    setbuf(stdout, NULL);
}


void uart_puts(char *string) {
    while (*string) {
        usart_send_blocking(USART, *string);
        string++;
    }
}

void uart_putln(char *string) {
    uart_puts(string);
    uart_puts("\r\n");
}

uint64_t millis(void) {
    return _millis;
}

void sys_tick_handler(void) {
    _millis++;
}

void delay(uint64_t duration) {
    const uint64_t until = millis() + duration;
    while(millis() < until);
}


int main(void) {
    rcc_setup();

    // Our test LED is connected to Port B pin 3, so let's set it as output
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

    systick_setup();

    usart_setup();

    // Now, let's forever toggle this LED back and forth
    while (true) {
        delay(500);
        uart_putln("LED toggle");
        gpio_toggle(GPIOB, GPIO3);
    }

    return 0;
}
