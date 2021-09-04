#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#define RCC_USART RCC_USART2
#define USART         USART2
#define USART_PORT GPIOA
#define USART_GPIO GPIO2
#define USART_AF GPIO_AF7
#define USART_BAUD 115200

#define I2C_SLAVE_ADDR 0x69  // 105d
#define RCC_I2C RCC_I2C1
#define I2C         I2C1
#define I2C_PORT     GPIOB
#define I2C_SDA_GPIO GPIO7
#define I2C_SCL_GPIO GPIO6
#define I2C_AF GPIO_AF4
#define I2C_OTYPE GPIO_OTYPE_OD
#define I2C_GPIO_SPEED GPIO_OSPEED_50MHZ



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
    rcc_periph_clock_enable(RCC_I2C);
}

static void i2c_setup(void) {
    //nvic_enable_irq(NVIC_I2C1_EV_IRQ);

    gpio_mode_setup(
            I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE,
            I2C_SCL_GPIO | I2C_SDA_GPIO);
    gpio_set_output_options(
            I2C_PORT, I2C_OTYPE, I2C_GPIO_SPEED,
            I2C_SCL_GPIO | I2C_SDA_GPIO);
    gpio_set_af(I2C_PORT, I2C_AF, I2C_SCL_GPIO | I2C_SDA_GPIO);

    i2c_reset(I2C);
    i2c_peripheral_disable(I2C);

    i2c_enable_analog_filter(I2C);
    i2c_set_digital_filter(I2C, 0);

    i2c_set_speed(I2C, i2c_speed_sm_100k, rcc_apb1_frequency / 1e6);
    //i2c_set_own_7bit_slave_address(I2C, I2C_SLAVE_ADDR);
    //i2c_enable_interrupt(
    //        I2C, 
    //        I2C_CR1_ADDRIE | I2C_CR1_RXIE);

    i2c_peripheral_enable(I2C);
}



static void usart_setup(void) {
    gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_GPIO);
    gpio_set_af(USART_PORT, USART_AF, USART_GPIO);
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

void i2c1_ev_isr(void) {
    uart_putln("evt");
    uint32_t isr = I2C_ISR(I2C);

    if (isr & I2C_ISR_ADDR) {
        // Address matched
        uart_putln("adr");
        I2C_ICR(I2C) &= ~(I2C_ICR_ADDRCF);
    }

}


int main(void) {
    rcc_setup();

    // Our test LED is connected to Port B pin 3, so let's set it as output
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);

    systick_setup();

    usart_setup();

    i2c_setup();

    uint8_t addr = 0x20;
    uint8_t cmd = 0x42;
    uint8_t data;

    // Now, let's forever toggle this LED back and forth
    while (true) {
        uart_putln("LED toggle");
        gpio_toggle(GPIOB, GPIO3);
        uart_putln("i2c send");
        i2c_transfer7(I2C, addr, &cmd, 1, &data, 1);

        delay(1000);
    }

    return 0;
}
