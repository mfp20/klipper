// Serial Peripheral Interface (SPI) support
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_atmega644p
#include "command.h" // shutdown
#include "gpio.h" // spi_setup
#include "internal.h" // GPIO
#include "pgm.h" // READP
#include "sched.h" // sched_shutdown
#include "generic/serial_irq.h" // serial_rx_byte
#include "avr/interrupt.h" // ISR

DECL_ENUMERATION("spi_bus", "spi", 0);

#if CONFIG_MACH_atmega168 || CONFIG_MACH_atmega328 || CONFIG_MACH_atmega328p
static const uint8_t MISO = GPIO('B', 4), MOSI = GPIO('B', 3);
static const uint8_t SCK = GPIO('B', 5), SS = GPIO('B', 2);
DECL_CONSTANT_STR("BUS_PINS_spi", "PB4,PB3,PB5");
#elif CONFIG_MACH_atmega644p || CONFIG_MACH_atmega1284p
static const uint8_t MISO = GPIO('B', 6), MOSI = GPIO('B', 5);
static const uint8_t SCK = GPIO('B', 7), SS = GPIO('B', 4);
DECL_CONSTANT_STR("BUS_PINS_spi", "PB6,PB5,PB7");
#elif CONFIG_MACH_at90usb1286 || CONFIG_MACH_at90usb646 \
      || CONFIG_MACH_atmega32u4 || CONFIG_MACH_atmega1280 \
      || CONFIG_MACH_atmega2560
static const uint8_t MISO = GPIO('B', 3), MOSI = GPIO('B', 2);
static const uint8_t SCK = GPIO('B', 1), SS = GPIO('B', 0);
DECL_CONSTANT_STR("BUS_PINS_spi", "PB3,PB2,PB1");
#endif

#if CONFIG_AVR_RUNTIME_SPI
static const uint8_t NEEDTX = GPIO('L', 0); // D49
static uint8_t connected = 0;

// prepare next transfer's tx data then trigger the tx gpio to make master start a transaction
void
spi_runtime_tx(void)
{
    uint8_t data;
    int ret = serial_get_tx_byte(&data);
    if (ret) {
        SPDR = MESSAGE_SYNC;
        gpio_out_setup(NEEDTX, 0);
    } else {
    	SPDR = data;
        gpio_out_setup(NEEDTX, 1);
    }
}

// SPI transfer completed
ISR(SPI_STC_vect) {
    if (connected) {
        // save RX data
        serial_rx_byte(SPDR);
        // init next tx (if any)
        spi_runtime_tx();
    } else {
        uint8_t data = SPDR;
        if (data == MESSAGE_SYNC) {
            connected = 1;
            enable_tx_irq = spi_runtime_tx;
            gpio_out_setup(NEEDTX, 0);
        }
    }
}
#endif

static void
spi_init(void)
{
#if CONFIG_AVR_RUNTIME_SPI
    // Set MISO and NEEDTX output, all others input
    gpio_out_setup(MISO, 0);
    gpio_out_setup(NEEDTX, 0);
    gpio_in_setup(MOSI, 0);
    gpio_in_setup(SCK, 0);
    gpio_in_setup(SS, 0);

    // Enable SPI controller and STC interrupt
    SPCR = (1<<SPE) || (1<<SPIE);
    SPSR = 0;
#else
    if (!(GPIO2REGS(SS)->mode & GPIO2BIT(SS)))
        // The SS pin must be an output pin (but is otherwise unused)
        gpio_out_setup(SS, 0);
    gpio_out_setup(SCK, 0);
    gpio_out_setup(MOSI, 0);
    gpio_in_setup(MISO, 0);

    SPCR = (1<<SPE) | (1<<MSTR);
    SPSR = 0;
#endif
}

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus)
        shutdown("Invalid spi_setup parameters");

    // Make sure the SPI interface is enabled
    spi_init();

    struct spi_config config = {0, 0};
    config.spcr = (1<<SPE);

#if CONFIG_AVR_RUNTIME_SPI
    // Enable STC irq
    config.spcr |= (1<<SPIE);

    // Placeholder: rate is uselss in slave mode
#else
    // Enable Master role
    config.spcr |= (1<<MSTR);

    // Setup rate
    if (rate >= (CONFIG_CLOCK_FREQ / 2)) {
        config.spsr = (1<<SPI2X);
    } else if (rate >= (CONFIG_CLOCK_FREQ / 4)) {
        config.spcr = 0;
    } else if (rate >= (CONFIG_CLOCK_FREQ / 8)) {
        config.spcr = 1;
        config.spsr = (1<<SPI2X);
    } else if (rate >= (CONFIG_CLOCK_FREQ / 16)) {
        config.spcr = 1;
    } else if (rate >= (CONFIG_CLOCK_FREQ / 32)) {
        config.spcr = 2;
        config.spsr = (1<<SPI2X);
    } else if (rate >= (CONFIG_CLOCK_FREQ / 64)) {
        config.spcr = 2;
    } else {
        config.spcr = 3;
    }
#endif
    // Setup mode
    config.spcr |= (mode << CPHA);

    return config;
}

#if CONFIG_AVR_RUNTIME_SPI
// Perform runtime handshake
void
spi_runtime_init(void)
{
    if (connected) {
        // Note: once connected the task could be removed... how?
        return;
    }
    spi_setup(0,0,0);
    gpio_out_setup(NEEDTX, 1);
}
DECL_TASK(spi_runtime_init);
#endif

void
spi_prepare(struct spi_config config)
{
    SPCR = config.spcr;
    SPSR = config.spsr;
}

void
spi_transfer(struct spi_config config, uint8_t receive_data
             , uint8_t len, uint8_t *data)
{
    if (receive_data) {
        while (len--) {
            SPDR = *data;
            while (!(SPSR & (1<<SPIF)))
                ;
            *data++ = SPDR;
        }
    } else {
        while (len--) {
            SPDR = *data++;
            while (!(SPSR & (1<<SPIF)))
                ;
        }
    }
}
