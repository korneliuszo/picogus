#pragma once

#include "hardware/gpio.h"
#include "isa_io.pio.h"

#define IOW_PIO_SM 0
#define IOR_PIO_SM 1


constexpr uint32_t iow_rxempty = 1u << (PIO_FSTAT_RXEMPTY_LSB + IOW_PIO_SM);
__force_inline bool iow_has_data() {
    return !(pio0->fstat & iow_rxempty);
}

constexpr uint32_t ior_rxempty = 1u << (PIO_FSTAT_RXEMPTY_LSB + IOR_PIO_SM);
__force_inline bool ior_has_data() {
    return !(pio0->fstat & ior_rxempty);
}

constexpr float iow_clkdiv = (float)rp2_clock / 183000.0;

__force_inline void handle_iow(uint16_t port,uint8_t iow_read);
__force_inline void handle_ior(uint16_t port);

__force_inline void isa_prepare()
{
    for(int i=AD0_PIN; i<(AD0_PIN + 10); ++i) {
        gpio_disable_pulls(i);
    }
    gpio_disable_pulls(IOW_PIN);
    gpio_disable_pulls(IOR_PIN);
    gpio_pull_down(IOCHRDY_PIN);
    gpio_set_dir(IOCHRDY_PIN, GPIO_OUT);

    puts("Enabling bus transceivers...");
    // waggle ADS to set BUSOE latch
    gpio_init(ADS_PIN);
    gpio_set_dir(ADS_PIN, GPIO_OUT);
    gpio_put(ADS_PIN, 1);
    busy_wait_ms(10);
    gpio_put(ADS_PIN, 0);

    puts("Starting ISA bus PIO...");
    // gpio_set_drive_strength(ADS_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(ADS_PIN, GPIO_SLEW_RATE_FAST);

    uint iow_offset = pio_add_program(pio0, &iow_program);
    pio_sm_claim(pio0, IOW_PIO_SM);
    printf("iow sm: %u\n", IOW_PIO_SM);

    uint ior_offset = pio_add_program(pio0, &ior_program);
    pio_sm_claim(pio0, IOR_PIO_SM);
    printf("ior sm: %u\n", IOR_PIO_SM);

    iow_program_init(pio0, IOW_PIO_SM, iow_offset, iow_clkdiv);
    ior_program_init(pio0, IOR_PIO_SM, ior_offset);
}


#ifdef USE_IRQ

void iow_isr(void) {
    /* //printf("ints %x\n", pio0->ints0); */
    uint32_t iow_read = pio_sm_get(pio0, IOW_PIO_SM); //>> 16;
    // printf("%x", iow_read);
    uint16_t port = (iow_read >> 8) & 0x3FF;
    handle_iow(port,iow_read);
    // pio_interrupt_clear(pio0, pio_intr_sm0_rxnempty_lsb);
    irq_clear(PIO0_IRQ_0);
}
void ior_isr(void) {
    uint16_t port = pio_sm_get(pio0, IOR_PIO_SM) & 0x3FF;
    handle_ior(port);
    // pio_interrupt_clear(pio0, PIO_INTR_SM0_RXNEMPTY_LSB);
    irq_clear(PIO0_IRQ_1);
}

__force_inline void isa_irq_prepare()
{
	puts("Enabling IRQ on ISA IOR/IOW events");
	// iow irq
	irq_set_enabled(PIO0_IRQ_0, false);
	pio_set_irq0_source_enabled(pio0, pis_sm0_rx_fifo_not_empty, true);
	irq_set_priority(PIO0_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
	irq_set_exclusive_handler(PIO0_IRQ_0, iow_isr);
	irq_set_enabled(PIO0_IRQ_0, true);
	// ior irq
	irq_set_enabled(PIO0_IRQ_1, false);
	pio_set_irq1_source_enabled(pio0, pis_sm1_rx_fifo_not_empty, true);
	irq_set_priority(PIO0_IRQ_1, PICO_HIGHEST_IRQ_PRIORITY);
	irq_set_exclusive_handler(PIO0_IRQ_1, ior_isr);
	irq_set_enabled(PIO0_IRQ_1, true);
}
#else

__force_inline void isa_poll()
{
    if (iow_has_data()) {
        uint32_t iow_read = pio_sm_get(pio0, IOW_PIO_SM); //>> 16;
        // printf("%x", iow_read);
        uint16_t port = (iow_read >> 8) & 0x3FF;
        handle_iow(port,iow_read);
    }

    if (ior_has_data()) {
        uint16_t port = pio_sm_get(pio0, IOR_PIO_SM) & 0x3FF;
        handle_ior(port);
    }
}

#endif
