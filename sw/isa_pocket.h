#pragma once

#include "hardware/gpio.h"
#include "isa_pocket.pio.h"

#define ATTN_PIO_SM 0
#define IOW_PIO_SM 1
#define IOR_PIO_SM 1

constexpr uint32_t attn_rxempty = 1u << (PIO_FSTAT_RXEMPTY_LSB + ATTN_PIO_SM);
__force_inline bool attn_has_data() {
    return !(pio0->fstat & attn_rxempty);
}

__force_inline void handle_iow(uint16_t port,uint8_t iow_read);
__force_inline void handle_ior(uint16_t port);


__force_inline void isa_prepare()
{
    for(int i=AD0_PIN; i<(AD0_PIN + 8); ++i) {
        gpio_disable_pulls(i);
    }

    gpio_init(ADS1_PIN);
    gpio_set_dir(ADS1_PIN, GPIO_OUT);
    gpio_put(ADS1_PIN, 1);
    gpio_init(ADS1_PIN+1);
    gpio_set_dir(ADS1_PIN+1, GPIO_OUT);
    gpio_put(ADS1_PIN+1, 0);

    gpio_init(ATTN_PIN);
    gpio_set_dir(ATTN_PIN, GPIO_IN);
    gpio_init(DACK_PIN);
    gpio_set_dir(DACK_PIN, GPIO_IN);

    puts("Starting ISA bus PIO...");
    // gpio_set_drive_strength(ADS_PIN, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_slew_rate(ADS1_PIN, GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(ADS1_PIN+1, GPIO_SLEW_RATE_FAST);

    uint attn_offset = pio_add_program(pio0, &attn_program);
    pio_sm_claim(pio0, ATTN_PIO_SM);

    uint ior_offset = pio_add_program(pio0, &ior_program);
    pio_sm_claim(pio0, IOR_PIO_SM);


    pio_sm_config c_attn = attn_program_get_default_config(attn_offset);
    pio_sm_config c_ior = ior_program_get_default_config(ior_offset);

    // sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_in_pins(&c_attn, AD0_PIN);
    sm_config_set_in_shift(&c_attn, false, true, 32);

    // Set the pin direction to input at the PIO
    pio_sm_set_consecutive_pindirs(pio0, ATTN_PIO_SM, ATTN_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio0, ATTN_PIO_SM, DACK_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio0, ATTN_PIO_SM, AD0_PIN, 8, false);
    pio_sm_set_consecutive_pindirs(pio0, IOR_PIO_SM, AD0_PIN, 8, false);

    sm_config_set_out_pins(&c_ior, AD0_PIN, 8);
    sm_config_set_out_shift(&c_ior, true, true /* autopull */, 24);
    // Set this pin's GPIO function (connect PIO to the pad)
    for (int i = AD0_PIN; i < AD0_PIN + 8; i++) {
        pio_gpio_init(pio0, i);
    }

    // set up IOCHRDY and ADS
    sm_config_set_sideset_pins(&c_attn, ADS1_PIN);
    sm_config_set_sideset_pins(&c_ior, IOCHRDY_PIN);
    pio_gpio_init(pio0, IOCHRDY_PIN);
    pio_gpio_init(pio0, ADS1_PIN);
    pio_gpio_init(pio0, ADS1_PIN+1);
    pio_gpio_init(pio0, ATTN_PIN);
    pio_gpio_init(pio0, DACK_PIN);
    pio_sm_set_pins_with_mask(pio0, IOR_PIO_SM, 0, 1u << IOCHRDY_PIN);
    pio_sm_set_pins_with_mask(pio0, ATTN_PIO_SM, 3u << ADS1_PIN, 3u << ADS1_PIN);
    pio_sm_set_consecutive_pindirs(pio0, IOR_PIO_SM, IOCHRDY_PIN, 1, true);
    pio_sm_set_consecutive_pindirs(pio0, ATTN_PIO_SM, ADS1_PIN, 2, true);

    sm_config_set_jmp_pin(&c_attn, DACK_PIN);

    // Load our configuration, and jump to the start of the program
    // pio->input_sync_bypass = ((0x1u << IOR_PIN) | (0x3ffu << AD0_PIN));
    pio_sm_init(pio0, ATTN_PIO_SM, attn_offset, &c_attn);
    pio_sm_init(pio0, IOR_PIO_SM, ior_offset, &c_ior);

    // Set the state machine running
    pio_sm_set_enabled(pio0, ATTN_PIO_SM, true);
    pio_sm_set_enabled(pio0, IOR_PIO_SM, true);

}

__force_inline void isa_work()
{
	uint32_t iow_read = pio_sm_get(pio0, ATTN_PIO_SM); //>> 16;
	if (attn_has_data()) // much too late ....
	{
		do {
			pio_sm_get(pio0, ATTN_PIO_SM);
		}
		while(attn_has_data());
		return;
	}
	// printf("%x", iow_read);
	uint16_t port = (iow_read >> 8) & 0x3FF;
	switch(iow_read>>28)
	{
	case 1:
		handle_iow(port,iow_read);
		break;
	case 2:
		handle_ior(port);
		break;
	}
}

#ifdef USE_IRQ

void attn_isr(void) {
	isa_work();
    irq_clear(PIO0_IRQ_0);
}

__force_inline void isa_irq_prepare()
{
	puts("Enabling IRQ on ISA IOR/IOW events");
	// attn irq
	irq_set_enabled(PIO0_IRQ_0, false);
	pio_set_irq0_source_enabled(pio0, pis_sm0_rx_fifo_not_empty, true);
	irq_set_priority(PIO0_IRQ_0, PICO_HIGHEST_IRQ_PRIORITY);
	irq_set_exclusive_handler(PIO0_IRQ_0, attn_isr);
	irq_set_enabled(PIO0_IRQ_0, true);
}

#else

__force_inline void isa_poll()
{
    if (attn_has_data()) {
    	isa_work();
    }
}

#endif
