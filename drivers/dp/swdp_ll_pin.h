/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

static ALWAYS_INLINE void pin_delay_asm(uint32_t delay)
{
#if defined(CONFIG_CPU_CORTEX_M)
	__asm volatile (".syntax unified\n"
			".start_%=:\n"
			"subs %0, #1\n"
			"bne .start_%=\n"
			: "+l" (delay)
			:
			: "cc"
			);
#elif defined(CONFIG_RISCV)
	__asm volatile("1:\n"
		       "addi %0, %0, -1\n"
		       "bnez %0, 1b\n"
		       : "+r"(delay));
#else
#warning "Pin delay is not defined"
#endif
}

#if defined(CONFIG_SOC_SERIES_NRF52) || defined(CONFIG_SOC_SERIES_NRF53)

#include "swdp_ll_pin_nrf.h"

#elif defined(CONFIG_SOC_FAMILY_STM32)

#include "swdp_ll_pin_stm32.h"

#elif defined(CONFIG_SOC_FAMILY_MCXA)

#include "swdp_ll_pin_mcx.h"

#elif defined(CONFIG_SOC_SERIES_ESP32P4)

#include "swdp_ll_pin_esp32.h"

#else

#define CPU_CLOCK CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
#define FAST_BITBANG_HW_SUPPORT 0

static ALWAYS_INLINE void swdp_ll_pin_input(void *const base, uint8_t pin)
{
}

static ALWAYS_INLINE void swdp_ll_pin_output(void *const base, uint8_t pin)
{
}


static ALWAYS_INLINE void swdp_ll_pin_set(void *const base, uint8_t pin)
{
}

static ALWAYS_INLINE void swdp_ll_pin_clr(void *const base, uint8_t pin)
{
}

static ALWAYS_INLINE uint32_t swdp_ll_pin_get(void *const base, uint8_t pin)
{
	return 0UL;
}

#endif

#ifndef SWDP_LL_HAS_PORT_HOOKS
/*
 * Optional hooks for SoCs that switch the SWD pins to a faster access path
 * while the debug port is on.
 */
static ALWAYS_INLINE void swdp_ll_port_attach(void *const base, uint8_t pin, bool output)
{
}

static ALWAYS_INLINE void swdp_ll_port_detach(void *const base, uint8_t pin)
{
}
#endif

#ifndef CPU_CLOCK
#error "CPU_CLOCK not defined by any soc specific driver"
#endif

#ifndef FAST_BITBANG_HW_SUPPORT
#error "FAST_BITBANG_HW_SUPPORT not defined by any soc specific driver"
#endif
