/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <hal/gpio_ll.h>
#include <hal/dedic_gpio_cpu_ll.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_pins.h>
#include <soc/soc_caps.h>
#include <soc/gpio_sig_map.h>

/*
 * The CPU clock, not the kernel tick source: pin_delay_asm() burns CPU
 * cycles, and CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC is the system timer rate.
 */
#define CPU_CLOCK               DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency)
#define FAST_BITBANG_HW_SUPPORT 1

/*
 * Every access to the GPIO block crosses the peripheral bus and takes about
 * 250 ns, which limits SWCLK to roughly 1 MHz. While the debug port is on,
 * the SWD pins are instead routed through the GPIO matrix to the CPU's
 * dedicated GPIO channels, whose output, output enable and input are CPU
 * CSRs accessed in a single cycle. Pins that are not routed (before the port
 * is switched on, or the reset pin) use the GPIO block.
 */
#define SWDP_LL_HAS_PORT_HOOKS 1

#define SWDP_ESP32_NUM_GPIO SOC_GPIO_PIN_COUNT
#define SWDP_ESP32_NUM_CHAN 8U
#define SWDP_ESP32_NO_CHAN  0xFFU

static uint8_t swdp_esp32_chan[SWDP_ESP32_NUM_GPIO] = {
	[0 ... (SWDP_ESP32_NUM_GPIO - 1)] = SWDP_ESP32_NO_CHAN,
};
static uint8_t swdp_esp32_chan_used;

/*
 * The SoC has a single GPIO block. Zephyr splits it into gpio0 (pins 0-31)
 * and gpio1 (pins 32 and up), and the gpio1 node address only identifies
 * the upper bank, so turn the node address and pin into a GPIO number.
 */
static ALWAYS_INLINE uint32_t swdp_ll_pin_num(void *const base, uint8_t pin)
{
#if DT_NODE_EXISTS(DT_NODELABEL(gpio1))
	if ((uintptr_t)base == DT_REG_ADDR(DT_NODELABEL(gpio1))) {
		return pin + 32U;
	}
#endif

	return pin;
}

/* Route a pin to a dedicated GPIO channel, initially as output or input */
static inline void swdp_ll_port_attach(void *const base, uint8_t pin, bool output)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch == SWDP_ESP32_NO_CHAN) {
		if (swdp_esp32_chan_used == BIT_MASK(SWDP_ESP32_NUM_CHAN)) {
			return;
		}

		ch = (uint8_t)find_lsb_set(~swdp_esp32_chan_used) - 1U;
		swdp_esp32_chan_used |= BIT(ch);
	}

	/* Keep the current level so the pin does not glitch */
	if (gpio_ll_get_level(&GPIO, num) != 0) {
		RV_SET_CSR(CSR_GPIO_OUT_USER, BIT(ch));
	} else {
		RV_CLEAR_CSR(CSR_GPIO_OUT_USER, BIT(ch));
	}

	if (output) {
		RV_CLEAR_CSR(CSR_GPIO_OEN_USER, BIT(ch));
	} else {
		RV_SET_CSR(CSR_GPIO_OEN_USER, BIT(ch));
	}

	gpio_ll_input_enable(&GPIO, num);
	esp_rom_gpio_connect_in_signal(num, CORE_GPIO_IN_PAD_IN0_IDX + ch, false);
	esp_rom_gpio_connect_out_signal(num, CORE_GPIO_OUT_PAD_OUT0_IDX + ch, false, false);

	swdp_esp32_chan[num] = ch;
}

/* Hand a pin back to the GPIO block */
static inline void swdp_ll_port_detach(void *const base, uint8_t pin)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch == SWDP_ESP32_NO_CHAN) {
		return;
	}

	gpio_ll_set_level(&GPIO, num, (RV_READ_CSR(CSR_GPIO_OUT_USER) >> ch) & 1U);
	esp_rom_gpio_connect_out_signal(num, SIG_GPIO_OUT_IDX, false, false);
	esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, CORE_GPIO_IN_PAD_IN0_IDX + ch,
				       false);

	swdp_esp32_chan[num] = SWDP_ESP32_NO_CHAN;
	swdp_esp32_chan_used &= ~BIT(ch);
}

static ALWAYS_INLINE void swdp_ll_pin_input(void *const base, uint8_t pin)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch != SWDP_ESP32_NO_CHAN) {
		RV_SET_CSR(CSR_GPIO_OEN_USER, BIT(ch));
	} else {
		gpio_ll_output_disable(&GPIO, num);
		gpio_ll_input_enable(&GPIO, num);
	}
}

static ALWAYS_INLINE void swdp_ll_pin_output(void *const base, uint8_t pin)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch != SWDP_ESP32_NO_CHAN) {
		RV_CLEAR_CSR(CSR_GPIO_OEN_USER, BIT(ch));
	} else {
		gpio_ll_output_enable(&GPIO, num);
	}
}

static ALWAYS_INLINE void swdp_ll_pin_set(void *const base, uint8_t pin)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch != SWDP_ESP32_NO_CHAN) {
		RV_SET_CSR(CSR_GPIO_OUT_USER, BIT(ch));
	} else {
		gpio_ll_set_level(&GPIO, num, 1U);
	}
}

static ALWAYS_INLINE void swdp_ll_pin_clr(void *const base, uint8_t pin)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch != SWDP_ESP32_NO_CHAN) {
		RV_CLEAR_CSR(CSR_GPIO_OUT_USER, BIT(ch));
	} else {
		gpio_ll_set_level(&GPIO, num, 0U);
	}
}

static ALWAYS_INLINE uint32_t swdp_ll_pin_get(void *const base, uint8_t pin)
{
	uint32_t num = swdp_ll_pin_num(base, pin);
	uint8_t ch = swdp_esp32_chan[num];

	if (ch != SWDP_ESP32_NO_CHAN) {
		return (RV_READ_CSR(CSR_GPIO_IN_USER) >> ch) & 1U;
	}

	return (uint32_t)gpio_ll_get_level(&GPIO, num);
}
