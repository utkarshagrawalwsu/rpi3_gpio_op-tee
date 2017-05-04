/*
 * Copyright (c) 2015, Linaro Limited
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <compiler.h>
#include <stdio.h>
#include <trace.h>
#include <kernel/pseudo_ta.h>
#include <mm/tee_pager.h>
#include <mm/tee_mm.h>
#include <string.h>
#include <string_ext.h>
#include <malloc.h>

#include <mm/core_mmu.h>
#include <mm/core_memprot.h>
#include <io.h>
// #include <mach/platform.h>

#define TA_NAME     "stats.ta"

#define STATS_UUID \
		{ 0xd96a5b40, 0xe2c7, 0xb1af, \
			{ 0x87, 0x94, 0x10, 0x02, 0xa5, 0xd5, 0xc6, 0x1b } }

#define GPIO_ON             0
#define GPIO_OFF            1
#define TEST_PSEUDO_TA      2

#define RPI3_PERI_BASE      0x3F000000
#define GPIO_BASE           (RPI3_PERI_BASE + 0x200000)

typedef enum {
	GPIO_FSEL_INPT  = 0x00,
	GPIO_FSEL_OUTP  = 0x01
} FunctionSelect_GPFSEL;

typedef enum {
	GPIO_LOW     =  0x0,
	GPIO_HIGH    =  0x1
} GPIO_Value;

struct GpioRegister {
	uint32_t GPFSEL[6];
	uint32_t Reserved1;
	uint32_t GPSET[2];
	uint32_t Reserved2;
	uint32_t GPCLR[2];
};

struct GpioRegister *gpioRegister;

static vaddr_t nsec_periph_base(paddr_t pa) {
	if (cpu_mmu_enabled()) {
		return (vaddr_t)phys_to_virt(pa, MEM_AREA_IO_NSEC);
	}
	return (vaddr_t)pa;
}

static vaddr_t get_base_address_GPFSEL(uint8_t index) {
	return nsec_periph_base((paddr_t)&(gpioRegister->GPFSEL[index]));
}

static vaddr_t get_base_address_GPSET(uint8_t index) {
	return nsec_periph_base((paddr_t)&(gpioRegister->GPSET[index]));
}

static vaddr_t get_base_address_GPCLR(uint8_t index) {
	return nsec_periph_base((paddr_t)&(gpioRegister->GPCLR[index]));
}

static void set_gpio_pin_function(uint8_t pinNumber, FunctionSelect_GPFSEL functionCode) {
	uint32_t index_GPFSEL = pinNumber / 10;
	uint32_t bit_GPFSEL = (pinNumber % 10) * 3;

	vaddr_t base_addr_GPFSEL = get_base_address_GPFSEL(index_GPFSEL);
	uint32_t mask = 0x7 << bit_GPFSEL;

	uint32_t data;

	data = read32(base_addr_GPFSEL);

	data = data & ~mask;
	data = data | (functionCode << bit_GPFSEL);
	write32(data, base_addr_GPFSEL);
}

static void set_gpio_pin_value(uint8_t pinNumber, GPIO_Value value) {
	uint32_t index_register = pinNumber / 32;
	uint32_t bit_register = pinNumber % 32;

	vaddr_t base_addr = (value == GPIO_LOW) ? get_base_address_GPCLR(index_register) : get_base_address_GPSET(index_register);
	uint32_t mask = 0x1 << bit_register;

	uint32_t data;

	data = read32(base_addr);

	data = data & ~mask;
	data = data | (0x1 << bit_register);
	write32(data, base_addr);
}

static void initializeGpio(void) {
	gpioRegister = (struct GpioRegister *)(GPIO_BASE);
}

static TEE_Result testGpioOn(void) {
	static const int LedGpioPin = 18;

	initializeGpio();

	set_gpio_pin_function(LedGpioPin, GPIO_FSEL_OUTP);
	set_gpio_pin_value(LedGpioPin, GPIO_HIGH);
	
	return TEE_SUCCESS;
}

static TEE_Result testGpioOff(void) {
	static const int LedGpioPin = 18;

	initializeGpio();

	set_gpio_pin_function(LedGpioPin, GPIO_FSEL_OUTP);
	set_gpio_pin_value(LedGpioPin, GPIO_LOW);
	
	return TEE_SUCCESS;
}

static TEE_Result testPseudoTA(uint32_t type, TEE_Param p[TEE_NUM_PARAMS]) {
	if (TEE_PARAM_TYPES(
			TEE_PARAM_TYPE_VALUE_OUTPUT,
			TEE_PARAM_TYPE_NONE,
			TEE_PARAM_TYPE_NONE,
			TEE_PARAM_TYPE_NONE) != type) {
		EMSG("Invalid arguments");
		return TEE_ERROR_BAD_PARAMETERS;
	}
	p[0].value.a = 20;
	return TEE_SUCCESS;
}

static TEE_Result invoke_command(
	void *psess __unused,
	uint32_t cmd,
	uint32_t ptypes, TEE_Param params[TEE_NUM_PARAMS]
) {
	(void)ptypes;
	(void)params;

	switch (cmd) {
		case GPIO_ON:
			return testGpioOn();
		case GPIO_OFF:
			return testGpioOff();
		case TEST_PSEUDO_TA:
			return testPseudoTA(ptypes, params);
		default:
			break;
	}
	return TEE_ERROR_BAD_PARAMETERS;
}

pseudo_ta_register(.uuid = STATS_UUID, .name = TA_NAME,
		   .flags = PTA_DEFAULT_FLAGS,
		   .invoke_command_entry_point = invoke_command);
