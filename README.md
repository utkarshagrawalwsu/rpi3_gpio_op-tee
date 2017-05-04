Boilerplate code taken from Linaro OP-TEE pseudo TA example:
	$(OPTEE_LOCATION)/optee_os/core/arch/arm/pta/stats.c

Pseduo-TA implementation to interact with Raspberry Pi 3 GPIO pin 18.
Exposes the following TA commands:
GPIO_ON
	Send high signal to GPIO 18
GPIO_OFF
	Send low signal to GPIO 18
TEST_PSEUDO_TA
	Test Pseduo TA