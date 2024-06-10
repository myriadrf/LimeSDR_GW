// This file is Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
// License: BSD

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <irq.h>
#include <libbase/uart.h>
#include <libbase/console.h>
#include <generated/csr.h>

#include "i2c0.h"
#include "i2c1.h"

/*-----------------------------------------------------------------------*/
/* Constants                                                             */
/*-----------------------------------------------------------------------*/
#define LP8758_I2C_ADDR  0x60


/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

static char *readstr(void)
{
	char c[2];
	static char s[64];
	static int ptr = 0;

	if(readchar_nonblock()) {
		c[0] = getchar();
		c[1] = 0;
		switch(c[0]) {
			case 0x7f:
			case 0x08:
				if(ptr > 0) {
					ptr--;
					fputs("\x08 \x08", stdout);
				}
				break;
			case 0x07:
				break;
			case '\r':
			case '\n':
				s[ptr] = 0x00;
				fputs("\n", stdout);
				ptr = 0;
				return s;
			default:
				if(ptr >= (sizeof(s) - 1))
					break;
				fputs(c, stdout);
				s[ptr] = c[0];
				ptr++;
				break;
		}
	}

	return NULL;
}

static char *get_token(char **str)
{
	char *c, *d;

	c = (char *)strchr(*str, ' ');
	if(c == NULL) {
		d = *str;
		*str = *str+strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c+1;
	return d;
}

static void prompt(void)
{
	printf("\e[92;1mlitex-demo-app\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

static void help(void)
{
	puts("\nLiteX minimal demo app built "__DATE__" "__TIME__"\n");
	puts("Available commands:");
	puts("help               - Show this command");
	puts("reboot             - Reboot CPU");
	puts("i2c_test           - Test I2C Buses");
	puts("init_pmic          - Initialize DC-DC switching regulators");
	puts("dump_pmic          - Dump DC-DC switching regulator configuration");
#ifdef CSR_LEDS_BASE
	puts("led                - Led demo");
#endif
#ifdef CSR_GPIO_LED_BASE
	puts("gpioled            - GPIO override demo");
#endif
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

static void reboot_cmd(void)
{
	ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE
static void led_cmd(void)
{
	int i;
	printf("Led demo...\n");

	printf("Counter mode...\n");
	for(i=0; i<32; i++) {
		leds_out_write(i);
		busy_wait(100);
	}

	printf("Shift mode...\n");
	for(i=0; i<4; i++) {
		leds_out_write(1<<i);
		busy_wait(200);
	}
	for(i=0; i<4; i++) {
		leds_out_write(1<<(3-i));
		busy_wait(200);
	}

	printf("Dance mode...\n");
	for(i=0; i<4; i++) {
		leds_out_write(0x55);
		busy_wait(200);
		leds_out_write(0xaa);
		busy_wait(200);
	}
}
#endif

#ifdef CSR_GPIO_LED_BASE
static void gpioled_cmd(void)
{
	int i;
	printf("GPIO Led override demo...\n");
	gpio_led_control_gpio_override_dir_write(0x0);
	gpio_led_control_gpio_override_write(0x1);
	for(i=0; i<32; i++) {
		gpio_led_control_gpio_override_val_write(0x0);
		busy_wait(100);
		gpio_led_control_gpio_override_val_write(0x1);
		busy_wait(100);
	}

	gpio_led_control_gpio_override_write(0x0);
	printf("GPIO Led override demo...ended\n");
}
#endif

/*-----------------------------------------------------------------------*/
/* I2C                                                                   */
/*-----------------------------------------------------------------------*/

static void i2c_test(void)
{
	printf("I2C0 Scan...\n");
	i2c0_scan();

	printf("\n");

	printf("I2C1 Scan...\n");
	i2c1_scan();
}

static void dump_pmic(void){
	unsigned char adr;
	unsigned char dat;
	printf("FPGA_I2C1 PMIC Dump...\n");
	for (adr=0; adr<32; adr++) {
		i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
	printf("FPGA_I2C2 PMIC Dump...\n");
	for (adr=0; adr<32; adr++) {
		i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
		printf("0x%02x: 0x%02x\n", adr, dat);
	}
}

static void init_pmic(void)
{
	printf("Initializing DC-DC switching regulators...\n");

	unsigned char adr;
	unsigned char dat;

	printf("PMICs Initialization...\n");
	printf("-----------------------\n");

	printf("FPGA_I2C1 PMIC: Check ID ");
	adr = 0x01;
	i2c0_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0) {
		printf("KO, exiting.\n");
	} else {
		printf("OK.\n");

		printf("PMIC: Enable Buck0.\n");
		adr = 0x02;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
		adr = 0x03;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck1.\n");
		adr = 0x04;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
		adr = 0x05;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck2.\n");
		adr = 0x06;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
		adr = 0x07;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck3.\n");
		adr = 0x08;
		dat = 0x88;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
		adr = 0x09;
		dat = 0xD2;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck1 to 3.3V.\n");
		adr = 0x0C;
		dat = 0xFC;
		i2c0_write(LP8758_I2C_ADDR, adr, &dat, 1);

		busy_wait(1);
	}


	printf("FPGA_I2C2 PMIC: Check ID ");
	adr = 0x01;
	i2c1_read(LP8758_I2C_ADDR, adr, &dat, 1, true);
	if (dat != 0xe0) {
		printf("KO, exiting.\n");
	} else {
		printf("OK.\n");

		printf("PMIC: Enable Buck0.\n");
		adr = 0x02;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM0=2.5A, SLEW_RATE0=10mV/uS.\n");
		adr = 0x03;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck1.\n");
		adr = 0x04;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM1=2.5A, SLEW_RATE1=10mV/uS.\n");
		adr = 0x05;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck2.\n");
		adr = 0x06;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM2=2.5A, SLEW_RATE2=10mV/uS.\n");
		adr = 0x07;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Enable Buck3.\n");
		adr = 0x08;
		dat = 0x88;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: ILIM3=2.5A, SLEW_RATE3=10mV/uS.\n");
		adr = 0x09;
		dat = 0xD2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck0 to 1.5V.\n");
		adr = 0x0A;
		dat = 0xA2;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck1 to 3.3V.\n");
		adr = 0x0C;
		dat = 0xFC;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck2 to 1.75V.\n");
		adr = 0x0E;
		dat = 0xAF;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Set Buck3 to 2.05V.\n");
		adr = 0x10;
		dat = 0xBE;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		printf("PMIC: Clear INT_BUCK_2_3 Status.\n");
		adr = 0x1A;
		dat = 0xFF;
		i2c1_write(LP8758_I2C_ADDR, adr, &dat, 1);

		busy_wait(1);
	}

}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

static void console_service(void)
{
	char *str;
	char *token;

	str = readstr();
	if(str == NULL) return;
	token = get_token(&str);
	if(strcmp(token, "help") == 0)
		help();
	else if(strcmp(token, "reboot") == 0)
		reboot_cmd();
	else if(strcmp(token, "i2c_test") == 0)
		i2c_test();
	else if(strcmp(token, "init_pmic") == 0)
		init_pmic();
	else if(strcmp(token, "dump_pmic") == 0)
		dump_pmic();
#ifdef CSR_LEDS_BASE
	else if(strcmp(token, "led") == 0)
		led_cmd();
#endif
#ifdef CSR_GPIO_LED_BASE
	else if(strcmp(token, "gpioled") == 0)
		gpioled_cmd();
#endif
	prompt();
}

int main(void)
{
#ifdef CONFIG_CPU_HAS_INTERRUPT
	irq_setmask(0);
	irq_setie(1);
#endif
	uart_init();

	help();
	prompt();

	while(1) {
		console_service();
	}

	return 0;
}
