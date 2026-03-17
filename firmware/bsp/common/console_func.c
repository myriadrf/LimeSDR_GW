//
// Created by ts on 5/14/25.
//

#include "console_func.h"

#include <libbase/console.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <generated/csr.h>

#include "bsp.h"
/*-----------------------------------------------------------------------*/
/* Uart                                                                  */
/*-----------------------------------------------------------------------*/

  char *readstr(void) {
    char c[2];
    static char s[64];
    static int ptr = 0;

    if (readchar_nonblock()) {
        c[0] = getchar();
        c[1] = 0;
        switch (c[0]) {
            case 0x7f:
            case 0x08:
                if (ptr > 0) {
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
                if (ptr >= (sizeof(s) - 1))
                    break;
                fputs(c, stdout);
                s[ptr] = c[0];
                ptr++;
                break;
        }
    }

    return NULL;
}

  char *get_token(char **str) {
    char *c, *d;

    c = (char *) strchr(*str, ' ');
    if (c == NULL) {
        d = *str;
        *str = *str + strlen(*str);
        return d;
    }
    *c = 0;
    d = *str;
    *str = c + 1;
    return d;
}

  void prompt(void) {
    printf("\e[92;1mlimesdr\e[0m> ");
}

/*-----------------------------------------------------------------------*/
/* Help                                                                  */
/*-----------------------------------------------------------------------*/

  void help(void) {
    puts("\nLimeSDR minimal app built "__DATE__" "__TIME__"\n");
    puts("Available commands:");
    puts("help               - Show this command");
    puts("reboot             - Reboot CPU");
    puts("i2c_test           - Test I2C Buses");
    puts("init_pmic          - Initialize DC-DC switching regulators");
    puts("dump_pmic          - Dump DC-DC switching regulator configuration");
#ifdef CSR_LEDS_BASE
    puts("led                - Led demo");
#endif
#ifdef CSR_LIME_TOP_BASE
    puts("gpioled            - GPIO override demo");
    puts("mmap               - MMAP demo");
#endif
#ifdef CSR_LIME_TOP_BASE
    puts("lms                - lms7002_top module status");
#endif
    puts("dac_test           - Test DAC");
    puts("flash_test         - Test FPGA FLASH");
    puts("lmk_stat           - Get status of LMK05318B");
}

/*-----------------------------------------------------------------------*/
/* Commands                                                              */
/*-----------------------------------------------------------------------*/

  void reboot_cmd(void) {
    ctrl_reset_write(1);
}

#ifdef CSR_LEDS_BASE

  void led_cmd(void) {
    int i;
    printf("Led demo...\n");

    printf("Counter mode...\n");
    for (i = 0; i < 32; i++) {
        leds_out_write(i);
        busy_wait(100);
    }

    printf("Shift mode...\n");
    for (i = 0; i < 4; i++) {
        leds_out_write(1 << i);
        busy_wait(200);
    }
    for (i = 0; i < 4; i++) {
        leds_out_write(1 << (3 - i));
        busy_wait(200);
    }

    printf("Dance mode...\n");
    for (i = 0; i < 4; i++) {
        leds_out_write(0x55);
        busy_wait(200);
        leds_out_write(0xaa);
        busy_wait(200);
    }
}

#endif



  void gpioled_cmd(void) {
#ifdef CSR_GPIO_BASE
    int i, j;
    printf("GPIO Led override demo...\n");

      // Commented out code and added an error
      // TODO: Implement in the future maybe
    printf("ERROR: NOT IMPLEMENTED");

    //
    // lime_top_gpio_gpio_override_write(0x7);
    // lime_top_gpio_gpio_override_dir_write(0x0);
    // lime_top_gpio_gpio_override_val_write(0x0);
    // for (i = 0; i < 3; i++) {
    //     for (j = 0; j < 8; j++) {
    //         lime_top_gpio_gpio_override_val_write(0x0);
    //         busy_wait(100);
    //         lime_top_gpio_gpio_override_val_write(1 << i);
    //         busy_wait(100);
    //     }
    // }
    //
    // lime_top_gpio_gpio_override_val_write(0x0);
    // printf("GPIO Led override demo...ended\n");
#endif
}

  void mmap_cmd(void) {
#ifdef LIME_TOP_MMAP_BASE
    int i;

    volatile uint32_t *mmap_ptr = (volatile uint32_t *) LIME_TOP_MMAP_BASE;
    printf("MMAP demo...\n");

    /* Write counter values to SRAM. */

    printf("Writing values to SRAM at address 0x%08lX...\n", LIME_TOP_MMAP_BASE);
    for (i = 0; i < 8; i++) {
        mmap_ptr[i] = i;
        printf("Wrote %d to address 0x%08X\n", i, (unsigned int) (LIME_TOP_MMAP_BASE + i * sizeof(uint32_t)));
    }

    /* Read back values from SRAM. */
    printf("Reading values from SRAM at address 0x%08lX...\n", LIME_TOP_MMAP_BASE);
    for (i = 0; i < 8; i++) {
        uint32_t value = mmap_ptr[i];
        printf("Read %ld from address 0x%08X\n", value, (unsigned int) (LIME_TOP_MMAP_BASE + i * sizeof(uint32_t)));
    }
    // *mmap_ptr = (volatile uint32_t *)CSR_BASE;
    // printf("Reading values from SRAM at address 0x%08lX...\n", CSR_BASE);
    // for (i = 0; i < 8; i++) {
    // 	uint32_t value = mmap_ptr[i];
    // 	printf("Read 0x%08X from address 0x%08X\n", value, (unsigned int)(CSR_BASE + i * sizeof(uint32_t)));
    // }
    uint32_t dest;
    printf(" CNTRL: ");
    for (int cnt = 0; cnt < 16; cnt++) {
        dest = csr_read_simple((CSR_CNTRL_CNTRL_ADDR + cnt * 4));
        printf("%x ", dest);
    }
    printf(" \n");

    printf(" CNTRL wr: \n");
    csr_write_simple(CSR_CNTRL_CNTRL_ADDR, 5);
#endif
}



/*-----------------------------------------------------------------------*/
/* I2C                                                                   */
/*-----------------------------------------------------------------------*/
  void i2c_test(void) {
      // TODO: Leaving this empty for now, should probably fix later
    // printf("I2C0 Scan...\n");
    // i2c0_scan();
    //
    // printf("\n");
    //
    // printf("I2C1 Scan...\n");
    // i2c1_scan();
    //
    // printf("I2C2 Scan...\n");
    // i2c2_scan();
}

/*-----------------------------------------------------------------------*/
/* LMS7002m status                                                       */
/*-----------------------------------------------------------------------*/


  void lms7002_status(void)
{
#ifdef CSR_LIME_TOP_LMS7002
	printf("TX PLL Lock status = %x \n", csr_read_simple(pll0_tx_addrs.locked));
	printf("\n");
	printf("RX PLL Lock status = %x \n", csr_read_simple(pll1_rx_addrs.locked));
#endif
}

  /*-----------------------------------------------------------------------*/
  /* LMK05318B status                                                       */
  /*-----------------------------------------------------------------------*/
 void lmk_status(void)
{
   bsp_lmk_check_lock();
}

/*-----------------------------------------------------------------------*/
/* Console service / Main                                                */
/*-----------------------------------------------------------------------*/

  void console_service(void) {
    char *str;
    char *token;

    str = readstr();
    if (str == NULL) return;
    token = get_token(&str);
    if (strcmp(token, "help") == 0)
        help();
    else if (strcmp(token, "reboot") == 0)
        reboot_cmd();
    else if (strcmp(token, "i2c_test") == 0)
        i2c_test();
    //else if (strcmp(token, "init_pmic") == 0)
        //init_pmic();
    //else if (strcmp(token, "dump_pmic") == 0)
        //dump_pmic();
#ifdef CSR_LEDS_BASE
    else if (strcmp(token, "led") == 0)
        led_cmd();
#endif
#ifdef CSR_LIME_TOP_BASE
    else if (strcmp(token, "gpioled") == 0)
        gpioled_cmd();
    else if (strcmp(token, "mmap") == 0)
        mmap_cmd();
#endif
#ifdef CSR_LIME_TOP_BASE
    else if (strcmp(token, "lms") == 0)
        lms7002_status();
#endif
    else if (strcmp(token, "lmk_stat") == 0)
        lmk_status();
    //else if (strcmp(token, "dac_test") == 0)
        //dac_test();
    //else if (strcmp(token, "flash_test") == 0)
        //flash_test();

    prompt();
}
