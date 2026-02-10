#include <irq.h>
#include "bsp.h"

#include "fpga_flash_qspi.h"

litei2c_regs I2C0_REGS = {
    .master_active_addr   = CSR_I2C0_MASTER_ACTIVE_ADDR,
    .master_addr_addr     = CSR_I2C0_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C0_MASTER_SETTINGS_ADDR,
    .master_status_addr   = CSR_I2C0_MASTER_STATUS_ADDR,
    .master_rxtx_addr     = CSR_I2C0_MASTER_RXTX_ADDR
};

litei2c_regs I2C1_REGS = {
    .master_active_addr   = CSR_I2C1_MASTER_ACTIVE_ADDR,
    .master_addr_addr     = CSR_I2C1_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C1_MASTER_SETTINGS_ADDR,
    .master_status_addr   = CSR_I2C1_MASTER_STATUS_ADDR,
    .master_rxtx_addr     = CSR_I2C1_MASTER_RXTX_ADDR
};

litei2c_regs I2C2_REGS = {
    .master_active_addr   = CSR_I2C2_MASTER_ACTIVE_ADDR,
    .master_addr_addr     = CSR_I2C2_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C2_MASTER_SETTINGS_ADDR,
    .master_status_addr   = CSR_I2C2_MASTER_STATUS_ADDR,
    .master_rxtx_addr     = CSR_I2C2_MASTER_RXTX_ADDR
};

litei2c_regs I2C3_REGS = {
    .master_active_addr   = CSR_I2C3_MASTER_ACTIVE_ADDR,
    .master_addr_addr     = CSR_I2C3_MASTER_ADDR_ADDR,
    .master_settings_addr = CSR_I2C3_MASTER_SETTINGS_ADDR,
    .master_status_addr   = CSR_I2C3_MASTER_STATUS_ADDR,
    .master_rxtx_addr     = CSR_I2C3_MASTER_RXTX_ADDR
};

//Temperature sensors
#define I2C1_TEMP_SENSOR_ADDR TMP114NB_ADDR
#define I2C2_TEMP_SENSOR_ADDR TMP114NB_ADDR
#define I2C3_TEMP_SENSOR_ADDR TMP114NB_ADDR

//IO Expanders
#define I2C2_IO_EXP0_ADDR TCA6424_ADR0
#define I2C2_IO_EXP1_ADDR TCA6424_ADR1
#define I2C3_IO_EXP0_ADDR TCA6424_ADR1
#define I2C3_IO_EXP1_ADDR TCA6424_ADR0

//VCTCXO DAC
#define I2C3_VCTCXO_DAC_ADDR DAC8050_ADDR_A0_AGND

//Cached GPIO values
#define BSP_GPIO_COUNT 18
// Static ensures this is private to bsp.c, initialized to 0 by default
static uint8_t gpio_shadow_cache[BSP_GPIO_COUNT] = {0};

#define BSP_IRQ0_BIT    (1U << CSR_BSP_EV_PENDING_EVENT0_OFFSET)
#define BSP_IRQ_SW_TRIG (1U << CSR_BSP_EV_PENDING_SW_OFFSET)

static volatile uint32_t bsp_irq_pending = 0;

static void bsp_isr(void) {
    bsp_irq_pending = bsp_ev_pending_read();
    bsp_ev_pending_write(bsp_ev_pending_read()); // Clear interrupt
    //bsp_ev_enable_write(1 << CSR_BSP_EV_STATUS_EVENT0_OFFSET); // re-enable the event handler
}

void bsp_isr_init(void) {
    printf("BSP IRQ initialization \n");

    uint32_t irq_enable_mask = 0;

    /* Clear all pending interrupts. */
    bsp_ev_pending_write(bsp_ev_pending_read());

    /* Enable irq */
    irq_enable_mask |= (1 << CSR_BSP_EV_ENABLE_EVENT0_OFFSET);
    irq_enable_mask |= (1 << CSR_BSP_EV_ENABLE_SW_OFFSET);

    bsp_ev_enable_write(irq_enable_mask);

    /* Attach isr to interrupt */
    irq_attach(BSP_INTERRUPT, bsp_isr);

    /* Enable interrupt */

    irq_setmask(irq_getmask() | (1 << BSP_INTERRUPT));
}

void bsp_process_irqs(void) {
    // If no interrupts are pending, return immediately
    if (bsp_irq_pending == 0) return;

    // Check specific bits and execute interrupt request and clear pending bit
    if (bsp_irq_pending & BSP_IRQ0_BIT) {
        bsp_init();
        bsp_irq_pending &= ~BSP_IRQ0_BIT;
    }

}


// Stub: Board-specific initialization
void bsp_init(void) {
    printf("[BSP] Begin\n");

    //NOTE: Do not change init sequence.
    //See AFE79xx Configuration Guide for Initialization flow
    bsp_powerup();

    bool status = LMK05318B_init(&I2C1_REGS);

    afe7901_init();

    if (status) {
    	printf("[BSP] Initialized\n");
    } else {
    	printf("[BSP] Fail\n");
    }

    // Set output values for all expanders to 0
    TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP0_ADDR,0,0);
    TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP0_ADDR,1,0);
    TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP0_ADDR,2,0);

    TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP1_ADDR,0,0);
    TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP1_ADDR,1,0);
    TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP1_ADDR,2,0);

    TCA6424_SetPortOutputValues(&I2C3_REGS,I2C3_IO_EXP0_ADDR,0,(REF_EN_OSC|ENABLE_5VIN|ENABLE_6VIN));
    TCA6424_SetPortOutputValues(&I2C3_REGS,I2C3_IO_EXP0_ADDR,1,PWR_LMS8_NRST);
    TCA6424_SetPortOutputValues(&I2C3_REGS,I2C3_IO_EXP0_ADDR,2,0);

    TCA6424_SetPortOutputValues(&I2C3_REGS,I2C3_IO_EXP1_ADDR,0,0);
    TCA6424_SetPortOutputValues(&I2C3_REGS,I2C3_IO_EXP1_ADDR,1,0);
    TCA6424_SetPortOutputValues(&I2C3_REGS,I2C3_IO_EXP1_ADDR,2,0);


    // Set most expander pins to be output
    TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP0_ADDR,0,0);
    TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP0_ADDR,1,0);
    TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP0_ADDR,2,0);

    TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP1_ADDR,0,0);
    TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP1_ADDR,1,0);
    TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP1_ADDR,2,0);

    TCA6424_SetPortDirection(&I2C3_REGS,I2C3_IO_EXP0_ADDR,0,0x30);
    TCA6424_SetPortDirection(&I2C3_REGS,I2C3_IO_EXP0_ADDR,1,0);
    TCA6424_SetPortDirection(&I2C3_REGS,I2C3_IO_EXP0_ADDR,2,0);

    TCA6424_SetPortDirection(&I2C3_REGS,I2C3_IO_EXP1_ADDR,0,0);
    TCA6424_SetPortDirection(&I2C3_REGS,I2C3_IO_EXP1_ADDR,1,0);
    TCA6424_SetPortDirection(&I2C3_REGS,I2C3_IO_EXP1_ADDR,2,0);

    bsp_init_adf();

    // VCTCXO Init
    DAC8050_Write_Command(&I2C3_REGS, I2C3_VCTCXO_DAC_ADDR,DAC8050_GAIN,257);

    // An arbitrary delay at the end of init seems to prevent some weird behavior
    for (int i =0; i<10000; i++) {
        __asm__ volatile ("nop");
    }


    //debug
    uint8_t value;
    uint8_t stat;

    /*
    for (int offset = 0; offset <= 13; offset++) {
        stat = bsp_gpio_read(&value, offset);

        if (stat == 0) {
            printf("GPIO offset %d value: 0x%02X\n", offset, value);
        } else {
            printf("Error reading GPIO offset %d\n", offset);
        }
    }

    uint8_t gpio_values[14] = {0xAA, 0xF0, 0x55, 0xFE,
                               0x40, 0xF0, 0x49, 0x10, 0x0A,
    							0xF1, 0x00, 0x00, 0x00, 0x00};

    for (int offset = 0; offset <= 13; offset++) {
        stat = bsp_gpio_write(gpio_values[offset], offset);

        if (stat == 0) {
            printf("Wrote 0x%02X to GPIO offset %d\n", gpio_values[offset], offset);
        } else {
            printf("Error writing to GPIO offset %d\n", offset);
        }
    }

    for (int offset = 0; offset <= 13; offset++) {
        stat = bsp_gpio_read(&value, offset);

        if (stat == 0) {
            printf("GPIO offset %d value: 0x%02X\n", offset, value);
        } else {
            printf("Error reading GPIO offset %d\n", offset);
        }
    }
    */


}

// Stub: Board power-up
void bsp_powerup(void) {
	uint32_t value =0;
	uint32_t rd_val =0;
	const int timeout_limit = 10000000;   // Timeout threshold (number of attempts)
	int timeout_counter = 0;

    printf("[BSP] Power up\n");


    //Start with know state (Everything is powered down)
    pwr_control_reg00_write(value);

    //-------------------------------------------------------------------------
    // Enable power CLK_2P05 rail. It is needed for LMK and AFE
    //-------------------------------------------------------------------------
    value = value | PWR_EN_2P05_BIT;
    pwr_control_reg00_write(value);


    //-------------------------------------------------------------------------
    // Enable power for AFE7901
    //-------------------------------------------------------------------------
    bsp_afe7901_pwrup_seq();

    //-------------------------------------------------------------------------
    // Enable power for LMK7901ABJ
    //-------------------------------------------------------------------------
    bsp_lmk0518_pwrup_seq();

    printf("[BSP] Power up done\n");

}


void bsp_lmk0518_pwrup_seq(void) {
	uint32_t value =0;
	uint32_t rd_val =0;
	// TPS62088 PG rising edge has a 100-μs blanking time, CPU is typically
	// running on 100MHz. We need at least 10000 threshold, 20000 to be safe
	const int timeout_limit = 20000;   // Timeout threshold (number of attempts)
	int timeout_counter = 0;

    // Read current register state
	value = pwr_control_reg00_read();

    // CLK_2P05 rail has to be powered first
    printf("value = 0x%08X\n", value);
    value = value | PWR_EN_2P05_BIT;
    printf("value = 0x%08X\n", value);
    pwr_control_reg00_write(value);


    // Wait for PG_EN_2P05_BIT to become 1, with timeout
    while ((pwr_control_reg01_read() & PG_EN_2P05_BIT) == 0) {
        cdelay(1);  // Small delay to avoid hammering the register
        timeout_counter++;
        if (timeout_counter >= timeout_limit) {
            printf("[BSP] ERROR: Timeout waiting for PG_EN_2P05_BIT\n");
            break;
        }
    }

    // Set PWR_EN_LMK_BIT to power up LMK
    if ((pwr_control_reg01_read() & PG_EN_2P05_BIT) != 0) {
        printf("value = 0x%08X\n", value);
        value = value | PWR_EN_2P05_BIT | PWR_EN_LMK_BIT;
        printf("value = 0x%08X\n", value);
        pwr_control_reg00_write(value);
        cdelay(50000000);  // Delay to allow TPSA3501 5ms startup delay
        printf("[BSP] LMK power-up sequence done\n");
    }

    cdelay(10000);
}

void bsp_afe7901_pwrup_seq(void) {
	uint32_t value =0;
	uint32_t rd_val =0;
	uint8_t i2c_val =0;
	// TPS62088 has PG rising edge has a 100-μs blanking time, CPU is typically
	// running on 100MHz. We need at least 10000 threshold, 20000 to be safe
	int timeout_limit = 20000;   // Timeout threshold (number of attempts)
	int timeout_counter = 0;

	//Hold AFE7901 in known reset state
	afe_reg00_write(0x0);

    // Read current register state
    value = pwr_control_reg00_read();

	//-------------------------------------------------------------------------
    // Power up PMIC_VIOSYS rail
	//-------------------------------------------------------------------------
    printf("value = 0x%08X\n", value);
    value = value | PAFE_EN_D1P0_BIT;
    printf("value = 0x%08X\n", value);
    pwr_control_reg00_write(value);
    // 200us delay to wait for LP590718Q start-up (datasheet - 150us max tON), there is no PG to check
    cdelay(20000);


    //-------------------------------------------------------------------------
    // Initialize LP8754 PMIC
    //-------------------------------------------------------------------------

    // From LP8754 datasheet:
    /*
    VVIOSYS set high. Enables the system I/O interface. For power-on-reset (POR), the I2C host should allow at
    least 500 μs before sending data to the LP8754 after the rising edge of the VIOSYS line.
    */
    cdelay(55000);
    bool status = LP8754_init(&I2C0_REGS);
    //Set NRST=1 to start LP8754
    value = value | AFE_DCDC_1P0_NRST_BIT;
    pwr_control_reg00_write(value);
    printf("After LP8754_init: value = 0x%08X\n", value);
    // 7ms delay to wait for LP8754 PG masking (400us+6.4msdatasheet spec)
    cdelay(700000);

	timeout_limit = 20000;   // Timeout threshold (number of attempts)
	timeout_counter = 0;

    while (true) {
        if (!LP8754_read_reg(&I2C0_REGS,REG_FLAGS_0, &i2c_val)) {
            printf("[BSP] ERROR: Failed to read LP8754 register\n");
            //exit early, we cannot continue without LP8754
            return;
        }

        if ((i2c_val & FLAGS_0_nPG_B0_BIT) == 0x00) {
        	printf("[BSP] Info: LP8754 nPG_B0_BIT=0 \n");
            break;
        }

        cdelay(1);  // Small delay to avoid hammering the register
        timeout_counter++;
        if (timeout_counter >= timeout_limit) {
            printf("[BSP] ERROR: Timeout waiting for LP8754  nPG_B0_BIT=0\n");
            //exit early, we cannot continue without LP8754
            return;
        }
    }


    //-------------------------------------------------------------------------
    // Enable A1P2
    //-------------------------------------------------------------------------
    value = value | PAFE_EN_A1P2_BIT;
    printf("value = 0x%08X\n", value);
    pwr_control_reg00_write(value);

    timeout_limit = 20000;   // Timeout threshold (number of attempts)
    timeout_counter = 0;

    // Wait for PG_AFE_AVDD_1P2 bit to become 1, with timeout
    while ((pwr_control_reg01_read() & PG_AFE_AVDD_1P2_BIT) == 0) {
        cdelay(1);  // Small delay to avoid hammering the register
        timeout_counter++;
        if (timeout_counter >= timeout_limit) {
            printf("[BSP] ERROR: Timeout waiting for PG_AFE_AVDD_1P2\n");
            break;
        }
    }

    //-------------------------------------------------------------------------
    // Enable AFE_AVDD_1P8 and AFE_VDDCLK_1P8
    //-------------------------------------------------------------------------
    // Ensure that AFE_VD12 is rail is powered up before powering AFE_AVDD_1P8 and
    // AFE_VDDCLK_1P8 rails. See afe7901 datasheet.
    // CLK_2P5 rail is needed for AFE_VDDCLK_1P8
    if ( (pwr_control_reg01_read() & (PG_AFE_AVDD_1P2_BIT | PG_EN_2P05_BIT)) == (PG_AFE_AVDD_1P2_BIT | PG_EN_2P05_BIT) ) {
        printf("value = 0x%08X\n", value);
        value = value | PAFE_EN_A1P8_BIT | PAFE_EN_A1P8_1_BIT;
        printf("value = 0x%08X\n", value);
        pwr_control_reg00_write(value);
        //Delays from datasheet Figure 9-7. Power Supply Sequence Requirements
        cdelay(550000);	// Delay to allow TPSA3501 5ms startup delay
        cdelay(15000);	// 100us+ delay for clk to toggle
        //afe_reg00_write(0x1); // deaser resetz
        //cdelay(10000);  // 100Us delay for reset to stabilize
        printf("[BSP] AFE7901 power-up sequence done\n");
    } else {
    	printf("[BSP] AFE7901 power-up fail, AFE_VD12 or CLK_2P5 rails are not powered\n");
    }

}


// Stub: Board shutdown or cleanup (optional)
void bsp_shutdown(void) {
    // Power down peripherals, save state, etc.
    printf("[BSP] boardname_bsp shutting down\n");
}

// Stub: Delay function (could use timer or busy-wait)
void bsp_delay_ms(unsigned int ms) {
    // Implement platform-specific delay
    // Example: busy wait or use a hardware timer
    while (ms--) {
        // rough CPU delay loop (not accurate)
        for (volatile int i = 0; i < 1000; i++);
    }
}

void lms7002m_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
    // No LMS7's on HiPer board and nothing to return
    }

uint16_t lms7002m_spi_read(uint16_t addr, uint8_t periph_id) {
    // No LMS7's on HiPer board
    return -1;
}

void lms8001_spi_write(uint16_t addr, uint16_t val, uint8_t periph_id) {
    lms_spi_write(addr, val, periph_id);
}

uint16_t lms8001_spi_read(uint16_t addr, uint8_t periph_id) {
    return lms_spi_read(addr, periph_id);
}
/** Checks if peripheral ID is valid.
 Returns 1 if valid, else 0.
 Returns 2 if no LMS7's are present at all*/
int8_t lms7002m_periph_id_check(uint8_t periph_id) {
    // No LMS7's on HiPer board
    return 2; // No LMS7's
}

/** Checks if peripheral ID is valid.
 Returns 1 if valid, else 0.
 Returns 2 if no LMS8's are present at all*/
int8_t lms8001_periph_id_check(uint8_t periph_id) {
    if (periph_id > MAX_ID_LMS8) {
        return 0; // Invalid ID
    }
    return 1; // Valid ID
}



void afe7901_bringup_pre(void) {
	uint32_t res;
	//Configuring JESD IP CORE
	afe_rx_cfg0_write(0x0);  // tiafe_cfg_rx_lane_enabled=0x0, tiafe_cfg_rx_lane_polarity =0x0;
	afe_tx_cfg0_write(0x0);  // tiafe_cfg_tx_lane_enabled=0x0, tiafe_cfg_tx_lane_polarity =0x0;
	afe_rx_ctrl_write(0x1); //tiafe_rx_sync_reset
	afe_tx_ctrl_write(0x1); //tiafe_tx_sync_reset


	res = afe_core_ctrl_read();
	afe_core_ctrl_write(res & ~(1U << CSR_AFE_CORE_CTRL_AFE_CORE_RST_N_OFFSET));  //afe_core_rst_n = 0;

	cdelay(10000);

	afe_rx_cfg2_write(0x00); // 0 means autodetect and adjust

	afe_rx_cfg1_write(0x3210); //tiafe_cfg_rx_lane_map
	afe_tx_cfg1_write(0x3210); //tiafe_cfg_tx_lane_map

	afe_rx_cfg0_write(0xF);  // tiafe_cfg_rx_lane_enabled=0xf, tiafe_cfg_rx_lane_polarity =0x0;
	afe_tx_cfg0_write(0xF);  // tiafe_cfg_tx_lane_enabled=0xf, tiafe_cfg_tx_lane_polarity =0x0;

	cdelay(1000);

	printf("[BSP] AFE Core reset_n = 1 \n");
	res = afe_core_ctrl_read();
	afe_core_ctrl_write(res | (1U << CSR_AFE_CORE_CTRL_AFE_CORE_RST_N_OFFSET));   //afe_core_rst_n = 0x1;


    for (unsigned k = 0; k < 100; k++) {
    	cdelay(250000);

        res = afe_rx_status0_read();
        //xil_printf("[BSP] AFE RX SYSREF_RAC : 0x%x\n",res);
        res = afe_tx_status0_read();
        //xil_printf("[BSP] AFE TX SYSREF_RAC : 0x%x\n",res);
        //xil_printf("[BSP] AFE TX SYSREF_RAC : 0x%x\n",res);
        if (res & 0x00000008) {
        	printf("[BSP] AFE PLL ready \n");
            break;
        }
    }

    afe_tx_ctrl_write(0x00); //tiafe_tx_sync_reset
	cdelay(1000000); //wait just to be safe
    res = afe_tx_status0_read();
    xil_printf("[BSP] AFE TX SYSREF_RAC : 0x%x\n",res);


}

void afe7901_bringup_post(void) {
	uint32_t res;
	afe_rx_ctrl_write(0x00); //tiafe_rx_sync_reset

	cdelay(1000000); //wait just to be safe
    res = afe_rx_status0_read();
    xil_printf("[BSP] AFE RX SYSREF_RAC : 0x%x\n",res);
    res = afe_tx_status0_read();
    xil_printf("[BSP] AFE TX SYSREF_RAC : 0x%x\n",res);

}

/** AFE7901 initialization
 *
 */
void afe7901_init(void) {
	printf("[BSP] AFE7901 init\n");


	cdelay(10000000); //wait just to be safe
	// Assuming clocks are stable
	//Hardware reset deasert
	afe_reg00_write(0x0); // AFE is in reset FPGA
	cdelay(10000000);  // 100Us delay for reset to stabilize
    afe_reg00_write(0x1); // deaser resetz
    cdelay(10000000);  // 100Us delay for reset to stabilize
    afe7901_bringup_pre();
	//AFE7901_bringup();
	printf("[BSP] Waiting AFE7901 init\n");
	//cdelay(300000000);
	//afe7901_bringup_post();
	//LMK05318B_write_reg(OUTCTL_2, 0xD0);

	//AFE7901_resync();

	printf("[BSP] AFE7901 init done\n");

}

uint8_t bsp_analog_read(const uint8_t channel, uint8_t *unit, uint8_t* value_msb, uint8_t* value_lsb) {
    uint16_t buf;
    uint8_t* buf_point = (uint8_t*)&buf;
    switch (channel) {
        // Assume Continuous conversion is enabled for all temperature sensors
        case 0:
            *unit = 0x50; // unit = 0.1C
            buf = TMP114_Read_Temp(&I2C1_REGS, I2C1_TEMP_SENSOR_ADDR);
            buf = TMP114_Convert_Temp(buf);
            *value_lsb = buf_point[0];
            *value_msb = buf_point[1];
            break;
        case 1:
            *unit = 0x50; // unit = 0.1C
            buf = TMP114_Read_Temp(&I2C2_REGS, I2C2_TEMP_SENSOR_ADDR);
            buf = TMP114_Convert_Temp(buf);
            *value_lsb = buf_point[0];
            *value_msb = buf_point[1];
            break;
        case 2:
            *unit = 0x50; // unit = 0.1C
            buf = TMP114_Read_Temp(&I2C3_REGS, I2C3_TEMP_SENSOR_ADDR);
            buf = TMP114_Convert_Temp(buf);
            *value_lsb = buf_point[0];
            *value_msb = buf_point[1];
            break;
        case 3:
            *unit = 0x00; // RAW unit
            DAC8050_Read_Value(&I2C3_REGS, I2C3_VCTCXO_DAC_ADDR, &buf);
            *value_lsb = buf_point[0];
            *value_msb = buf_point[1];
            break;
        default:
            // Error
            return 1;
    }
    return 0;
}

uint8_t bsp_analog_write(const uint8_t channel, const uint8_t unit, const uint8_t value_msb, const uint8_t value_lsb) {
    uint16_t buf;
    uint8_t* buf_point = (uint8_t*)&buf;
    switch (channel) {
        // No write options for temp sensors (case 0,1,2)
        case 3:
            if (unit == 0x00) {
                buf_point[0] = value_lsb;
                buf_point[1] = value_msb;
                DAC8050_Write_Value(&I2C3_REGS, I2C3_VCTCXO_DAC_ADDR, buf);
                break;
            }
            else {
                // Wrong unit, return error
                return 1;
            }
        // No devices to write to are defined. Auto fail.
        default:
            // Error
            return 1;
    }
    return 0;
}

uint8_t bsp_gpio_dir_read(uint8_t* data, const uint8_t offset) {
    uint8_t buf;
    switch (offset) {
        case 0:
        case 1:
        case 2:
            // Note: GPIO dir write/read command uses 0 for input, TCA6424 uses 1 for input,
            //       so we must do bitwise inversion
            TCA6424_ReadPortDirection(&I2C2_REGS,I2C2_IO_EXP0_ADDR,offset,&buf);
            *data = ~buf;
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U114_ADDR,buf,offset);
            break;
        case 3:
        case 4:
        case 5:
            TCA6424_ReadPortDirection(&I2C2_REGS,I2C2_IO_EXP1_ADDR,offset-3,&buf);
            *data = ~buf;
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U113_ADDR,buf,offset-3);
            break;
        case 6:
        case 7:
        case 8:
            TCA6424_ReadPortDirection(&I2C3_REGS, I2C3_IO_EXP0_ADDR, offset-6, &buf);
            *data = ~buf;
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U115_ADDR,buf,offset-6);
            break;
        case 9:
        case 10:
        case 11:
            TCA6424_ReadPortDirection(&I2C3_REGS, I2C3_IO_EXP1_ADDR, offset-9, &buf);
            *data = ~buf;
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U110_ADDR,buf,offset-9);
            break;
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
            *data = 0xFF; // These GPIOs are output only
            break;
        default:
            return 1;
    }

    return 0;
}

uint8_t bsp_gpio_dir_write(const uint8_t data, const uint8_t offset) {
    switch (offset) {
        case 0:
        case 1:
        case 2:
            // Note: GPIO dir write/read command uses 0 for input, TCA6424 uses 1 for input,
            //       so we must do bitwise inversion
            TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP0_ADDR,offset,~data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U114_ADDR,~data,offset);
            break;
        case 3:
        case 4:
        case 5:
            TCA6424_SetPortDirection(&I2C2_REGS,I2C2_IO_EXP1_ADDR,offset-3,~data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U113_ADDR,~data,offset-3);
            break;
        case 6:
        case 7:
        case 8:
            TCA6424_SetPortDirection(&I2C3_REGS, I2C3_IO_EXP0_ADDR, offset-6, ~data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U115_ADDR,~data,offset-6);
            break;
        case 9:
        case 10:
        case 11:
            TCA6424_SetPortDirection(&I2C3_REGS, I2C3_IO_EXP1_ADDR, offset-9, ~data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_DIRECTION_U110_ADDR,~data,offset-9);
            break;
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
            return 1; // These GPIO's don't have writable directions for now
            break;
        default:
            return 1;
    }
    return 0;
}

uint8_t bsp_gpio_read(uint8_t* data, const uint8_t offset) {
    switch (offset) {
        case 0:
        case 1:
        case 2:
            TCA6424_ReadPortInputValues(&I2C2_REGS,I2C2_IO_EXP0_ADDR,offset, data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_IN_VALUE_114_ADDR,data[0],offset);
            break;
        case 3:
        case 4:
        case 5:
            TCA6424_ReadPortInputValues(&I2C2_REGS,I2C2_IO_EXP1_ADDR,offset-3, data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_IN_VALUE_113_ADDR,data[0],offset-3);
            break;
        case 6:
        case 7:
        case 8:
            TCA6424_ReadPortInputValues(&I2C3_REGS, I2C3_IO_EXP0_ADDR, offset-6, data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_IN_VALUE_115_ADDR,data[0],offset-6);
            break;
        case 9:
        case 10:
        case 11:
            TCA6424_ReadPortInputValues(&I2C3_REGS, I2C3_IO_EXP1_ADDR, offset-9, data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_IN_VALUE_110_ADDR,data[0],offset-9);
            break;
        case 12:
        case 13:
        case 14:
        case 15:
            helper_csr_read_byte(CSR_GPIO_CONTROL_GPIO_ADDR,data,offset-12);
            break;
        case 16:
        case 17:
            helper_csr_read_byte(CSR_GPIO_CONTROL_GPIO2_ADDR,data, offset-16);
            break;
        default:
            return 1;
    }
    return 0;
}

uint8_t bsp_gpio_get_cached(const uint8_t offset) {
    uint8_t data = 0; // Buffer to store the read value
    switch (offset) {
        case 0:
        case 1:
        case 2:
            // Corresponds to CSR_GPIO_CONTROL_PORT_OUT_VALUE_114_ADDR
            helper_csr_read_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_114_ADDR, &data, offset);
            break;
        case 3:
        case 4:
        case 5:
            // Corresponds to CSR_GPIO_CONTROL_PORT_OUT_VALUE_113_ADDR
            helper_csr_read_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_113_ADDR, &data, offset - 3);
            break;
        case 6:
        case 7:
        case 8:
            // Corresponds to CSR_GPIO_CONTROL_PORT_OUT_VALUE_115_ADDR
            helper_csr_read_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_115_ADDR, &data, offset - 6);
            break;
        case 9:
        case 10:
        case 11:
            // Corresponds to CSR_GPIO_CONTROL_PORT_OUT_VALUE_110_ADDR
            helper_csr_read_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_110_ADDR, &data, offset - 9);
            break;
        case 12:
        case 13:
        case 14:
        case 15:
            // Corresponds to CSR_GPIO_CONTROL_GPIO_ADDR
            helper_csr_read_byte(CSR_GPIO_CONTROL_GPIO_ADDR, &data, offset - 12);
            break;
        case 16:
        case 17:
            // Corresponds to CSR_GPIO_CONTROL_GPIO2_ADDR
            helper_csr_read_byte(CSR_GPIO_CONTROL_GPIO2_ADDR, &data, offset - 16);
            break;
        default:
            return 0; // Or handle error
    }

    return data;
}

uint8_t bsp_gpio_write(const uint8_t data, const uint8_t offset) {

    if (offset >= BSP_GPIO_COUNT) {
        return 1; // Safety check
    }

    // Update shadow register
    gpio_shadow_cache[offset] = data;

    switch (offset) {
        case 0:
        case 1:
        case 2:
            TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP0_ADDR,offset,data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_114_ADDR,data,offset);
            break;
        case 3:
        case 4:
        case 5:
            TCA6424_SetPortOutputValues(&I2C2_REGS,I2C2_IO_EXP1_ADDR,offset-3,data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_113_ADDR,data,offset-3);
            break;
        case 6:
        case 7:
        case 8:
            TCA6424_SetPortOutputValues(&I2C3_REGS, I2C3_IO_EXP0_ADDR, offset-6, data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_115_ADDR,data,offset-6);
            break;
        case 9:
        case 10:
        case 11:
            TCA6424_SetPortOutputValues(&I2C3_REGS, I2C3_IO_EXP1_ADDR, offset-9, data);
            helper_csr_update_byte(CSR_GPIO_CONTROL_PORT_OUT_VALUE_110_ADDR,data,offset-9);
            break;
        case 12:
        case 13:
        case 14:
        case 15:
            helper_csr_update_byte(CSR_GPIO_CONTROL_GPIO_ADDR,data,offset-12);
            break;
        case 16:
        case 17:
            helper_csr_update_byte(CSR_GPIO_CONTROL_GPIO2_ADDR,data, offset-16);
            break;
        default:
            return 1;
    }
    return 0;
}

/**
 * @brief Transfers data over SPI using the selected SPI master and chip select.
 *
 * This function handles SPI transactions for LiteX SPIMaster cores.
 *
 * @param master        SPI master index
 * @param cs            Chip select line index (converted to bitmask internally).
 * @param mosidata      Pointer to the transmit buffer (MSB-first).
 * @param transfer_len  Total number of bytes to toggle on the SPI bus (1-4).
 * @param recv_data_len Number of bytes to extract from the received MISO data (0-4),
 *                      counting from the end of the transfer (i.e. if recv_data_len = 1,
 *                      the last byte of the MISO data is extracted).
 * @param misodata      Pointer to receive buffer (can be NULL if response is ignored).
 *                      Buffer must be at least 'data_len' bytes.
 *
 * @return 0 on success, 1 on error (invalid master or length).
 */
uint8_t bsp_spi_transfer(uint8_t master, uint8_t cs, uint8_t *mosidata, uint8_t transfer_len, uint8_t recv_data_len, uint8_t *misodata) {

    uint32_t recv_val = 0;
    uint32_t bits = transfer_len * 8;
    uint32_t cs_mask = 1 << cs;

    if (transfer_len == 0 || transfer_len > 4) return 1;

    // Pack mosidata MSB-first into a 32-bit register
    uint32_t packed_mosi = 0;
    for (uint32_t i = 0; i < transfer_len; i++) {
        packed_mosi = (packed_mosi << 8) | mosidata[i];
    }

    // LiteX SPIMaster in 'raw' mode (default) shifts out from the MSB of its data_width.
    // We must left-align our data to the core's width.
    switch (master) {
        case 0: // 32-bit data_width
            packed_mosi <<= (4 - transfer_len) * 8;
            spimaster_cs_write(cs_mask);
            cdelay(1);
            while ((spimaster_status_read() & 0x1) == 0) {}
            spimaster_mosi_write(packed_mosi);
            spimaster_control_write(bits * SPI_LENGTH | SPI_START);
            while ((spimaster_status_read() & 0x1) == 0) {}
            recv_val = spimaster_miso_read();
            break;

        case 1: // 24-bit data_width
            packed_mosi <<= (3 - transfer_len) * 8;
            spimaster1_cs_write(cs_mask);
            cdelay(1);
            while ((spimaster1_status_read() & 0x1) == 0) {}
            spimaster1_mosi_write(packed_mosi);
            spimaster1_control_write(bits * SPI_LENGTH | SPI_START);
            while ((spimaster1_status_read() & 0x1) == 0) {}
            recv_val = spimaster1_miso_read();
            break;

        case 2: // 24-bit data_width
            packed_mosi <<= (3 - transfer_len) * 8;
            spimaster_adf_cs_write(cs_mask);
            cdelay(1);
            while ((spimaster_adf_status_read() & 0x1) == 0) {}
            spimaster_adf_mosi_write(packed_mosi);
            spimaster_adf_control_write(bits * SPI_LENGTH | SPI_START);
            while ((spimaster_adf_status_read() & 0x1) == 0) {}
            recv_val = spimaster_adf_miso_read();
            break;

        default:
            return 1;
    }

    // LiteX SPIMaster captures MISO into the LSBs of the register.
    // If we want 'recv_data_len' bytes, they are in recv_val[recv_data_len*8-1:0].
    if (misodata && recv_data_len > 0) {
        for (int i = recv_data_len - 1; i >= 0; i--) {
            misodata[i] = recv_val & 0xFF;
            recv_val >>= 8;
        }
    }

    return 0;
}

void bsp_lms8_pwrup(void) {
	LP8758_init(&I2C2_REGS);
}


// Helper function just to pack the integer into the buffer
static void adf_pack_u32(uint32_t value, uint8_t *buf) {
    buf[0] = (value >> 16) & 0xFF; // Upper byte
    buf[1] = (value >> 8) & 0xFF;  // Middle byte
    buf[2] = value & 0xFF;         // Lower byte
}

 void bsp_control_adf(uint8_t oe, const uint8_t data[3])
{
    Control_TCXO_ADF(ADF4002_SPIMASTER, ADF4002_CS, oe, (uint8_t*)data);
}

void bsp_init_adf(void) {
    uint32_t data;
    uint8_t tx_buf[3];

    // R-counter to 5
    data = (1 << 20) | (5 << 2);
    data |= 0; // 00 control bits - R counter
    adf_pack_u32(data, tx_buf); // 1. Pack
    bsp_control_adf(1, tx_buf); // 2. Send

    // N-counter to 8
    data = (0 << 21) | (8 << 8);
    data |= 1; // 01 control bits - N counter
    adf_pack_u32(data, tx_buf); // 1. Pack
    bsp_control_adf(1, tx_buf); // 2. Send

    // Function latch
    data = (3 << 18) | (3 << 15) | (8 << 11) | (1 << 7) | (1 << 4);
    data |= 2; // 10 control bits - Function latch
    adf_pack_u32(data, tx_buf); // 1. Pack
    bsp_control_adf(1, tx_buf); // 2. Send

    // Init latch
    data = (3 << 18) | (3 << 15) | (8 << 11) | (1 << 7) | (1 << 4);
    data |= 3; // 11 control bits - Init latch
    adf_pack_u32(data, tx_buf); // 1. Pack
    bsp_control_adf(1, tx_buf); // 2. Send
}

void bsp_vctcxo_permanent_dac_read(uint8_t *data) {
    FlashQspi_CMD_ReadDataByte(mem_write_offset, &data[0]);
    FlashQspi_CMD_ReadDataByte(mem_write_offset + 1, &data[1]);
}

void bsp_vctcxo_permanent_dac_write(uint8_t *data) {
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_SectorErase(mem_write_offset);
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_PageProgramByte(mem_write_offset, &data[0]);
    FlashQspi_CMD_WREN();
    FlashQspi_CMD_PageProgramByte(mem_write_offset + 1, &data[1]);
}

uint8_t bsp_mem_read(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count) {
    // Check if the operation is going to be performed on EEPROM #1 and
    // that it's specifically being used to read VCTCXO DAC value
    // NOTE: condition for IF is copied from previous implementation, might need review
    if (data_count == 2 && target == 3 && progmode == 0 && offset == 16) {
        bsp_vctcxo_permanent_dac_read(data);
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}

uint8_t bsp_mem_write(uint32_t offset, uint8_t progmode, uint16_t target, uint8_t *data, uint8_t data_count) {
    // Check if the operation is going to be performed on EEPROM #1 and
    // that it's specifically being used to store VCTCXO DAC value
    // NOTE: condition for IF is copied from previous implementation, might need review
    if (data_count == 2 && target == 3 && progmode == 0 && offset == 16) {
        bsp_vctcxo_permanent_dac_write(data);
        return STATUS_COMPLETED_CMD;
    }
    return STATUS_ERROR_CMD;
}