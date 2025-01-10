//
// Created by ts on 8/7/24.
//

#include "Xil_clk_drp.h"
#include "stdio.h"

uint16_t Read_MMCM_DRP(PLL_ADDRS *addresses, uint16_t Addr) {
    uint16_t retval;
    uint16_t timeout = 0;
    uint16_t timeout_limit = 2000;
    csr_write_simple(Addr, addresses->adr);
    // Reset drdy latch just in case
    Reset_Drdy_latch(addresses->drdy_reset);
    // Litex only needs read csr to be written to, the value doesn't matter
    csr_write_simple(1, addresses->read);
    // Wait for the read to complete
    while (csr_read_simple(addresses->drdy) != 1 && timeout < timeout_limit) {
        busy_wait_us(1);
        timeout++;
    }
    Reset_Drdy_latch(addresses->drdy_reset);
    // Retrieve and return the DRP register value
    retval = csr_read_simple(addresses->dat_r);
    if (timeout > (timeout_limit - 1)) {
        printf("DRP READ TIMED OUT! \n");
    }
    return retval;
}

void Write_MMCM_DRP(PLL_ADDRS *addresses, uint16_t Addr, uint16_t Val) {
    uint16_t timeout = 0;
    uint16_t timeout_limit = 2000;
    csr_write_simple(Addr, addresses->adr);
    csr_write_simple(Val, addresses->dat_w);
    // Reset drdy latch just in case
    Reset_Drdy_latch(addresses->drdy_reset);
    // Litex only needs write csr to be written to, the value doesn't matter
    csr_write_simple(1, addresses->write);
    // Wait for the Write to complete
    while (csr_read_simple(addresses->drdy) != 1 && timeout < timeout_limit) {
        busy_wait_us(1);
        timeout++;
    }
    Reset_Drdy_latch(addresses->drdy_reset);
    if (timeout > (timeout_limit - 1)) {
        printf("DRP WRITE TIMED OUT! \n");
    }
}

void Reset_Drdy_latch(uint16_t reset_addr) {
    csr_write_simple(1, reset_addr);
    csr_write_simple(0, reset_addr);
}

void
SetPhase_DRP(PLL_ADDRS *addresses, uint8_t phase_mux, uint8_t delay_time, uint16_t clkreg1_adr, uint16_t clkreg2_adr) {
    uint16_t reg_val;
    // Read current register value and update
    reg_val = Read_MMCM_DRP(addresses, clkreg1_adr); // Read current CLKOUT1 Reg1 Value
    reg_val &= 0x1FFF; // Clear phase mux value (bits 15:13)
    reg_val |= phase_mux
            << 13; // Using &0x7 to extract 3LSB, in essence modulus(8). Shift left to phase mux location
    Write_MMCM_DRP(addresses, clkreg1_adr, reg_val); // Write new CLKOUT1 Reg1 Value
    // Read current register value and update
    reg_val = Read_MMCM_DRP(addresses, clkreg2_adr); // Read current CLKOUT1 Reg2 Value
    reg_val &= 0xFCFF; // Clear MX bits(9:8). This is mandatory
    reg_val &= 0xFFC0; // Clear Delay time bits(5:0)
    reg_val |= delay_time; //(i>>3); // Write phase value divided by 8
    Write_MMCM_DRP(addresses, clkreg2_adr, reg_val); // Write new CLKOUT1 Reg2 Value
}

void Update_MMCM_CFG(PLL_ADDRS *pll_addresses, CLK_CTRL_ADDRS *ctrl_addresses) {
    uint16_t DIVIDE;
    uint8_t BYPASS;
    uint16_t PHASE;

    // Prepare MMCM for reconfig
    csr_write_simple(1, pll_addresses->reset);
    Write_MMCM_DRP(pll_addresses, PowerReg_7Series, 0xffff);

    //    Clock 0
    DIVIDE = csr_read_simple(ctrl_addresses->c0_div_cnt);
    // Add LSB and MSB to get divide value (register contains two values for counters)
    DIVIDE = DIVIDE & 0xFF + DIVIDE >> 8 & 0xFF;
    BYPASS = csr_read_simple(ctrl_addresses->c0_div_byp);
    PHASE = 0;
    if (BYPASS == 1)
        DIVIDE = 1;
    SetMMCM_CLKREG(pll_addresses, DIVIDE, PHASE, ClkReg1_CLKOUT0, ClkReg2_CLKOUT0);

    //    Clock 1
    DIVIDE = csr_read_simple(ctrl_addresses->c1_div_cnt);
    // Add LSB and MSB to get divide value (register contains two values for counters)
    DIVIDE = DIVIDE & 0xFF + DIVIDE >> 8 & 0xFF;
    BYPASS = csr_read_simple(ctrl_addresses->c1_div_byp);
    PHASE = csr_read_simple(ctrl_addresses->c1_phase);
    PHASE = PHASE % 360; // Make sure requested phase is between 0 and 359 deg
    if (BYPASS == 1)
        DIVIDE = 1;
    SetMMCM_CLKREG(pll_addresses, DIVIDE, PHASE, ClkReg1_CLKOUT1, ClkReg2_CLKOUT1);

    // FBCLK
    DIVIDE = csr_read_simple(ctrl_addresses->vco_mult_cnt);
    // Add LSB and MSB to get divide value (register contains two values for counters)
    DIVIDE = DIVIDE & 0xFF + DIVIDE >> 8 & 0xFF;
    BYPASS = csr_read_simple(ctrl_addresses->vco_mult_byp);
    PHASE = 0;
    if (BYPASS == 1)
        DIVIDE = 1;
    SetMMCM_CLKREG(pll_addresses, DIVIDE, PHASE, ClkReg1_CLKFBOUT, ClkReg2_CLKFBOUT);


    //unreset MMCM
    csr_write_simple(0, pll_addresses->reset);
}


void SetMMCM_CLKREG(PLL_ADDRS *addresses, uint8_t DIVIDE, uint32_t PHASE, uint16_t clkreg1_adr, uint16_t clkreg2_adr) {
    uint8_t DELAY_TIME;
    uint8_t PHASE_MUX;
    uint8_t HIGH_TIME;
    uint8_t LOW_TIME;
    uint8_t NO_COUNT;
    uint8_t EDGE;
    uint32_t phase_step =
            (45 * 10000000) / DIVIDE; // 360/(DIVIDE*8); Multiply by 10^7 to get better accuracy with fixed point
    PHASE = PHASE * 10000000; // Multiply by 10^7 to get better accuracy with fixed point
    uint16_t phase_value;

    // calculate and write phase values first
    phase_value = PHASE / phase_step;
    DELAY_TIME = phase_value >> 3;
    PHASE_MUX = phase_value & 0x7;
    SetPhase_DRP(addresses, PHASE_MUX, DELAY_TIME, clkreg1_adr, clkreg2_adr);

    uint16_t reg_val = Read_MMCM_DRP(addresses, clkreg1_adr);
    uint16_t reg_val_2 = Read_MMCM_DRP(addresses, clkreg2_adr);

    // PARSE OLD VALUES
    HIGH_TIME = (reg_val >> 6) & 0x3F;
    LOW_TIME = reg_val & 0x3F;
    //	NO_COUNT = (reg_val_2 >> 6) & 0x1;
    EDGE = (reg_val_2 >> 7) & 0x1;

    // Determine new values
    if (DIVIDE == 1) // If no dividing is required
    {
        NO_COUNT = 1; // Enable NO COUNT byte (this ignores the dividers)
    } else if (DIVIDE % 2 == 0) // If the divider is even
    {
        EDGE = 0;
        NO_COUNT = 0;
        HIGH_TIME = DIVIDE / 2;
        LOW_TIME = DIVIDE / 2;
    } else // Divider is odd
    {
        EDGE = 1;
        NO_COUNT = 0;
        HIGH_TIME = DIVIDE / 2;
        LOW_TIME = (DIVIDE / 2) + 1;
    }

    // Set and write new values
    reg_val &= 0xF000; // Clear HIGH TIME and LOW TIME bits
    reg_val |= LOW_TIME & 0x3F;
    reg_val |= (HIGH_TIME & 0x3F) << 6;

    Write_MMCM_DRP(addresses, clkreg1_adr, reg_val);
    reg_val_2 &= 0xFC3F; // Clear NO COUNT, EDGE, MX(just in case) bits
    reg_val_2 |= NO_COUNT << 6;
    reg_val_2 |= EDGE << 7;
    Write_MMCM_DRP(addresses, clkreg2_adr, reg_val_2);
}

int AutoPH_MMCM_CFG(PLL_ADDRS *pll_addresses, CLK_CTRL_ADDRS *ctrl_addresses, SMPL_CMP_ADDRS *smpl_cmp_addrs) {
    uint8_t phase_mux;
    uint8_t delay_time;
    uint8_t PhaseMin = 0;
    uint8_t PhaseMax = 0;
    uint8_t PhaseMiddle;

    uint8_t cmp_error;
    uint32_t timeout = 0;
    uint32_t timeout_limit = 2000000; // Function will timeout after 2 seconds
    // Read divider counter values
    uint16_t max_phase = csr_read_simple(ctrl_addresses->c1_div_cnt);
    // Sum counter values to get effective divider value
    max_phase = max_phase & 0xFF + max_phase >> 8 & 0xFF;
    // Calculate max phase index
    max_phase = max_phase * 8;

    typedef enum state {
        PHASE_MIN,
        PHASE_MAX,
        PHASE_DONE
    } state_t;

    state_t phase_state = PHASE_MIN;

    // Set initial configuration
    Update_MMCM_CFG(pll_addresses, ctrl_addresses);

    for (uint16_t i = 0; i <= max_phase; i++) {
        phase_mux = (i & 0x7);
        delay_time = i >> 3;

        csr_write_simple(1, pll_addresses->reset);
        Write_MMCM_DRP(pll_addresses, PowerReg_7Series, 0xffff);
        SetPhase_DRP(pll_addresses, phase_mux, delay_time, ClkReg1_CLKOUT1, ClkReg2_CLKOUT1);
        csr_write_simple(0, pll_addresses->reset);

        //         Wait for lock before continuing
        while (csr_read_simple(pll_addresses->locked) == 0) {
            busy_wait_us(1);
            timeout++;
            if (timeout > timeout_limit) {
                return AUTO_PH_MMCM_CFG_TIMEOUT;
            }
        }

        // Reset sample compare module
        csr_write_simple(0, smpl_cmp_addrs->cmp_start);

        // Wait for done to reset before continuing
        while (csr_read_simple(smpl_cmp_addrs->cmp_done) != 0) {
            busy_wait_us(1);
            timeout++;
            if (timeout > timeout_limit) {
                return AUTO_PH_MMCM_CFG_TIMEOUT;
            }
        }

        // Start sample compare module
        csr_write_simple(1, smpl_cmp_addrs->cmp_start);

        //         Wait for sample compare to be done
        while (csr_read_simple(smpl_cmp_addrs->cmp_done) == 0) {
            busy_wait_us(1);
            timeout++;
            if (timeout > timeout_limit) {
                return AUTO_PH_MMCM_CFG_TIMEOUT;
            }
        }
        cmp_error = csr_read_simple(smpl_cmp_addrs->cmp_error);

        switch (phase_state) {
            case PHASE_MIN:
                if (cmp_error == 0) {
                    PhaseMin = i;
                    phase_state = PHASE_MAX;
                }
                break;

            case PHASE_MAX:
                if (cmp_error == 1) {
                    PhaseMax = i - 1;
                    phase_state = PHASE_DONE;
                }
                break;

            case PHASE_DONE:
                PhaseMiddle = (PhaseMin + PhaseMax) / 2;
                phase_mux = (PhaseMiddle & 0x7);
                delay_time = PhaseMiddle >> 3;

                csr_write_simple(1, pll_addresses->reset);
                Write_MMCM_DRP(pll_addresses, PowerReg_7Series, 0xffff);
                SetPhase_DRP(pll_addresses, phase_mux, delay_time, ClkReg1_CLKOUT1, ClkReg2_CLKOUT1);
                csr_write_simple(0, pll_addresses->reset);

            //			 Wait for lock before continuing
                while (csr_read_simple(pll_addresses->locked) == 0) {
                    busy_wait_us(1);
                    timeout++;
                    if (timeout > timeout_limit) {
                        return AUTO_PH_MMCM_CFG_TIMEOUT;
                    }
                }


                return AUTO_PH_MMCM_CFG_SUCCESS;
        }
    }
    // All phase values were checked and no good values found
    return AUTO_PH_MMCM_CFG_FAILURE;
}
