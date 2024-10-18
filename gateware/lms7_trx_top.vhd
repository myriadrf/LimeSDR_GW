-- ----------------------------------------------------------------------------
-- FILE:          lms7_trx_top.vhd
-- DESCRIPTION:   Top level file for LimeSDR-Mini board
-- DATE:          10:06 AM Friday, May 11, 2018
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
--NOTES:
-- ----------------------------------------------------------------------------
-- altera vhdl_input_version vhdl_2008
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.fpgacfg_pkg.all;
use work.pllcfg_pkg.all;
use work.tstcfg_pkg.all;
use work.periphcfg_pkg.all;
use work.FIFO_PACK.all;

library ECP5U;
use ECP5U.components.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity lms7_trx_top is
   generic(
      -- General parameters
      -- LMS7002 related 
      -- FTDI (USB3) related
      FTDI_DQ_WIDTH           : integer := 32;     -- FTDI Data bus size      
      CTRL0_FPGA_RX_SIZE      : integer := 1024;   -- Control PC->FPGA, FIFO size in bytes.
      CTRL0_FPGA_RX_RWIDTH    : integer := 32;     -- Control PC->FPGA, FIFO rd width.
      CTRL0_FPGA_TX_SIZE      : integer := 1024;   -- Control FPGA->PC, FIFO size in bytes
      CTRL0_FPGA_TX_WWIDTH    : integer := 32;     -- Control FPGA->PC, FIFO wr width
      -- 
      -- Internal configuration memory 
      FPGACFG_START_ADDR      : integer := 0;
      PLLCFG_START_ADDR       : integer := 32;
      TSTCFG_START_ADDR       : integer := 96;
      PERIPHCFG_START_ADDR    : integer := 192
   );
   port (
      -- ----------------------------------------------------------------------------
      -- External GND pin for reset
      --EXT_GND           : in     std_logic;
      -- ----------------------------------------------------------------------------
      -- Clock sources
         -- Reference clock, coming from LMK clock buffer.
      reset_n_o         : out    std_logic;
      osc_clk_o         : out    std_logic;

      -- ----------------------------------------------------------------------------
      -- LMS7002 Digital
         -- PORT2
      LMS_TXNRX2_or_CLK_SEL : out    std_logic; --In v2.3 board version this pin is changed to CLK_SEL
         --MISC
      LMS_RESET         : out    std_logic := '1';

      lms_delay_en      : out std_logic;
      lms_delay_sel     : out std_logic_vector(1 downto 0);
      lms_delay_dir     : out std_logic;
      lms_delay_mode    : out std_logic;
      lms_delay_done    : in  std_logic;
      lms_delay_error   : in  std_logic;

      -- ----------------------------------------------------------------------------
      -- FTDI (USB3)
         -- Clock source
      --FT_CLK            : in     std_logic;
      -- FT601 FIFOs Clk/Reset
      EP82_aclrn_o      : out    std_logic;
      EP03_aclrn_o      : out    std_logic;

      -- ctrl
      -- between inst/cpu and external
      EP02_rd_src       : out    std_logic;
      EP02_rdata_src    : in     std_logic_vector(CTRL0_FPGA_RX_RWIDTH-1 downto 0);
      EP02_rempty_src   : in     std_logic;

      -- ctrl
      -- between inst/cpu and external
      EP82_wr_src       : out    std_logic;
      EP82_wdata_src    : out    std_logic_vector(CTRL0_FPGA_TX_WWIDTH-1 downto 0);
      EP82_wfull_src    : in     std_logic;

      -- ----------------------------------------------------------------------------
      -- Tst Top / Clock Test
        --input ports
       tst_test_en      : out std_logic_vector(3 downto 0);
       tst_test_frc_err : out std_logic_vector(3 downto 0);
       tst_test_cmplt   : in  std_logic_vector(3 downto 0);
       tst_test_rez     : in  std_logic_vector(3 downto 0);

       Si5351C_clk_0    : out std_logic;
       Si5351C_clk_1    : out std_logic;
       Si5351C_clk_2    : out std_logic;
       Si5351C_clk_3    : out std_logic;
       Si5351C_clk_5    : out std_logic;
       Si5351C_clk_6    : out std_logic;
       Si5351C_clk_7    : out std_logic;
       ADF_MUXOUT       : out std_logic;

       FX3_clk_cnt      : in  std_logic_vector(15 downto 0);
       Si5351C_clk_0_cnt: in  std_logic_vector(15 downto 0);
       Si5351C_clk_1_cnt: in  std_logic_vector(15 downto 0);
       Si5351C_clk_2_cnt: in  std_logic_vector(15 downto 0);
       Si5351C_clk_3_cnt: in  std_logic_vector(15 downto 0);
       Si5351C_clk_5_cnt: in  std_logic_vector(15 downto 0);
       Si5351C_clk_6_cnt: in  std_logic_vector(15 downto 0);
       Si5351C_clk_7_cnt: in  std_logic_vector(15 downto 0);
       LMK_CLK_cnt      : in  std_logic_vector(23 downto 0);
       ADF_MUXOUT_cnt   : in  std_logic_vector(15 downto 0);

      -- ----------------------------------------------------------------------------
      -- External communication interfaces
         -- FPGA_SPI
      FPGA_SPI_SCLK     : out    std_logic;
      FPGA_SPI_MOSI     : out    std_logic;
      FPGA_SPI_MISO     : in     std_logic;      
      FPGA_SPI_LMS_SS   : out    std_logic;
      FPGA_SPI_DAC_SS   : out    std_logic;
         -- FPGA_CFG
      FPGA_CFG_SPI_MISO : in     std_logic;
      FPGA_CFG_SPI_MOSI : out    std_logic;
      --FPGA_CFG_SPI_SCLK : out    std_logic; -- SCLK pin can be accessed only trough USRMCLK component
      FPGA_CFG_SPI_SS_N : out    std_logic;
         -- FPGA I2C
      FPGA_I2C_SCL      : inout  std_logic;
      FPGA_I2C_SDA      : inout  std_logic;
      -- ----------------------------------------------------------------------------
      -- General periphery
         -- LEDs          
      mico32_busy         : out std_logic;
      led1_ctrl           : out std_logic_vector(2 downto 0);
      led2_ctrl           : out std_logic_vector(2 downto 0);
      led3_ctrl           : out std_logic_vector(2 downto 0);
      -- to_periphcfg
      BOARD_GPIO_RD        : in  std_logic_vector(15 downto 0);
      PERIPH_INPUT_RD_0    : in  std_logic_vector(15 downto 0);
      PERIPH_INPUT_RD_1    : in  std_logic_vector(15 downto 0);
      -- from_periphcfg
      BOARD_GPIO_OVRD      : out std_logic_vector(15 downto 0);
      BOARD_GPIO_DIR       : out std_logic_vector(15 downto 0);
      BOARD_GPIO_VAL       : out std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_OVRD_0 : out std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_VAL_0  : out std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_OVRD_1 : out std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_VAL_1  : out std_logic_vector(15 downto 0);

      -- ----------------------------------------------------------------------------
      -- RXTX
      rxtx_smpl_cmp_length : out std_logic_vector(15 downto 0);
      -- from_fpgacfg
      phase_reg_sel        : out  std_logic_vector(15 downto 0);
      clk_ind              : out  std_logic_vector(4 downto 0);
      cnt_ind              : out  std_logic_vector(4 downto 0);
      load_phase_reg       : out  std_logic;
      drct_clk_en          : out  std_logic_vector(15 downto 0);
      ch_en                : out  std_logic_vector(15 downto 0);
      smpl_width           : out  std_logic_vector(1 downto 0);
      mode                 : out  std_logic;
      ddr_en               : out  std_logic;
      trxiq_pulse          : out  std_logic;
      mimo_int_en          : out  std_logic;
      synch_dis            : out  std_logic;
      synch_mode           : out  std_logic;
      smpl_nr_clr          : out  std_logic;
      txpct_loss_clr       : out  std_logic;
      rx_en                : out  std_logic;
      tx_en                : out  std_logic;
      rx_ptrn_en           : out  std_logic;
      tx_ptrn_en           : out  std_logic;
      tx_cnt_en            : out  std_logic;
      wfm_ch_en            : out  std_logic_vector(15 downto 0);
      wfm_play             : out  std_logic;
      wfm_load             : out  std_logic;
      wfm_smpl_width       : out  std_logic_vector(1 downto 0);
      SPI_SS               : out  std_logic_vector(15 downto 0);
      LMS1_SS              : out  std_logic;
      LMS1_RESET           : out  std_logic;
      LMS1_CORE_LDO_EN     : out  std_logic;
      LMS1_TXNRX1          : out  std_logic;
      LMS1_TXNRX2          : out  std_logic;
      LMS1_TXEN            : out  std_logic;
      LMS1_RXEN            : out  std_logic;
      GPIO                 : out  std_logic_vector(15 downto 0);
      FPGA_LED1_CTRL       : out  std_logic_vector(2 downto 0);
      FPGA_LED2_CTRL       : out  std_logic_vector(2 downto 0);
      FX3_LED_CTRL         : out  std_logic_vector(2 downto 0);
      CLK_ENA              : out  std_logic_vector(3 downto 0);
      sync_pulse_period    : out  std_logic_vector(31 downto 0);
      sync_size            : out  std_logic_vector(15 downto 0);
      txant_pre            : out  std_logic_vector(15 downto 0);
      txant_post           : out  std_logic_vector(15 downto 0);
      -- to_tst_cfg_from_rxtx
      DDR2_1_STATUS        : in   std_logic_vector(2 downto 0);
      DDR2_1_pnf_per_bit   : in   std_logic_vector(31 downto 0);
      --from_tstcfg
      TEST_EN              : out  std_logic_vector(5 downto 0);
      TEST_FRC_ERR         : out  std_logic_vector(5 downto 0);
      TX_TST_I             : out  std_logic_vector(15 downto 0);
      TX_TST_Q             : out  std_logic_vector(15 downto 0);

         -- RF loop back control 
      RFSW_RX_V1        : out    std_logic;
      RFSW_RX_V2        : out    std_logic;
      RFSW_TX_V1        : out    std_logic;
      RFSW_TX_V2        : out    std_logic;
      TX_LB_AT          : out    std_logic;
      TX_LB_SH          : out    std_logic;
         -- Bill Of material and hardware version 
      BOM_VER           : in     std_logic_vector(2 downto 0);
      HW_VER            : in     std_logic_vector(3 downto 0)  --PULL UP has to be enabled for HW_VER

   );
end lms7_trx_top;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of lms7_trx_top is
--declare signals,  components here
signal ext_rst                   : std_logic;
signal por_rst_vect              : std_logic_vector(3 downto 0);
signal por_rst_n                 : std_logic;

signal reset_n                   : std_logic; 

--inst0 (NIOS CPU instance)
signal inst0_exfifo_if_rd        : std_logic;
signal inst0_exfifo_of_d         : std_logic_vector(FTDI_DQ_WIDTH-1 downto 0);
signal inst0_exfifo_of_wr        : std_logic;
signal inst0_exfifo_of_rst       : std_logic;
signal inst0_gpo                 : std_logic_vector(7 downto 0);
signal inst0_gpo_reg             : std_logic_vector(7 downto 0);
signal inst0_lms_ctr_gpio        : std_logic_vector(3 downto 0);
signal inst0_spi_0_MISO          : std_logic;
signal inst0_spi_0_MOSI          : std_logic;
signal inst0_spi_0_SCLK          : std_logic;
signal inst0_spi_0_SS_n          : std_logic_vector(4 downto 0);
signal inst0_spi_1_MISO          : std_logic;
signal inst0_spi_1_MOSI          : std_logic;
signal inst0_spi_1_SCLK          : std_logic;
signal inst0_spi_1_SS_n          : std_logic_vector(1 downto 0);
signal inst0_from_fpgacfg        : t_FROM_FPGACFG;
signal inst0_to_fpgacfg          : t_TO_FPGACFG;
signal inst0_from_pllcfg         : t_FROM_PLLCFG;
signal inst0_to_pllcfg           : t_TO_PLLCFG;
signal inst0_from_tstcfg         : t_FROM_TSTCFG;
signal inst0_to_tstcfg           : t_TO_TSTCFG;
signal inst0_from_periphcfg      : t_FROM_PERIPHCFG;
signal inst0_to_periphcfg        : t_TO_PERIPHCFG;
signal cpu_alive_cnt             : std_logic_vector(15 downto 0);
signal inst0_fpga_cfg_usrmclkts  : std_logic_vector(0 downto 0);
signal inst0_fpga_cfg_spi_MOSI   : std_logic;
signal inst0_fpga_cfg_spi_SCLK   : std_logic;
signal inst0_fpga_cfg_spi_SS_n   : std_logic;

signal u1_USRMCLKI               : std_logic;

--inst2
constant C_EP02_RDUSEDW_WIDTH    : integer := FIFO_WORDS_TO_Nbits(CTRL0_FPGA_RX_SIZE/(CTRL0_FPGA_RX_RWIDTH/8),true);
constant C_EP82_WRUSEDW_WIDTH    : integer := FIFO_WORDS_TO_Nbits(CTRL0_FPGA_TX_SIZE/(CTRL0_FPGA_TX_WWIDTH/8),true);
signal inst2_EP82_wfull          : std_logic;
signal inst2_EP02_rdata          : std_logic_vector(CTRL0_FPGA_RX_RWIDTH-1 downto 0);
signal inst2_EP02_rempty         : std_logic;
signal inst2_EP02_rdusedw        : std_logic_vector(C_EP02_RDUSEDW_WIDTH-1 downto 0);

--inst6
signal inst6_to_tstcfg_from_rxtx    : t_TO_TSTCFG_FROM_RXTX;

signal osc_clk             : std_logic;

attribute keep: boolean;
attribute keep of por_rst_vect: signal is true;


-- Component for FPGA configuration flash access
COMPONENT USRMCLK
PORT(
   USRMCLKI    : IN STD_ULOGIC;
   USRMCLKTS   : IN STD_ULOGIC
);
END COMPONENT;

attribute syn_noprune: boolean ;
attribute syn_noprune of USRMCLK: component is true;

COMPONENT OSCG
--synthesis translate_off
   GENERIC (
      DIV: integer := 128
   );
--synthesis translate_on
   PORT (
      OSC : OUT std_logic
   );
END COMPONENT;

attribute DIV : integer;
attribute DIV of OSCinst0 : label is 4;


begin

-- ----------------------------------------------------------------------------
-- Internal Oscilator
-- ----------------------------------------------------------------------------
-- DIV values: 2(~155MHz) - 128(~2.4MHz)
OSCInst0: OSCG
--synthesis translate_off
   GENERIC MAP (DIV => 4)
--synthesis translate_on
   PORT MAP (OSC => osc_clk);
osc_clk_o <= osc_clk;

-- ----------------------------------------------------------------------------
-- Reset logic
-- ----------------------------------------------------------------------------
   -- HW_VER(3) is connected to GND
   ext_rst  <= HW_VER(3);
   
   process(ext_rst, osc_clk)
   begin 
      if ext_rst = '1' then 
         por_rst_vect <= (others=>'0');
      elsif rising_edge(osc_clk) then 
         por_rst_vect <= por_rst_vect(2 downto 0) & '1';
      end if;
   end process;
      
   reset_n <= '1' when por_rst_vect = "1111" else '0';
   reset_n_o <= reset_n;
  
   -- Reset for all logic. 
   --reset_n <= por_rst_n;  

-- ----------------------------------------------------------------------------
-- CPU (Mico32) instance.
-- CPU is responsible for communication interfaces and control logic
-- ----------------------------------------------------------------------------   
   inst0_cpu : entity work.cpu
   generic map (
      FPGACFG_START_ADDR   => FPGACFG_START_ADDR,
      PLLCFG_START_ADDR    => PLLCFG_START_ADDR,
      TSTCFG_START_ADDR    => TSTCFG_START_ADDR,
      PERIPHCFG_START_ADDR => PERIPHCFG_START_ADDR
   )
   port map(
      clk                        => osc_clk,
      reset_n                    => reset_n,
      -- Control data FIFO
      exfifo_if_d                => inst2_EP02_rdata,
      exfifo_if_rd               => inst0_exfifo_if_rd, 
      exfifo_if_rdempty          => inst2_EP02_rempty,
      exfifo_of_d                => inst0_exfifo_of_d, 
      exfifo_of_wr               => inst0_exfifo_of_wr, 
      exfifo_of_wrfull           => inst2_EP82_wfull,
      exfifo_of_rst              => inst0_exfifo_of_rst, 
      -- SPI 0 
      spi_0_MISO                 => FPGA_SPI_MISO,
      spi_0_MOSI                 => inst0_spi_0_MOSI,
      spi_0_SCLK                 => inst0_spi_0_SCLK,
      spi_0_SS_n                 => inst0_spi_0_SS_n,
      -- SPI 1
      spi_1_MISO                 => '0',--FPGA_QSPI_IO1,
      spi_1_MOSI                 => inst0_spi_1_MOSI,
      spi_1_SCLK                 => inst0_spi_1_SCLK,
      spi_1_SS_n                 => inst0_spi_1_SS_n,
         -- FPGA_CFG_SPI
      fpga_cfg_spi_MISO          => FPGA_CFG_SPI_MISO, 
      fpga_cfg_spi_MOSI          => inst0_fpga_cfg_spi_MOSI,
      fpga_cfg_spi_SCLK          => inst0_fpga_cfg_spi_SCLK,
      fpga_cfg_spi_SS_n          => inst0_fpga_cfg_spi_SS_n,
      fpga_cfg_usrmclkts         => inst0_fpga_cfg_usrmclkts,
      -- I2C
      i2c_scl                    => FPGA_I2C_SCL,
      i2c_sda                    => FPGA_I2C_SDA,
      -- Genral purpose I/O
      gpi                        => (others=>'0'),
      gpo                        => inst0_gpo, 
      -- LMS7002 control 
      lms_ctr_gpio               => inst0_lms_ctr_gpio,
      -- Configuration registers
      from_fpgacfg               => inst0_from_fpgacfg,
      to_fpgacfg                 => inst0_to_fpgacfg,
      from_pllcfg                => inst0_from_pllcfg,
      to_pllcfg                  => inst0_to_pllcfg,
      from_tstcfg                => inst0_from_tstcfg,
      to_tstcfg                  => inst0_to_tstcfg,
      to_tstcfg_from_rxtx        => inst6_to_tstcfg_from_rxtx,
      from_periphcfg             => inst0_from_periphcfg,
      to_periphcfg               => inst0_to_periphcfg
   );
   
   -- Module to access FPGA_CFG_SPI_SCLK pin
   u1: USRMCLK port map (
      USRMCLKI    => u1_USRMCLKI,
      USRMCLKTS   => inst0_fpga_cfg_usrmclkts(0)
   );
   
   u1_USRMCLKI <= inst0_fpga_cfg_spi_SCLK when inst0_fpga_cfg_spi_SS_n ='0' else '0';
   
   inst0_to_fpgacfg.HW_VER    <= HW_VER;
   inst0_to_fpgacfg.BOM_VER   <= '0' & BOM_VER; 
   inst0_to_fpgacfg.PWR_SRC   <= '0';
   
   --CPU alive status
   busy_delay_inst : entity work.busy_delay
   generic map(
      clock_period   => 25,  -- input clock period in ns
      delay_time     => 100  -- delay time in ms
   )
   port map(
      clk      => osc_clk,
      reset_n  => reset_n,
      busy_in  => inst0_gpo(0),
      busy_out => mico32_busy
   );
   
   process(osc_clk, reset_n)
   begin 
      if reset_n = '0' then 
         cpu_alive_cnt <= (others=>'0');
         inst0_gpo_reg <= (others=>'0');
      elsif rising_edge(osc_clk) then 
         inst0_gpo_reg <= inst0_gpo;
         --Increment on rising edge
         if inst0_gpo_reg(0) = '0' AND inst0_gpo(0) = '1' then 
            cpu_alive_cnt <= std_logic_vector(unsigned(cpu_alive_cnt) +1);
         else 
            cpu_alive_cnt <= cpu_alive_cnt;
         end if;
      end if;
   end process;
   
-- ----------------------------------------------------------------------------
-- pll_top instance.
-- Clock source for LMS7002 RX and TX logic
-- ----------------------------------------------------------------------------   
   
   --TODO: remove pll config bypass logic
   inst0_to_pllcfg.pllcfg_busy    <= '0';
   inst0_to_pllcfg.pllcfg_done    <= '1';
   inst0_to_pllcfg.pll_lock       <= (others=>'0');
   
   lms_delay_en         <= inst0_from_pllcfg.phcfg_start;
   lms_delay_sel        <= "00" when inst0_from_pllcfg.cnt_ind= "00011" else 
                           "11";
   lms_delay_dir        <= inst0_from_pllcfg.phcfg_updn;
   lms_delay_mode       <= inst0_from_pllcfg.phcfg_mode;
   
   inst0_to_pllcfg.phcfg_done    <= lms_delay_done;
   inst0_to_pllcfg.phcfg_error   <= lms_delay_error; 
   
-- ----------------------------------------------------------------------------
-- FT601_top instance.
-- USB3 interface 
-- ----------------------------------------------------------------------------

   -- cpu to external
   -- PC->FPGA
   EP02_rd_src       <= inst0_exfifo_if_rd;
   inst2_EP02_rdata  <= EP02_rdata_src;
   inst2_EP02_rempty <= EP02_rempty_src;
   -- FPGA->PC
   EP82_wr_src       <= inst0_exfifo_of_wr;
   EP82_wdata_src    <= inst0_exfifo_of_d;
   inst2_EP82_wfull  <= EP82_wfull_src;

   EP82_aclrn_o       <= not inst0_exfifo_of_rst;
   EP03_aclrn_o       <= inst0_from_fpgacfg.rx_en;

-- ----------------------------------------------------------------------------
-- tst_top instance.
-- Clock test logic
-- ----------------------------------------------------------------------------
   tst_test_en      <= inst0_from_tstcfg.TEST_EN(3 downto 0);
   tst_test_frc_err <= inst0_from_tstcfg.TEST_FRC_ERR(3 downto 0);
   inst0_to_tstcfg.TEST_CMPLT(3 downto 0) <= tst_test_cmplt;
   inst0_to_tstcfg.TEST_REZ(3 downto 0)   <= tst_test_rez;
   Si5351C_clk_0 <= '0';
   Si5351C_clk_1 <= '0';
   Si5351C_clk_2 <= '0';
   Si5351C_clk_3 <= '0';
   Si5351C_clk_5 <= '0';
   Si5351C_clk_6 <= '0';
   Si5351C_clk_7 <= '0';
   ADF_MUXOUT    <= '0';

   inst0_to_tstcfg.FX3_CLK_CNT      <= FX3_clk_cnt;
   inst0_to_tstcfg.Si5351C_CLK0_CNT <= Si5351C_clk_0_cnt;
   inst0_to_tstcfg.Si5351C_CLK1_CNT <= Si5351C_clk_1_cnt;
   inst0_to_tstcfg.Si5351C_CLK2_CNT <= Si5351C_clk_2_cnt;
   inst0_to_tstcfg.Si5351C_CLK3_CNT <= Si5351C_clk_3_cnt;
   inst0_to_tstcfg.Si5351C_CLK5_CNT <= Si5351C_clk_5_cnt;
   inst0_to_tstcfg.Si5351C_CLK6_CNT <= Si5351C_clk_6_cnt;
   inst0_to_tstcfg.Si5351C_CLK7_CNT <= Si5351C_clk_7_cnt;
   inst0_to_tstcfg.LMK_CLK_CNT      <= LMK_CLK_cnt;
   inst0_to_tstcfg.ADF_CNT          <= ADF_MUXOUT_cnt;
   
-- ----------------------------------------------------------------------------
-- general_periph_top instance.
-- Control module for external periphery
-- ----------------------------------------------------------------------------
   led1_ctrl   <= inst0_from_fpgacfg.FPGA_LED1_CTRL;
   led2_ctrl   <= inst0_from_fpgacfg.FPGA_LED2_CTRL;
   led3_ctrl   <= inst0_from_fpgacfg.FX3_LED_CTRL;

   -- to_periphcfg
   inst0_to_periphcfg.BOARD_GPIO_RD     <= BOARD_GPIO_RD;
   inst0_to_periphcfg.PERIPH_INPUT_RD_0 <= PERIPH_INPUT_RD_0;
   inst0_to_periphcfg.PERIPH_INPUT_RD_1 <= PERIPH_INPUT_RD_1;
   -- from_periphcfg
   BOARD_GPIO_OVRD      <= inst0_from_periphcfg.BOARD_GPIO_OVRD;
   BOARD_GPIO_DIR       <= inst0_from_periphcfg.BOARD_GPIO_DIR;
   BOARD_GPIO_VAL       <= inst0_from_periphcfg.BOARD_GPIO_VAL;
   PERIPH_OUTPUT_OVRD_0 <= inst0_from_periphcfg.PERIPH_OUTPUT_OVRD_0;
   PERIPH_OUTPUT_VAL_0  <= inst0_from_periphcfg.PERIPH_OUTPUT_VAL_0;
   PERIPH_OUTPUT_OVRD_1 <= inst0_from_periphcfg.PERIPH_OUTPUT_OVRD_1;
   PERIPH_OUTPUT_VAL_1  <= inst0_from_periphcfg.PERIPH_OUTPUT_VAL_1;
   
 ----------------------------------------------------------------------------
 -- rxtx_top instance.
 -- Receive and transmit interface for LMS7002
 ----------------------------------------------------------------------------
   rxtx_smpl_cmp_length <= inst0_from_pllcfg.auto_phcfg_smpls;
   phase_reg_sel     <= inst0_from_fpgacfg.phase_reg_sel;
   clk_ind           <= inst0_from_fpgacfg.clk_ind;
   cnt_ind           <= inst0_from_fpgacfg.cnt_ind;
   load_phase_reg    <= inst0_from_fpgacfg.load_phase_reg;
   drct_clk_en       <= inst0_from_fpgacfg.drct_clk_en;
   ch_en             <= inst0_from_fpgacfg.ch_en;
   smpl_width        <= inst0_from_fpgacfg.smpl_width;
   mode              <= inst0_from_fpgacfg.mode;
   ddr_en            <= inst0_from_fpgacfg.ddr_en;
   trxiq_pulse       <= inst0_from_fpgacfg.trxiq_pulse;
   mimo_int_en       <= inst0_from_fpgacfg.mimo_int_en;
   synch_dis         <= inst0_from_fpgacfg.synch_dis;
   synch_mode        <= inst0_from_fpgacfg.synch_mode;
   smpl_nr_clr       <= inst0_from_fpgacfg.smpl_nr_clr;
   txpct_loss_clr    <= inst0_from_fpgacfg.txpct_loss_clr;
   rx_en             <= inst0_from_fpgacfg.rx_en;
   tx_en             <= inst0_from_fpgacfg.tx_en;
   rx_ptrn_en        <= inst0_from_fpgacfg.rx_ptrn_en;
   tx_ptrn_en        <= inst0_from_fpgacfg.tx_ptrn_en;
   tx_cnt_en         <= inst0_from_fpgacfg.tx_cnt_en;
   wfm_ch_en         <= inst0_from_fpgacfg.wfm_ch_en;
   wfm_play          <= inst0_from_fpgacfg.wfm_play;
   wfm_load          <= inst0_from_fpgacfg.wfm_load;
   wfm_smpl_width    <= inst0_from_fpgacfg.wfm_smpl_width;
   SPI_SS            <= inst0_from_fpgacfg.SPI_SS;
   LMS1_SS           <= inst0_from_fpgacfg.LMS1_SS;
   LMS1_RESET        <= inst0_from_fpgacfg.LMS1_RESET;
   LMS1_CORE_LDO_EN  <= inst0_from_fpgacfg.LMS1_CORE_LDO_EN;
   LMS1_TXNRX1       <= inst0_from_fpgacfg.LMS1_TXNRX1;
   LMS1_TXNRX2       <= inst0_from_fpgacfg.LMS1_TXNRX2;
   LMS1_TXEN         <= inst0_from_fpgacfg.LMS1_TXEN;
   LMS1_RXEN         <= inst0_from_fpgacfg.LMS1_RXEN;
   GPIO              <= inst0_from_fpgacfg.GPIO;
   FPGA_LED1_CTRL    <= inst0_from_fpgacfg.FPGA_LED1_CTRL;
   FPGA_LED2_CTRL    <= inst0_from_fpgacfg.FPGA_LED2_CTRL;
   FX3_LED_CTRL      <= inst0_from_fpgacfg.FX3_LED_CTRL;
   CLK_ENA           <= inst0_from_fpgacfg.CLK_ENA;
   sync_pulse_period <= inst0_from_fpgacfg.sync_pulse_period;
   sync_size         <= inst0_from_fpgacfg.sync_size;
   txant_pre         <= inst0_from_fpgacfg.txant_pre;
   txant_post        <= inst0_from_fpgacfg.txant_post;

   inst6_to_tstcfg_from_rxtx.DDR2_1_STATUS      <= DDR2_1_STATUS;
   inst6_to_tstcfg_from_rxtx.DDR2_1_pnf_per_bit <= DDR2_1_pnf_per_bit;

   TEST_EN          <= inst0_from_tstcfg.TEST_EN;
   TEST_FRC_ERR     <= inst0_from_tstcfg.TEST_FRC_ERR;
   TX_TST_I         <= inst0_from_tstcfg.TX_TST_I;
   TX_TST_Q         <= inst0_from_tstcfg.TX_TST_Q;

-- ----------------------------------------------------------------------------
-- Output ports
-- ----------------------------------------------------------------------------

   FPGA_SPI_MOSI     <= inst0_spi_0_MOSI;
   FPGA_SPI_SCLK     <= inst0_spi_0_SCLK;
   FPGA_SPI_LMS_SS   <= inst0_spi_0_SS_n(0);
   FPGA_SPI_DAC_SS   <= inst0_spi_0_SS_n(2);
    
   FPGA_CFG_SPI_MOSI <= inst0_fpga_cfg_spi_MOSI;
   FPGA_CFG_SPI_SS_N <= inst0_fpga_cfg_spi_SS_n;
   
   LMS_RESET         <= inst0_from_fpgacfg.LMS1_RESET AND inst0_lms_ctr_gpio(0);
   
   --In HW versions before v2.3 this pin is LMS_TXNRX2. After v2.3 - Clock select for LMK clock buffer (CLK_SEL) 
   LMS_TXNRX2_or_CLK_SEL <=   inst0_from_periphcfg.PERIPH_OUTPUT_VAL_1(0) when unsigned(HW_VER) > 5 else 
                              inst0_from_fpgacfg.LMS1_TXNRX2;
   
   RFSW_RX_V1        <= inst0_from_fpgacfg.GPIO(8);
   RFSW_RX_V2        <= inst0_from_fpgacfg.GPIO(9);
   RFSW_TX_V1        <= inst0_from_fpgacfg.GPIO(12);
   RFSW_TX_V2        <= inst0_from_fpgacfg.GPIO(13);
   TX_LB_AT          <= inst0_from_fpgacfg.GPIO(1);
   TX_LB_SH          <= inst0_from_fpgacfg.GPIO(2);
   
   --FT_WAKEUPn        <= '1';
   
   
end arch;   
