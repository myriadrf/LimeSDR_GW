-- ----------------------------------------------------------------------------
-- FILE:          general_periph_top.vhd
-- DESCRIPTION:   Top wrapper file for general periphery components
-- DATE:          3:39 PM Monday, May 7, 2018
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
--NOTES:
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use work.periphcfg_pkg.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity general_periph_top_wrapper is
   generic(
      DEV_FAMILY  : string := "MAX 10";
      N_GPIO      : integer := 8
   );
   port (
      -- General ports
      clk                  : in  std_logic; -- Free running clock
      reset_n              : in  std_logic; -- Asynchronous, active low reset

      -- to_periphcfg
      BOARD_GPIO_RD        : out std_logic_vector(15 downto 0);
      PERIPH_INPUT_RD_0    : out std_logic_vector(15 downto 0);
      PERIPH_INPUT_RD_1    : out std_logic_vector(15 downto 0);
      -- from_periphcfg
      BOARD_GPIO_OVRD      : in  std_logic_vector(15 downto 0);
      BOARD_GPIO_DIR       : in  std_logic_vector(15 downto 0);
      BOARD_GPIO_VAL       : in  std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_OVRD_0 : in  std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_VAL_0  : in  std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_OVRD_1 : in  std_logic_vector(15 downto 0);
      PERIPH_OUTPUT_VAL_1  : in  std_logic_vector(15 downto 0);

      -- Dual colour LEDs
      -- LED1 (Clock and PLL lock status)
      led1_mico32_busy     : in     std_logic;
      led1_ctrl            : in     std_logic_vector(2 downto 0);
      led1_g               : out    std_logic;
      led1_r               : out    std_logic;

      --LED2 (TCXO control status)
      led2_clk             : in     std_logic;
      led2_adf_muxout      : in     std_logic;
      led2_dac_ss          : in     std_logic;
      led2_adf_ss          : in     std_logic;
      led2_ctrl            : in     std_logic_vector(2 downto 0);
      led2_g               : out    std_logic;
      led2_r               : out    std_logic;

      --LED3 (FX3 and NIOS CPU busy)
      led3_g_in            : in     std_logic;
      led3_r_in            : in     std_logic;
      led3_ctrl            : in     std_logic_vector(2 downto 0);
      led3_hw_ver          : in     std_logic_vector(3 downto 0);
      led3_g               : out    std_logic;
      led3_r               : out    std_logic;

      --GPIO
      gpio_dir             : in     std_logic_vector(N_GPIO-1 downto 0);
      gpio_out_val         : in     std_logic_vector(N_GPIO-1 downto 0);
      gpio_rd_val          : out    std_logic_vector(N_GPIO-1 downto 0);
      gpio                 : inout  std_logic_vector(7 downto 0); -- to FPGA pins
      egpio                : inout  std_logic_vector(1 downto 0); -- to FPGA pins

      --Fan control
      fan_sens_in          : in     std_logic;
      fan_ctrl_out         : out    std_logic
   );
end general_periph_top_wrapper;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of general_periph_top_wrapper is
    signal to_periphcfg   : t_TO_PERIPHCFG;
    signal from_periphcfg : t_FROM_PERIPHCFG;
begin
    -- to_periphcfg
    BOARD_GPIO_RD                       <= to_periphcfg.BOARD_GPIO_RD;
    PERIPH_INPUT_RD_0                   <= to_periphcfg.PERIPH_INPUT_RD_0;
    PERIPH_INPUT_RD_1                   <= to_periphcfg.PERIPH_INPUT_RD_1;
    -- from_periphcfg
    from_periphcfg.BOARD_GPIO_OVRD      <= BOARD_GPIO_OVRD;
    from_periphcfg.BOARD_GPIO_DIR       <= BOARD_GPIO_DIR;
    from_periphcfg.BOARD_GPIO_VAL       <= BOARD_GPIO_VAL;
    from_periphcfg.PERIPH_OUTPUT_OVRD_0 <= PERIPH_OUTPUT_OVRD_0;
    from_periphcfg.PERIPH_OUTPUT_VAL_0  <= PERIPH_OUTPUT_VAL_0;
    from_periphcfg.PERIPH_OUTPUT_OVRD_1 <= PERIPH_OUTPUT_OVRD_1;
    from_periphcfg.PERIPH_OUTPUT_VAL_1  <= PERIPH_OUTPUT_VAL_1;

    inst: entity work.general_periph_top
       generic map(
          DEV_FAMILY  => DEV_FAMILY,
          N_GPIO      => N_GPIO
       )
       port map (
          -- General ports
          clk                  => clk,
          reset_n              => reset_n,

          to_periphcfg         => to_periphcfg,
          from_periphcfg       => from_periphcfg,

          -- Dual colour LEDs
          -- LED1 (Clock and PLL lock status)
          led1_mico32_busy     => led1_mico32_busy,
          led1_ctrl            => led1_ctrl,
          led1_g               => led1_g,
          led1_r               => led1_r,

          --LED2 (TCXO control status)
          led2_clk             => led2_clk,
          led2_adf_muxout      => led2_adf_muxout,
          led2_dac_ss          => led2_dac_ss,
          led2_adf_ss          => led2_adf_ss,
          led2_ctrl            => led2_ctrl,
          led2_g               => led2_g,
          led2_r               => led2_r,

          --LED3 (FX3 and NIOS CPU busy)
          led3_g_in            => led3_g_in,
          led3_r_in            => led3_r_in,
          led3_ctrl            => led3_ctrl,
          led3_hw_ver          => led3_hw_ver,
          led3_g               => led3_g,
          led3_r               => led3_r,

          --GPIO
          gpio_dir             => gpio_dir,
          gpio_out_val         => gpio_out_val,
          gpio_rd_val          => gpio_rd_val,
          gpio(7 downto 0)     => gpio,
          gpio(9 downto 8)     => egpio,

          --Fan control
          fan_sens_in          => fan_sens_in,
          fan_ctrl_out         => fan_ctrl_out
       );
end arch;
