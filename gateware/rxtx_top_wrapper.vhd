   -- ----------------------------------------------------------------------------
-- FILE:          rxtx_top.vhd
-- DESCRIPTION:   Top wrapper file for RX and TX components
-- DATE:          9:47 AM Thursday, May 10, 2018
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
use work.tstcfg_pkg.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity rxtx_top_wrapper is
   generic(
      DEV_FAMILY              : string := "MAX 10";
      -- TX parameters
      TX_IQ_WIDTH             : integer := 12;
      TX_N_BUFF               : integer := 4; -- 2,4 valid values
      TX_IN_PCT_SIZE          : integer := 4096; -- TX packet size in bytes
      TX_IN_PCT_HDR_SIZE      : integer := 16;
      TX_IN_PCT_DATA_W        : integer := 128;
      TX_IN_PCT_RDUSEDW_W     : integer := 11;
      TX_OUT_PCT_DATA_W       : integer := 64
   );
   port (
      -- Configuration memory ports
      --from_fpgacfg            : in     t_FROM_FPGACFG;
      ch_en               : in std_logic_vector(15 downto 0);
      smpl_width          : in std_logic_vector(1 downto 0);
      mode                : in std_logic;
      ddr_en              : in std_logic;
      trxiq_pulse         : in std_logic;
      mimo_int_en         : in std_logic;
      synch_dis           : in std_logic;
      synch_mode          : in std_logic;
      txpct_loss_clr      : in std_logic;
      rx_en               : in std_logic;
      tx_ptrn_en          : in std_logic;
      tx_cnt_en           : in std_logic;
      wfm_play            : in std_logic;
      sync_pulse_period   : in std_logic_vector(31 downto 0);
      sync_size           : in std_logic_vector(15 downto 0);
      txant_pre           : in std_logic_vector(15 downto 0);
      txant_post          : in std_logic_vector(15 downto 0);

      ---- to_tstcfg_from_rxtx     : out    t_TO_TSTCFG_FROM_RXTX;
      DDR2_1_STATUS       : out std_logic_vector(2 downto 0);
      DDR2_1_pnf_per_bit  : out std_logic_vector(31 downto 0);

      --from_tstcfg             : in     t_FROM_TSTCFG;
      TEST_EN             : in  std_logic_vector(5 downto 0);
      TEST_FRC_ERR        : in  std_logic_vector(5 downto 0);
      TX_TST_I            : in  std_logic_vector(15 downto 0);
      TX_TST_Q            : in  std_logic_vector(15 downto 0);

      ---- TX path
      tx_clk              : in  std_logic;
      tx_clkout           : out std_logic;
      tx_clk_reset_n      : in  std_logic;
      tx_pct_loss_flg     : out std_logic;
      tx_txant_en         : out std_logic;
         -- Tx interface data
      tx_diq1_h           : out std_logic_vector(TX_IQ_WIDTH downto 0);
      tx_diq1_l           : out std_logic_vector(TX_IQ_WIDTH downto 0);
         -- TX FIFO read ports
      tx_in_pct_rdreq     : out std_logic;
      tx_in_pct_data      : in  std_logic_vector(TX_IN_PCT_DATA_W-1 downto 0);
      tx_in_pct_rdempty   : in  std_logic;
      tx_in_pct_rdusedw   : in  std_logic_vector(TX_IN_PCT_RDUSEDW_W-1 downto 0);

      smpl_nr_cnt         : in 	std_logic_vector(63 downto 0);
      pct_hdr_cap         : in 	std_logic;

      ------ RX path
      rx_clk              : in  std_logic
      );
end rxtx_top_wrapper;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of rxtx_top_wrapper is
  signal from_fpgacfg        : t_FROM_FPGACFG;
  signal to_tstcfg_from_rxtx : t_TO_TSTCFG_FROM_RXTX;
  signal from_tstcfg         : t_FROM_TSTCFG;
begin

   -- All signals with a default value must be considered
   -- as unused.
   from_fpgacfg.phase_reg_sel     <= (others => '0');
   from_fpgacfg.clk_ind           <= (others => '0');
   from_fpgacfg.cnt_ind           <= (others => '0');
   from_fpgacfg.load_phase_reg    <= '0';
   from_fpgacfg.drct_clk_en       <= (others => '0');
   from_fpgacfg.ch_en             <= ch_en;
   from_fpgacfg.smpl_width        <= smpl_width;
   from_fpgacfg.mode              <= mode;
   from_fpgacfg.ddr_en            <= ddr_en;
   from_fpgacfg.trxiq_pulse       <= trxiq_pulse;
   from_fpgacfg.mimo_int_en       <= mimo_int_en;
   from_fpgacfg.synch_dis         <= synch_dis;
   from_fpgacfg.synch_mode        <= synch_mode;
   from_fpgacfg.smpl_nr_clr       <= '0';
   from_fpgacfg.txpct_loss_clr    <= txpct_loss_clr;
   from_fpgacfg.rx_en             <= rx_en;
   from_fpgacfg.tx_en             <= '0';
   from_fpgacfg.rx_ptrn_en        <= '0'; --rx_ptrn_en;
   from_fpgacfg.tx_ptrn_en        <= tx_ptrn_en;
   from_fpgacfg.tx_cnt_en         <= tx_cnt_en;
   from_fpgacfg.wfm_ch_en         <= (others => '0');
   from_fpgacfg.wfm_play          <= wfm_play;
   from_fpgacfg.wfm_load          <= '0';
   from_fpgacfg.wfm_smpl_width    <= (others => '0');
   from_fpgacfg.SPI_SS            <= (others => '0');
   from_fpgacfg.LMS1_SS           <= '0';
   from_fpgacfg.LMS1_RESET        <= '0';
   from_fpgacfg.LMS1_CORE_LDO_EN  <= '0';
   from_fpgacfg.LMS1_TXNRX1       <= '0';
   from_fpgacfg.LMS1_TXNRX2       <= '0';
   from_fpgacfg.LMS1_TXEN         <= '0';
   from_fpgacfg.LMS1_RXEN         <= '0';
   from_fpgacfg.GPIO              <= (others => '0');
   from_fpgacfg.FPGA_LED1_CTRL    <= (others => '0');
   from_fpgacfg.FPGA_LED2_CTRL    <= (others => '0');
   from_fpgacfg.FX3_LED_CTRL      <= (others => '0');
   from_fpgacfg.CLK_ENA           <= (others => '0');
   from_fpgacfg.sync_pulse_period <= sync_pulse_period;
   from_fpgacfg.sync_size         <= sync_size;
   from_fpgacfg.txant_pre         <= txant_pre;
   from_fpgacfg.txant_post        <= txant_post;

   DDR2_1_STATUS                  <= to_tstcfg_from_rxtx.DDR2_1_STATUS;
   DDR2_1_pnf_per_bit             <= to_tstcfg_from_rxtx.DDR2_1_pnf_per_bit;

   from_tstcfg.TEST_EN            <= TEST_EN;
   from_tstcfg.TEST_FRC_ERR       <= TEST_FRC_ERR;
   from_tstcfg.TX_TST_I           <= TX_TST_I;
   from_tstcfg.TX_TST_Q           <= TX_TST_Q;

   rx_tx_top_inst : entity work.rxtx_top
   generic map (
      DEV_FAMILY             => DEV_FAMILY,
      -- TX parameters
      TX_IQ_WIDTH            => TX_IQ_WIDTH,
      TX_N_BUFF              => TX_N_BUFF,
      TX_IN_PCT_SIZE         => TX_IN_PCT_SIZE,
      TX_IN_PCT_HDR_SIZE     => TX_IN_PCT_HDR_SIZE,
      TX_IN_PCT_DATA_W       => TX_IN_PCT_DATA_W,
      TX_IN_PCT_RDUSEDW_W    => TX_IN_PCT_RDUSEDW_W,
      TX_OUT_PCT_DATA_W      => TX_OUT_PCT_DATA_W
     )
     port map (
      -- Configuration memory ports
      from_fpgacfg           => from_fpgacfg,
      to_tstcfg_from_rxtx    => to_tstcfg_from_rxtx,
      from_tstcfg            => from_tstcfg,
      -- TX path
      tx_clk                 => tx_clk,
      tx_clkout              => tx_clkout,
      tx_clk_reset_n         => tx_clk_reset_n,
      tx_pct_loss_flg        => tx_pct_loss_flg,
      tx_txant_en            => tx_txant_en,
         -- Tx interface data
      tx_diq1_h              => tx_diq1_h,
      tx_diq1_l              => tx_diq1_l,
         -- TX FIFO read ports
      tx_in_pct_rdreq        => tx_in_pct_rdreq,
      tx_in_pct_data         => tx_in_pct_data,
      tx_in_pct_rdempty      => tx_in_pct_rdempty,
      tx_in_pct_rdusedw      => tx_in_pct_rdusedw,

      inst5_pct_hdr_cap      => pct_hdr_cap,
      inst5_smpl_nr_cnt      => smpl_nr_cnt,

      -- RX path
      rx_clk                 => rx_clk
     );
end arch;
