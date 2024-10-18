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
      TX_OUT_PCT_DATA_W       : integer := 64;

      -- RX parameters
      RX_IQ_WIDTH             : integer := 12;
      RX_INVERT_INPUT_CLOCKS  : string := "ON";
      RX_SMPL_BUFF_RDUSEDW_W  : integer := 11; --bus width in bits
      RX_PCT_BUFF_WRUSEDW_W   : integer := 12  --bus width in bits
   );
   port (
      -- Configuration memory ports
      --from_fpgacfg            : in     t_FROM_FPGACFG;
      phase_reg_sel       : in std_logic_vector(15 downto 0);
      clk_ind             : in std_logic_vector(4 downto 0);
      cnt_ind             : in std_logic_vector(4 downto 0);
      load_phase_reg      : in std_logic;
      drct_clk_en         : in std_logic_vector(15 downto 0);
      ch_en               : in std_logic_vector(15 downto 0);
      smpl_width          : in std_logic_vector(1 downto 0);
      mode                : in std_logic;
      ddr_en              : in std_logic;
      trxiq_pulse         : in std_logic;
      mimo_int_en         : in std_logic;
      synch_dis           : in std_logic;
      synch_mode          : in std_logic;
      smpl_nr_clr         : in std_logic;
      txpct_loss_clr      : in std_logic;
      rx_en               : in std_logic;
      tx_en               : in std_logic;
      rx_ptrn_en          : in std_logic;
      tx_ptrn_en          : in std_logic;
      tx_cnt_en           : in std_logic;
      wfm_ch_en           : in std_logic_vector(15 downto 0);
      wfm_play            : in std_logic;
      wfm_load            : in std_logic;
      wfm_smpl_width      : in std_logic_vector(1 downto 0);
      SPI_SS              : in std_logic_vector(15 downto 0);
      LMS1_SS             : in std_logic;
      LMS1_RESET          : in std_logic;
      LMS1_CORE_LDO_EN    : in std_logic;
      LMS1_TXNRX1         : in std_logic;
      LMS1_TXNRX2         : in std_logic;
      LMS1_TXEN           : in std_logic;
      LMS1_RXEN           : in std_logic;
      GPIO                : in std_logic_vector(15 downto 0);
      FPGA_LED1_CTRL      : in std_logic_vector(2 downto 0);
      FPGA_LED2_CTRL      : in std_logic_vector(2 downto 0);
      FX3_LED_CTRL        : in std_logic_vector(2 downto 0);
      CLK_ENA             : in std_logic_vector(3 downto 0);
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

      ---- RX path
      rx_clk              : in  std_logic;
      rx_clk_reset_n      : in  std_logic;
      --   --Rx interface data
      rx_diq2_h           : in  std_logic_vector(RX_IQ_WIDTH downto 0);
      rx_diq2_l           : in  std_logic_vector(RX_IQ_WIDTH downto 0);
      --   --Packet fifo ports
      rx_pct_fifo_aclrn_req : out std_logic;
      rx_pct_fifo_wusedw  : in  std_logic_vector(RX_PCT_BUFF_WRUSEDW_W-1 downto 0);
      rx_pct_fifo_wrreq   : out std_logic;
      rx_pct_fifo_wdata   : out std_logic_vector(63 downto 0);
         --sample compare
      rx_smpl_cmp_start   : in  std_logic;
      rx_smpl_cmp_length  : in  std_logic_vector(15 downto 0);
      rx_smpl_cmp_done    : out std_logic;
      rx_smpl_cmp_err     : out std_logic
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

   from_fpgacfg.phase_reg_sel     <= phase_reg_sel;
   from_fpgacfg.clk_ind           <= clk_ind;
   from_fpgacfg.cnt_ind           <= cnt_ind;
   from_fpgacfg.load_phase_reg    <= load_phase_reg;
   from_fpgacfg.drct_clk_en       <= drct_clk_en;
   from_fpgacfg.ch_en             <= ch_en;
   from_fpgacfg.smpl_width        <= smpl_width;
   from_fpgacfg.mode              <= mode;
   from_fpgacfg.ddr_en            <= ddr_en;
   from_fpgacfg.trxiq_pulse       <= trxiq_pulse;
   from_fpgacfg.mimo_int_en       <= mimo_int_en;
   from_fpgacfg.synch_dis         <= synch_dis;
   from_fpgacfg.synch_mode        <= synch_mode;
   from_fpgacfg.smpl_nr_clr       <= smpl_nr_clr;
   from_fpgacfg.txpct_loss_clr    <= txpct_loss_clr;
   from_fpgacfg.rx_en             <= rx_en;
   from_fpgacfg.tx_en             <= tx_en;
   from_fpgacfg.rx_ptrn_en        <= rx_ptrn_en;
   from_fpgacfg.tx_ptrn_en        <= tx_ptrn_en;
   from_fpgacfg.tx_cnt_en         <= tx_cnt_en;
   from_fpgacfg.wfm_ch_en         <= wfm_ch_en;
   from_fpgacfg.wfm_play          <= wfm_play;
   from_fpgacfg.wfm_load          <= wfm_load;
   from_fpgacfg.wfm_smpl_width    <= wfm_smpl_width;
   from_fpgacfg.SPI_SS            <= SPI_SS;
   from_fpgacfg.LMS1_SS           <= LMS1_SS;
   from_fpgacfg.LMS1_RESET        <= LMS1_RESET;
   from_fpgacfg.LMS1_CORE_LDO_EN  <= LMS1_CORE_LDO_EN;
   from_fpgacfg.LMS1_TXNRX1       <= LMS1_TXNRX1;
   from_fpgacfg.LMS1_TXNRX2       <= LMS1_TXNRX2;
   from_fpgacfg.LMS1_TXEN         <= LMS1_TXEN;
   from_fpgacfg.LMS1_RXEN         <= LMS1_RXEN;
   from_fpgacfg.GPIO              <= GPIO;
   from_fpgacfg.FPGA_LED1_CTRL    <= FPGA_LED1_CTRL;
   from_fpgacfg.FPGA_LED2_CTRL    <= FPGA_LED2_CTRL;
   from_fpgacfg.FX3_LED_CTRL      <= FX3_LED_CTRL;
   from_fpgacfg.CLK_ENA           <= CLK_ENA;
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
      TX_OUT_PCT_DATA_W      => TX_OUT_PCT_DATA_W,

      -- RX parameters
      RX_IQ_WIDTH            => RX_IQ_WIDTH,
      RX_INVERT_INPUT_CLOCKS => RX_INVERT_INPUT_CLOCKS,
      RX_SMPL_BUFF_RDUSEDW_W => RX_SMPL_BUFF_RDUSEDW_W,
      RX_PCT_BUFF_WRUSEDW_W  => RX_PCT_BUFF_WRUSEDW_W

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

      -- RX path
      rx_clk                 => rx_clk,
      rx_clk_reset_n         => rx_clk_reset_n,
         --Rx interface data
      rx_diq2_h              => rx_diq2_h,
      rx_diq2_l              => rx_diq2_l,
         --Packet fifo ports
      rx_pct_fifo_aclrn_req  => rx_pct_fifo_aclrn_req,
      rx_pct_fifo_wusedw     => rx_pct_fifo_wusedw,
      rx_pct_fifo_wrreq      => rx_pct_fifo_wrreq,
      rx_pct_fifo_wdata      => rx_pct_fifo_wdata,
         --sample compare
      rx_smpl_cmp_start      => rx_smpl_cmp_start,
      rx_smpl_cmp_length     => rx_smpl_cmp_length,
      rx_smpl_cmp_done       => rx_smpl_cmp_done,
      rx_smpl_cmp_err        => rx_smpl_cmp_err
     );
end arch;
