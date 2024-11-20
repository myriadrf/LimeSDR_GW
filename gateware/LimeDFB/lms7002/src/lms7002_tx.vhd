-- ----------------------------------------------------------------------------
-- FILE:          lms7002_tx.vhd
-- DESCRIPTION:   Transmit interface for LMS7002 IC
-- DATE:          13:44 2024-02-01
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- NOTES:
--
-- ----------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------

entity LMS7002_TX is
   generic (
      G_IQ_WIDTH           : integer   := 12
   );
   port (
      CLK               : in    std_logic;
      RESET_N           : in    std_logic;
      -- Mode settings
      MODE              : in    std_logic;                             -- JESD207: 1; TRXIQ: 0
      TRXIQPULSE        : in    std_logic;                             -- trxiqpulse on: 1; trxiqpulse off: 0
      DDR_EN            : in    std_logic;                             -- DDR: 1; SDR: 0
      MIMO_EN           : in    std_logic;                             -- SISO: 1; MIMO: 0
      CH_EN             : in    std_logic_vector(1 downto 0);          -- "01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B.
      FIDM              : in    std_logic;                             -- Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
      -- Tx interface data
      DIQ_H             : out   std_logic_vector(G_IQ_WIDTH downto 0); --! fsync + DIQ
      DIQ_L             : out   std_logic_vector(G_IQ_WIDTH downto 0); --! fsync + DIQ
      --! @virtualbus s_axis_tx @dir in Transmit AXIS bus
      S_AXIS_ARESET_N   : in    std_logic;                             --! AXIS reset
      S_AXIS_ACLK       : in    std_logic;                             --! AXIS clock
      S_AXIS_TVALID     : in    std_logic;                             --! AXIS valid transfer
      S_AXIS_TDATA      : in    std_logic_vector(63 downto 0);         --! AXIS data
      S_AXIS_TREADY     : out   std_logic;                             --! AXIS ready
      S_AXIS_TLAST      : in    std_logic                              --! AXIS last packet boundary @end
   );
end entity LMS7002_TX;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of LMS7002_TX is

   -- declare signals,  components here

   signal s_axis_tready_reg   : std_logic;

   type T_DIQ_SHIFT_REG_TYPE is array (0 to 1) of std_logic_vector(15 downto 0);

   signal diq_l_reg           : T_DIQ_SHIFT_REG_TYPE;
   signal diq_h_reg           : T_DIQ_SHIFT_REG_TYPE;

   alias a_axis_fifo_ai       is s_axis_tdata(63 downto 48);
   alias a_axis_fifo_aq       is s_axis_tdata(47 downto 32);
   alias a_axis_fifo_bi       is s_axis_tdata(31 downto 16);
   alias a_axis_fifo_bq       is s_axis_tdata(15 downto  0);

   signal fsync_l             : std_logic_vector(3 downto 0);
   signal fsync_h             : std_logic_vector(3 downto 0);

   signal mux_fsync_l         : std_logic_vector(3 downto 0);
   signal mux_fsync_h         : std_logic_vector(3 downto 0);

   signal int_fsync_l         : std_logic_vector(3 downto 0);
   signal int_fsync_h         : std_logic_vector(3 downto 0);

begin

   -- According to axi stream protocol:
   -- A Receiver is permitted to wait for TVALID to be asserted before asserting TREADY. It is permitted that a
   -- Receiver asserts and deasserts TREADY without TVALID being asserted.
   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         s_axis_tready_reg <= '0';
      elsif rising_edge(CLK) then
         if (S_AXIS_TVALID = '1') then
            if (DDR_EN = '1' and MIMO_EN = '0') then
               s_axis_tready_reg <= '1';
            else
               s_axis_tready_reg <= NOT s_axis_tready_reg;
            end if;
         else
            s_axis_tready_reg <= '0';
         end if;
      end if;

   end process;

   -- diq_l and diq_h shift registers
   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         diq_h_reg <= (others => (others => '0'));
      elsif rising_edge(CLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID ='1') then
            diq_h_reg(0) <= a_axis_fifo_ai;
            diq_h_reg(1) <= a_axis_fifo_bi;
         else
            diq_h_reg(0) <= diq_h_reg(1);
            diq_h_reg(1) <= (others => '0');
         end if;
      end if;

   end process;

   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         diq_l_reg <= (others => (others => '0'));
      elsif rising_edge(CLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID ='1') then
            diq_l_reg(0) <= a_axis_fifo_aq;
            diq_l_reg(1) <= a_axis_fifo_bq;
         else
            diq_l_reg(0) <= diq_l_reg(1);
            diq_l_reg(1) <= (others => '0');
         end if;
      end if;

   end process;

   -- ----------------------------------------------------------------------------
   -- Muxes for fsync signal
   -- ----------------------------------------------------------------------------
   mux_fsync_h <= "1111" when (MIMO_EN ='0' and DDR_EN ='1' and TRXIQPULSE='0') else
                  "0101";

   mux_fsync_l <= "0000" when (MIMO_EN ='0' and DDR_EN = '1') or TRXIQPULSE='1' else
                  "0101";

   int_fsync_h <= (not mux_fsync_h(3) & not mux_fsync_h(2) & not mux_fsync_h(1) & not mux_fsync_h(0)) when FIDM = '0' else
                  mux_fsync_h;

   int_fsync_l <= (not mux_fsync_l(3) & not mux_fsync_l(2) & not mux_fsync_l(1) & not mux_fsync_l(0)) when FIDM = '0' else
                  mux_fsync_l;

   -- fsync register has more taps than diq shift registers because
   -- when we stop receiving valid diq samples from s_axis bus we want to
   -- transmit one more frame with zeros on DIQ bus.
   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         fsync_l <= (others => '0');
         fsync_h <= (others => '0');
      elsif rising_edge(CLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID ='1') then
            fsync_l <= int_fsync_l;
            fsync_h <= int_fsync_h;
         else
            fsync_l <= '0' & fsync_l(3 downto 1);
            fsync_h <= '0' & fsync_h(3 downto 1);
         end if;
      end if;

   end process;

   -- ----------------------------------------------------------------------------
   -- Output ports
   -- ----------------------------------------------------------------------------

   DIQ_H <= fsync_h(0) & diq_h_reg(0)(15 downto 4);
   DIQ_L <= fsync_l(0) & diq_l_reg(0)(15 downto 4);

   S_AXIS_TREADY <= s_axis_tready_reg;

end architecture ARCH;


