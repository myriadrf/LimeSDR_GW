-- ----------------------------------------------------------------------------
-- FILE:          lms7002_rx.vhd
-- DESCRIPTION:   Receive interface for LMS7002 IC
-- DATE:          09:51 2024-02-12
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
entity lms7002_rx is
   generic(
      g_IQ_WIDTH           : integer   := 12;
      g_M_AXIS_FIFO_WORDS  : integer   := 16
   );
   port (
      clk               : in  std_logic;
      reset_n           : in  std_logic;
      --Mode settings
      mode              : in  std_logic; -- JESD207: 1; TRXIQ: 0
      trxiqpulse        : in  std_logic; -- trxiqpulse on: 1; trxiqpulse off: 0
      ddr_en            : in  std_logic; -- DDR: 1; SDR: 0
      mimo_en           : in  std_logic; -- SISO: 1; MIMO: 0
      ch_en             : in  std_logic_vector(1 downto 0); --"01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B. 
      fidm              : in  std_logic; -- Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.                 
      --Tx interface data 
      diq_h             : in  std_logic_vector(g_IQ_WIDTH downto 0);  -- fsync + DIQ
      diq_l             : in  std_logic_vector(g_IQ_WIDTH downto 0);  -- fsync + DIQ
      --! @virtualbus s_axis_tx @dir in Transmit AXIS bus
      m_axis_areset_n   : in  std_logic;  --! AXIS reset
      m_axis_aclk       : in  std_logic;  --! AXIS clock
      m_axis_tvalid     : out std_logic;  --! AXIS valid transfer
      m_axis_tdata      : out std_logic_vector(63 downto 0);   --! AXIS data
      m_axis_tkeep      : out std_logic_vector(7 downto 0);    --! AXIS byte qualifier 
      m_axis_tready     : in  std_logic;  --! AXIS ready 
      m_axis_tlast      : out std_logic   --! AXIS last packet boundary @end
   );
end lms7002_rx;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of lms7002_rx is
--declare signals,  components here
signal diq_h_reg : std_logic_vector(g_IQ_WIDTH downto 0);  -- DIQ + fsync
signal diq_l_reg : std_logic_vector(g_IQ_WIDTH downto 0);  -- DIQ + fsync

signal ai, aq, bi, bq : std_logic_vector(g_IQ_WIDTH-1 downto 0);
signal ai_reg, aq_reg : std_logic_vector(g_IQ_WIDTH-1 downto 0);
signal frame_valid    : std_logic;

type t_TXIQ_MODE is (MIMO_DDR, SISO_DDR, TXIQ_PULSE, SISO_SDR);
signal txiq_mode : t_TXIQ_MODE;

signal m_axis_tkeep_async : std_logic_vector(m_axis_tkeep'LENGTH -1 downto 0);
   
begin

   
   -- Internal signal to know which mode is set
   process(all)
   begin 
      if trxiqpulse = '1' then
         txiq_mode <= TXIQ_PULSE;
      else 
         if mimo_en = '1' AND ddr_en = '1' then 
            txiq_mode <= MIMO_DDR;
         elsif mimo_en = '0' AND ddr_en = '1' then
            txiq_mode <= SISO_DDR;
         else 
            txiq_mode <= SISO_SDR;
         end if;
      end if;
   end process;


   -- Input register
   process(clk)
   begin 
      if rising_edge(clk) then 
         diq_h_reg <= diq_h;
         diq_l_reg <= diq_l;
      end if;
   end process;
   
   -- AI and AQ capture
   process(clk)
   begin 
      if rising_edge(clk) then 
         if diq_h(diq_h'LEFT) = '0' AND diq_l(diq_l'LEFT)='0' AND txiq_mode = MIMO_DDR then
            ai <= diq_l(g_IQ_WIDTH-1 downto 0);
            aq <= diq_h(g_IQ_WIDTH-1 downto 0);
         elsif diq_h(diq_h'LEFT) = '1' AND diq_l(diq_l'LEFT)='0' AND (txiq_mode = SISO_DDR OR txiq_mode = TXIQ_PULSE) then
            ai <= diq_l(g_IQ_WIDTH-1 downto 0);
            aq <= diq_h(g_IQ_WIDTH-1 downto 0);
         else 
            ai <= ai;
            aq <= aq;
         end if;
      end if;
   end process;
   
   -- BI and BQ capture
   process(clk)
   begin 
      if rising_edge(clk) then 
         if diq_h(diq_h'LEFT) = '1' AND diq_l(diq_l'LEFT)='1' then
            bi <= diq_l(g_IQ_WIDTH-1 downto 0);
            bq <= diq_h(g_IQ_WIDTH-1 downto 0);
         else 
            bi <= bi;
            bq <= bq;
         end if;
      end if;
   end process;
   
   
   --Internal IQ frame valid signal. For e.g one frame is AI AQ BI BQ samples in MIMO DDR mode 
   process(clk, reset_n)
   begin 
      if reset_n = '0' then 
         frame_valid <= '0';
      elsif rising_edge(clk) then 
         if diq_h(diq_h'LEFT) = '1' AND diq_l(diq_l'LEFT)='1' AND txiq_mode = MIMO_DDR then 
            frame_valid <= '1';
         elsif diq_h(diq_h'LEFT) = '1' AND diq_l(diq_l'LEFT)='0' AND txiq_mode = SISO_DDR then
            frame_valid <= '1';
         elsif diq_h_reg(diq_h'LEFT) = '1' AND diq_l_reg(diq_l'LEFT)='0' AND diq_h(diq_h'LEFT) = '1' AND diq_l(diq_l'LEFT)='1' AND txiq_mode = TXIQ_PULSE then
            frame_valid <= '1';
         else 
            frame_valid <= '0';
         end if;
      end if;
   end process;
   
   process(all)
   begin 
      case txiq_mode is 
         when MIMO_DDR => 
            if ch_en = "01" then 
               m_axis_tkeep_async <= "00001111";
            elsif ch_en = "10" then 
               m_axis_tkeep_async <= "11110000";
            else 
               m_axis_tkeep_async <= (others=>'1');
            end if;
         when TXIQ_PULSE => 
            m_axis_tkeep_async <= (others=>'1');
         when SISO_DDR => 
            m_axis_tkeep_async <= "00001111";
         when others =>
            m_axis_tkeep_async <= (others=>'0');
      end case;
   end process;
   
   
-- ----------------------------------------------------------------------------
-- Output ports
-- ----------------------------------------------------------------------------
   m_axis_tvalid              <= frame_valid;
   --m_axis_tdata(63 downto 48) <= ai & "0000";
   --m_axis_tdata(47 downto 32) <= aq & "0000";
   --m_axis_tdata(31 downto 16) <= bi & "0000";
   --m_axis_tdata(15 downto  0) <= bq & "0000";
   
   m_axis_tdata(63 downto 48) <= bq & "0000";
   m_axis_tdata(47 downto 32) <= bi & "0000";
   m_axis_tdata(31 downto 16) <= aq & "0000";
   m_axis_tdata(15 downto  0) <= ai & "0000";
   m_axis_tkeep               <= m_axis_tkeep_async;
   m_axis_tlast               <= '0';
   
   
end arch;   


