-- ----------------------------------------------------------------------------
-- FILE:          rx_path_top_tb.vhd
-- DESCRIPTION:   Test bech for rx_path_top module
-- DATE:          12:58 2024-05-27
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- NOTES:
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.std_logic_textio.all;

use work.axis_pkg.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity rx_path_top_tb is
end rx_path_top_tb;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture tb_behave of rx_path_top_tb is
   constant clk0_period    : time := 10 ns;
   constant clk1_period    : time := 10 ns;
   constant clk2_period    : time := 10 ns;  
   
   --signals
   signal clk0,clk1,clk2   : std_logic;
   signal reset_n          : std_logic; 
   
   
   signal sys_reset_n            : std_logic;
   signal axis_iqsmpls_aresetn   : std_logic;
   signal axis_iqpacket_aresetn  : std_logic;
   
   signal axis_iqsmpls_tvalid     :  std_logic;
   signal axis_iqsmpls_tready     :  std_logic;
   signal axis_iqsmpls_tdata      :  std_logic_vector(63 downto 0);
   signal axis_iqsmpls_tkeep      :  std_logic_vector( 7 downto 0);
   signal axis_iqsmpls_tlast      :  std_logic;
   
   alias s_axis_tx_tdata_AI is axis_iqsmpls_tdata(63 downto 52);
   alias s_axis_tx_tdata_AQ is axis_iqsmpls_tdata(47 downto 36);
   alias s_axis_tx_tdata_BI is axis_iqsmpls_tdata(31 downto 20);
   alias s_axis_tx_tdata_BQ is axis_iqsmpls_tdata(15 downto  4);
   
   type t_TXIQ_MODE is (MIMO_DDR, SISO_DDR, TXIQ_PULSE);
   signal cfg_txiq_mode : t_TXIQ_MODE;
   signal cfg_ch_en     : std_logic_vector(1 downto 0);
   signal cfg_n_samples : integer;
   signal cfg_pkt_size  : std_logic_vector(15 downto 0);
   
   signal wait_cycles : integer;
   
   signal axis_iqpacket : t_AXI_STREAM(tdata(127 downto 0), tkeep(15 downto 0));
   
   type t_PACKET_ARRAY_256x128 is array (0 to 255) of std_logic_vector(127 downto 0);
   signal tx_packet : t_PACKET_ARRAY_256x128;
   signal tx_packet_valid : std_logic;
   
   type t_PACKET_PAYLOAD_ARRAY_85x384 is array (0 to 84) of std_logic_vector(383 downto 0);
   signal tx_packet_payload_array : t_PACKET_PAYLOAD_ARRAY_85x384;
   
   
   
   type t_PACKET_PAYLOAD_12bARRAY is array (0 to 2719) of std_logic_vector(11 downto 0);
   signal tx_packet_payload_12bit_iq_samples : t_PACKET_PAYLOAD_12bARRAY;
   
   type t_PACKET_PAYLOAD_16bARRAY is array (0 to 2039) of std_logic_vector(15 downto 0);
   signal tx_packet_payload_16bit_iq_samples : t_PACKET_PAYLOAD_16bARRAY;
   
   
   
   
   


   -- Procedure to generate AXIS data
   procedure generate_axis_data(
      signal reset_n       : in     std_logic;
      signal clk           : in     std_logic;
      signal cfg_txiq_mode : in     t_TXIQ_MODE;
      signal cfg_ch_en     : in     std_logic_vector(1 downto 0);
      signal cfg_n_samples : in     integer;
      signal m_axis_tvalid : out    std_logic;
      signal m_axis_tdata  : out    std_logic_vector(63 downto 0);
      signal m_axis_tkeep  : out    std_logic_vector(7 downto 0);
      signal m_axis_tready : in     std_logic;
      signal m_axis_tlast  : out    std_logic
      
   ) is 
   begin
      report "Entering generate_axis_data" severity NOTE;
      wait until rising_edge(clk);
      m_axis_tvalid     <= '0';
      m_axis_tdata      <= (others=>'0');
      m_axis_tkeep      <= (others=>'0');
      if (cfg_ch_en = "11" AND cfg_txiq_mode = MIMO_DDR) OR cfg_txiq_mode = TXIQ_PULSE  then 
         m_axis_tdata(63 downto 52) <= x"000";
         m_axis_tdata(47 downto 36) <= x"001";
         m_axis_tdata(31 downto 20) <= x"002";
         m_axis_tdata(15 downto  4) <= x"003";
         m_axis_tkeep <= x"FF";
      elsif (cfg_ch_en = "01" AND cfg_txiq_mode = MIMO_DDR) OR cfg_txiq_mode = SISO_DDR then 
         m_axis_tdata(63 downto 52) <= x"000";
         m_axis_tdata(47 downto 36) <= x"001";
         m_axis_tdata(31 downto 20) <= x"000";
         m_axis_tdata(15 downto  4) <= x"000";
         m_axis_tkeep <= x"F0";
      elsif (cfg_ch_en = "10" AND cfg_txiq_mode = MIMO_DDR) then 
         m_axis_tdata(63 downto 52) <= x"000";
         m_axis_tdata(47 downto 36) <= x"000";
         m_axis_tdata(31 downto 20) <= x"000";
         m_axis_tdata(15 downto  4) <= x"001";
         m_axis_tkeep <= x"0F";
      else 
         m_axis_tdata(63 downto 52) <= x"000";
         m_axis_tdata(47 downto 36) <= x"000";
         m_axis_tdata(31 downto 20) <= x"000";
         m_axis_tdata(15 downto  4) <= x"000";
         m_axis_tkeep <= x"00";
      end if;
      m_axis_tlast      <= '0';
      
      wait until rising_edge(clk);
      m_axis_tvalid <= '1';
      
      for i in 0 to cfg_n_samples-1 loop
         wait until rising_edge(clk) AND m_axis_tready='1';
         case cfg_txiq_mode is 
            when MIMO_DDR =>
               if cfg_ch_en = "11" then 
                  m_axis_tdata(63 downto 52) <= std_logic_vector(unsigned(m_axis_tdata(63 downto 52))+4); 
                  m_axis_tdata(47 downto 36) <= std_logic_vector(unsigned(m_axis_tdata(47 downto 36))+4);
                  m_axis_tdata(31 downto 20) <= std_logic_vector(unsigned(m_axis_tdata(31 downto 20))+4);
                  m_axis_tdata(15 downto  4) <= std_logic_vector(unsigned(m_axis_tdata(15 downto  4))+4);
               elsif cfg_ch_en = "01" then
                  m_axis_tdata(63 downto 52) <= std_logic_vector(unsigned(m_axis_tdata(63 downto 52))+2); 
                  m_axis_tdata(47 downto 36) <= std_logic_vector(unsigned(m_axis_tdata(47 downto 36))+2);
                  m_axis_tdata(31 downto 20) <= (others=>'0');
                  m_axis_tdata(15 downto  4) <= (others=>'0');
               elsif cfg_ch_en = "10" then 
                  m_axis_tdata(63 downto 52) <= (others=>'0');
                  m_axis_tdata(47 downto 36) <= (others=>'0');
                  m_axis_tdata(31 downto 20) <= std_logic_vector(unsigned(m_axis_tdata(31 downto 20))+2);
                  m_axis_tdata(15 downto  4) <= std_logic_vector(unsigned(m_axis_tdata(15 downto  4))+2);
               else
                  m_axis_tdata(63 downto 52) <= (others=>'0');
                  m_axis_tdata(47 downto 36) <= (others=>'0');
                  m_axis_tdata(31 downto 20) <= (others=>'0');
                  m_axis_tdata(15 downto  4) <= (others=>'0');
               end if;
            when SISO_DDR =>
               m_axis_tdata(63 downto 52) <= std_logic_vector(unsigned(m_axis_tdata(63 downto 52))+2); 
               m_axis_tdata(47 downto 36) <= std_logic_vector(unsigned(m_axis_tdata(47 downto 36))+2);
               m_axis_tdata(31 downto 20) <= (others=>'0');
               m_axis_tdata(15 downto  4) <= (others=>'0');
               
            when TXIQ_PULSE => 
               m_axis_tdata(63 downto 52) <= std_logic_vector(unsigned(m_axis_tdata(63 downto 52))+4); 
               m_axis_tdata(47 downto 36) <= std_logic_vector(unsigned(m_axis_tdata(47 downto 36))+4);
               m_axis_tdata(31 downto 20) <= std_logic_vector(unsigned(m_axis_tdata(31 downto 20))+4);
               m_axis_tdata(15 downto  4) <= std_logic_vector(unsigned(m_axis_tdata(15 downto  4))+4);
               
            when others => 
               m_axis_tdata(63 downto 52) <= (others=>'0');
               m_axis_tdata(47 downto 36) <= (others=>'0');
               m_axis_tdata(31 downto 20) <= (others=>'0');
               m_axis_tdata(15 downto  4) <= (others=>'0');
         end case;      
      end loop;
      wait until rising_edge(clk) AND m_axis_tready='1';
      m_axis_tvalid <= '0';
      report "Exiting generate_axis_data" severity NOTE;
      
   end procedure;
   
   
   -- Procedure to wait synchronous cycles
   procedure wait_sync_cycles (
      signal clk              : in std_logic;
      signal cycles_to_wait   : in integer
   ) is 
   begin 
      report "Entering wait_sync_cycles" severity NOTE;
      report "Cycles to wait: " & integer'image(cycles_to_wait);
      wait until rising_edge(clk);
      for i in 0 to cycles_to_wait loop
         wait until rising_edge(clk);
         report "Waiting" severity NOTE;
      end loop;
      report "Exiting wait_sync_cycles" severity NOTE;
   end procedure;
   
   
   --Procedure to reset whole DUT
   procedure assert_reset (
      signal clk                       : in  std_logic;
      signal sys_reset_n               : out std_logic;
      signal axis_iqsmpls_aresetn      : out std_logic;
      signal axis_iqpacket_aresetn     : out std_logic
   ) is
   begin 
      wait until rising_edge(clk);
      sys_reset_n             <= '0';
      axis_iqsmpls_aresetn    <= '0';
      axis_iqpacket_aresetn   <= '0';
   end procedure;
   
   
   --Procedure to deasert reset for DUT
   procedure deassert_reset (
      signal clk                       : in  std_logic;
      signal sys_reset_n               : out std_logic;
      signal axis_iqsmpls_aresetn      : out std_logic;
      signal axis_iqpacket_aresetn     : out std_logic
   ) is
   begin 
      wait until rising_edge(clk);
      sys_reset_n             <= '1';
      axis_iqsmpls_aresetn    <= '1';
      axis_iqpacket_aresetn   <= '1';
   end procedure;


begin 
  
   clock0: process is
   begin
      clk0 <= '0'; wait for clk0_period/2;
      clk0 <= '1'; wait for clk0_period/2;
   end process clock0;

   clock1: process is
   begin
      clk1 <= '0'; wait for clk1_period/2;
      clk1 <= '1'; wait for clk1_period/2;
   end process clock1;
   
   clock2: process is
   begin
      clk2 <= '0'; wait for clk2_period/2;
      clk2 <= '1'; wait for clk2_period/2;
   end process clock2;
   
   res: process is
   begin
      reset_n <= '0'; wait for 20 ns;
      reset_n <= '1'; wait;
   end process res;
   
   -- Design under test  
   dut_rx_path_top : entity work.rx_path_top
   generic map( 
      G_S_AXIS_IQSMPLS_BUFFER_WORDS    => 16,
      G_M_AXIS_IQPACKET_BUFFER_WORDS   => 512
      )
   port map (
      CLK                     => clk0,
      RESET_N                 => sys_reset_n, 
      -- AXI Stream Slave bus for IQ samples
      S_AXIS_IQSMPLS_ACLK     => clk1,
      S_AXIS_IQSMPLS_ARESETN  => axis_iqsmpls_aresetn,
      S_AXIS_IQSMPLS_TVALID   => axis_iqsmpls_tvalid,
      S_AXIS_IQSMPLS_TREADY   => axis_iqsmpls_tready,
      S_AXIS_IQSMPLS_TDATA    => axis_iqsmpls_tdata,
      S_AXIS_IQSMPLS_TKEEP    => axis_iqsmpls_tkeep,
      S_AXIS_IQSMPLS_TLAST    => axis_iqsmpls_tlast,
      -- AXI Stream Master bus for IQ packets
      M_AXIS_IQPACKET_ACLK    => clk2,
      M_AXIS_IQPACKET_ARESETN => axis_iqpacket_aresetn,
      M_AXIS_IQPACKET_TVALID  => axis_iqpacket.tvalid ,
      M_AXIS_IQPACKET_TREADY  => axis_iqpacket.tready ,
      M_AXIS_IQPACKET_TDATA   => axis_iqpacket.tdata  ,
      M_AXIS_IQPACKET_TKEEP   => axis_iqpacket.tkeep  ,
      M_AXIS_IQPACKET_TLAST   => axis_iqpacket.tlast  ,
      
      CFG_CH_EN               => cfg_ch_en,
      CFG_PKT_SIZE            => cfg_pkt_size,      
      -- Sample Nr.
      SMPL_NR_EN              => '1',
      SMPL_NR_CLR             => '0',
      SMPL_NR_LD              => '0',
      SMPL_NR_IN              => (others=>'0'),
      SMPL_NR_OUT             => open,
      -- Flag control
      TXFLAGS_PCT_LOSS        => '0',
      TXFLAGS_PCT_LOSS_CLR    => '0'
     
   );
   
   tb_proc: process is
   begin
      assert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      
      wait until reset_n='1' AND rising_edge(clk1);
      
      -- ----------------------------------------------------------------------------
      -- Set cfg_txiq_mode and enable A AND B channels
      cfg_txiq_mode  <= MIMO_DDR;
      cfg_ch_en      <= "11";
      cfg_n_samples  <= 680;
      cfg_pkt_size   <= x"0100";
      wait until rising_edge(clk1);
      deassert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      
      -- Generate IQ data
      generate_axis_data (
         reset_n        => reset_n              ,      
         clk            => clk1                 ,
         cfg_txiq_mode  => cfg_txiq_mode        ,
         cfg_ch_en      => cfg_ch_en            ,
         cfg_n_samples  => cfg_n_samples        , 
         m_axis_tvalid  => axis_iqsmpls_tvalid  ,
         m_axis_tdata   => axis_iqsmpls_tdata   ,
         m_axis_tkeep   => axis_iqsmpls_tkeep   ,
         m_axis_tready  => axis_iqsmpls_tready  ,  
         m_axis_tlast   => axis_iqsmpls_tlast     
      );
      
      wait_cycles <= 64;
      wait until rising_edge(clk1);
      wait_sync_cycles(clk1, wait_cycles);
      assert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      deassert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      
      -- ----------------------------------------------------------------------------
      -- Set cfg_txiq_mode and enable A channel
      cfg_txiq_mode  <= MIMO_DDR;
      cfg_ch_en      <= "01";
      cfg_n_samples  <= 1360;
      wait until rising_edge(clk1);
      
      -- Generate IQ data
      generate_axis_data (
         reset_n        => reset_n              ,      
         clk            => clk1                 ,
         cfg_txiq_mode  => cfg_txiq_mode        ,
         cfg_ch_en      => cfg_ch_en            ,
         cfg_n_samples  => cfg_n_samples        , 
         m_axis_tvalid  => axis_iqsmpls_tvalid  ,
         m_axis_tdata   => axis_iqsmpls_tdata   ,
         m_axis_tkeep   => axis_iqsmpls_tkeep   ,
         m_axis_tready  => axis_iqsmpls_tready  ,  
         m_axis_tlast   => axis_iqsmpls_tlast     
      );
      
      wait_cycles <= 64;
      wait until rising_edge(clk1);
      wait_sync_cycles(clk1, wait_cycles);
      assert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      deassert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      
      -- ----------------------------------------------------------------------------
      -- Set cfg_txiq_mode and enable B channel
      -- ----------------------------------------------------------------------------
      cfg_txiq_mode  <= MIMO_DDR;
      cfg_ch_en      <= "10";
      cfg_n_samples  <= 1360;
      wait until rising_edge(clk1);
      
      -- Generate IQ data
      generate_axis_data (
         reset_n        => reset_n              ,      
         clk            => clk1                 ,
         cfg_txiq_mode  => cfg_txiq_mode        ,
         cfg_ch_en      => cfg_ch_en            ,
         cfg_n_samples  => cfg_n_samples        ,
         m_axis_tvalid  => axis_iqsmpls_tvalid  ,
         m_axis_tdata   => axis_iqsmpls_tdata   ,
         m_axis_tkeep   => axis_iqsmpls_tkeep   ,
         m_axis_tready  => axis_iqsmpls_tready  ,  
         m_axis_tlast   => axis_iqsmpls_tlast     
      );
      
      wait_cycles <= 64;
      wait until rising_edge(clk1);
      wait_sync_cycles(clk1, wait_cycles);
      assert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      deassert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      
      -- ----------------------------------------------------------------------------
      -- Set cfg_txiq_mode and enable A channel
      cfg_txiq_mode  <= SISO_DDR;
      cfg_ch_en      <= "01";  -- Does not matter in SISO DDR
      cfg_n_samples  <= 1360;
      wait until rising_edge(clk1);
      
      -- Generate IQ data
      generate_axis_data (
         reset_n        => reset_n              ,      
         clk            => clk1                 ,
         cfg_txiq_mode  => cfg_txiq_mode        ,
         cfg_ch_en      => cfg_ch_en            ,
         cfg_n_samples  => cfg_n_samples        ,
         m_axis_tvalid  => axis_iqsmpls_tvalid  ,
         m_axis_tdata   => axis_iqsmpls_tdata   ,
         m_axis_tkeep   => axis_iqsmpls_tkeep   ,
         m_axis_tready  => axis_iqsmpls_tready  ,  
         m_axis_tlast   => axis_iqsmpls_tlast     
      );
      
      wait_cycles <= 64;
      wait until rising_edge(clk1);
      wait_sync_cycles(clk1, wait_cycles);
      assert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      deassert_reset(clk0, sys_reset_n, axis_iqsmpls_aresetn, axis_iqpacket_aresetn);
      
      wait;
      
   end process tb_proc;
   
   
   -- Capturing whole packet into tx_packet array
   process(clk2, axis_iqpacket_aresetn) 
   variable ptr : integer := 0;
   begin 
      if axis_iqpacket_aresetn = '0' then 
         tx_packet         <= (others=>(others=>'0'));
         tx_packet_valid   <= '0';
      elsif rising_edge(clk2) then
         -- tx_packet array
         if axis_iqpacket.tvalid = '1' AND axis_iqpacket.tready = '1' then
            tx_packet(ptr)         <= axis_iqpacket.tdata;
            
            if axis_iqpacket.tlast = '1' then 
               ptr := 0;
            else 
               ptr := ptr + 1;
            end if;
         else 
            tx_packet <= tx_packet;
         end if;
         
         -- Indicate when whole packet is formed
         if axis_iqpacket.tvalid = '1' AND axis_iqpacket.tready = '1' AND axis_iqpacket.tlast='1' then
            tx_packet_valid <= '1';
         else 
            tx_packet_valid <= '0';
         end if;
         
      end if;
   end process;
   
   
   -- Unpack packed payload to 12bit samples
   process(all)
   begin 
      -- Restructuring packet payload to 85x384 array. We need 3x128 words to unpack samples to even 12bit words 
      for i in 0 to 84 loop
         tx_packet_payload_array(i) <= tx_packet(i*3+1) & tx_packet(i*3+2) &  tx_packet(i*3+3);
      end loop;
      
      -- Restructuring packet payload to 2720x12 array - unpacked 12bit samples
      for i in 0 to 84 loop
         for j in 0 to 31 loop
            tx_packet_payload_12bit_iq_samples(i*32+j) <= tx_packet_payload_array(i)(384-(j*12)-1 downto 384-((j+1)*12));
         end loop;
      end loop;
      
   end process;
   
   
   -- Check received packet payload. 
   process is 
   begin
      axis_iqpacket.tready <= '0';
      wait until rising_edge(clk2) AND axis_iqpacket_aresetn = '1';
      axis_iqpacket.tready <= '1'; 
      wait until rising_edge(clk2) AND tx_packet_valid = '1'; 
      report "Received TX packet" severity NOTE;
      report "Received TX packet SAMPLE_COUNTER: 0x"& to_hstring(tx_packet(0)(127 downto 64)) severity NOTE;
      report "Received TX payload:" severity NOTE;
      for i in 0 to 339 loop
            report   " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+0)) & 
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+1)) &
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+2)) &
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+3)) &
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+4)) &
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+5)) &
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+6)) &
                     " 0x"& to_hstring(tx_packet_payload_12bit_iq_samples(i*8+7)) & LF severity NOTE;
      end loop;
   end process;
   
   --process   
   --begin 
   --   assert (tx_packet_valid = '0')  
   --      report "Simulation halted due to condition violation"
   --      severity failure;
   --end process;

end tb_behave;

