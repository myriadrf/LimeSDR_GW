-- ----------------------------------------------------------------------------	
-- FILE:    diq2fifo.vhd
-- DESCRIPTION:   Writes DIQ data to FIFO, FIFO word size = 4  DIQ samples 
-- DATE: Jan 13, 2016
-- AUTHOR(s): Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------	
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity diq2fifo is
   generic( 
      dev_family           : string := "Cyclone IV E";
      iq_width             : integer := 12;
      invert_input_clocks  : string := "ON"
      );
   port (
      clk            : in std_logic;
      reset_n        : in std_logic;
      test_ptrn_en   : in std_logic;
      --Mode settings
      mode           : in std_logic; -- JESD207: 1; TRXIQ: 0
      trxiqpulse     : in std_logic; -- trxiqpulse on: 1; trxiqpulse off: 0
      ddr_en         : in std_logic; -- DDR: 1; SDR: 0
      mimo_en        : in std_logic; -- SISO: 1; MIMO: 0
      ch_en          : in std_logic_vector(1 downto 0); --"01" - Ch. A, "10" - Ch. B, "11" - Ch. A and Ch. B. 
      fidm           : in std_logic; -- External Frame ID mode. Frame start at fsync = 0, when 0. Frame start at fsync = 1, when 1.
      --Rx interface data 
      rx_diq2_h      : in std_logic_vector(iq_width downto 0);
      rx_diq2_l      : in std_logic_vector(iq_width downto 0);
      -- AXI Stream master port.
      m_axis_tdata   : out std_logic_vector(63 downto 0);
      m_axis_tkeep   : out std_logic_vector(7 downto 0);
      m_axis_tvalid  : out std_logic;
      m_axis_tlast   : out std_logic;
      m_axis_tready  : in  std_logic;
      --sample compare
      smpl_cmp_start : in std_logic;
      -- sample counter enable
      smpl_cnt_en    : out std_logic

        );
end diq2fifo;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of diq2fifo is
--declare signals,  components here
signal inst0_diq_out_h  : std_logic_vector (iq_width downto 0); 
signal inst0_diq_out_l  : std_logic_vector (iq_width downto 0);
signal inst0_reset_n    : std_logic; 

signal inst2_data_h     : std_logic_vector (iq_width downto 0);
signal inst2_data_l     : std_logic_vector (iq_width downto 0);

signal mux0_diq_h       : std_logic_vector (iq_width downto 0); 
signal mux0_diq_l       : std_logic_vector (iq_width downto 0);

signal mux0_diq_h_reg   : std_logic_vector (iq_width downto 0); 
signal mux0_diq_l_reg   : std_logic_vector (iq_width downto 0);

signal smpl_cnt_en_reg  : std_logic;

signal tdata            : std_logic_vector(63 downto 0);
signal tkeep            : std_logic_vector(7 downto 0);
signal chaA             : std_logic_vector(iq_width*4-1 downto 0);
signal tvalid           : std_logic;
signal tvalid_d         : std_logic;
signal tlast            : std_logic;
  
begin

inst0_reset_n <= reset_n when smpl_cmp_start = '0' else '1';

   inst0_diq_out_h <= rx_diq2_h;
   inst0_diq_out_l <= rx_diq2_l;
        
   process(clk, inst0_reset_n)
   begin 
      if inst0_reset_n = '0' then 
         smpl_cnt_en_reg <= '0';
      elsif rising_edge(clk) then 
         if mimo_en = '0' AND ddr_en = '1' then 
            smpl_cnt_en_reg <= '1';
         else 
            smpl_cnt_en_reg <= not smpl_cnt_en_reg;
         end if;
      end if;
   end process;

  inst1_rxiq: entity work.lms7002_rx
   generic map (
      g_IQ_WIDTH           => iq_width,
      g_M_AXIS_FIFO_WORDS  => 16
   )
   port map (
      clk         => clk,
      reset_n     => reset_n,
      --Mode settings
      mode        => mode,
      trxiqpulse  => trxiqpulse,
      ddr_en      => ddr_en,
      mimo_en     => mimo_en,
      ch_en       => ch_en, 
      fidm        => fidm,
      --Tx interface data
      diq_h       => mux0_diq_h_reg,
      diq_l       => mux0_diq_l_reg,
      --! @virtualbus s_axis_tx @dir in Transmit AXIS bus
      m_axis_areset_n  => '0',
      m_axis_aclk      => '0',
      m_axis_tvalid    => tvalid,
      m_axis_tdata     => tdata,
      m_axis_tkeep     => tkeep,
      m_axis_tready    => m_axis_tready,
      m_axis_tlast     => tlast
   );

   -- to keep lms7002_rx unchanged but to honour channel select.
   -- ----------------------------------------------------------
   --process(clk, reset_n)
   -- begin
   --     if reset_n = '0' then
   --         chaA <= (others => '0');
   --         tvalid_d <= '0';
   --     elsif rising_edge(clk) then
   --         if tvalid = '1' then
   --             tvalid_d <= not tvalid_d;
   --             chaA <= chaA(23 downto 0) &
   --                    tdata(31 downto 20) &
   --                    tdata(15 downto  4);
   --             m_axis_tvalid <= tvalid_d;
   --             m_axis_tlast  <= tlast;
   --             m_axis_tkeep  <= tkeep;
   --         end if;
   --     end if;
   -- end process;
   m_axis_tdata <= tdata;
   m_axis_tvalid <= tvalid;
   m_axis_tkeep  <= tkeep;
   m_axis_tlast  <= tlast;
  
int2_test_data_dd : entity work.test_data_dd
port map(

   clk            => clk,
   reset_n        => reset_n,
   fr_start       => fidm,
   mimo_en        => mimo_en,  
   data_h         => inst2_data_h,
   data_l         => inst2_data_l

);


mux0_diq_h <= inst0_diq_out_h when test_ptrn_en = '0' else inst2_data_h;
mux0_diq_l <= inst0_diq_out_l when test_ptrn_en = '0' else inst2_data_l;	


process(clk, reset_n)
begin 
   if reset_n = '0' then 
      mux0_diq_h_reg <= (others=>'0');
      mux0_diq_l_reg <= (others=>'0');
   elsif (clk'event AND clk='1') then
      mux0_diq_h_reg <= mux0_diq_h;
      mux0_diq_l_reg <= mux0_diq_l;
   end if;
end process;

-- ----------------------------------------------------------------------------
-- Output ports
-- ----------------------------------------------------------------------------   
  
   smpl_cnt_en <= smpl_cnt_en_reg;
 
end arch;   





