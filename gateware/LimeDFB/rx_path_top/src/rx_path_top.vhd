-- ----------------------------------------------------------------------------
-- FILE:          rx_path_top.vhd
-- DESCRIPTION:   describe file
-- DATE:          13:24 2024-05-15
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:     v2.0
-- ----------------------------------------------------------------------------

-- TerosHDL module description
--! Top module for packaging IQ samples received from lms7002 module into stream packets.
--!
--! Functionality:
--! - Pack IQ samples from s_axis_iqsmpls stream int Stream packets
--! - Store Stream packets into AXIS buffer.
--!
--!

-- WaveDrom timing diagrams

-- s_axis_tx bus timing (MIMO DDR mode, A and B channels enabled)
--! { signal: [
--! { name: "CLK",  wave: "P........" , period: 2},
--! { name: "RESET_N", wave: "0.1......|...|...." },
--! { name: "CFG_CH_EN[1:0]",  wave: "x.=......|=.=|....", data: ["0x3", "0x3", ""] },
--! { name: "CFG_SMPL_WIDTH[1:0]",  wave: "x.=......|=.=|....", data: ["0x2", "0x2", ""] },
--! { name: "CFG_PKT_SIZE[15:0]",  wave: "x.=......|=.=|....", data: ["0x100", "0x100", ""] },
--! ['S_AXIS_IQSMPLS',
--!  { name: "S_AXIS_IQSMPLS_ACLK",  wave: "P........" , period: 2},
--!  { name: "S_AXIS_IQSMPLS_ARESETN", wave: "0.1......|...|...." },
--!  { name: "S_AXIS_IQSMPLS_TVALID", wave: "0...1....|...|...." },
--!  { name: "S_AXIS_IQSMPLS_TDATA[63:48]",  wave: "x...=.=.=|=.=|....", data: ["BQ(0)", "BQ(n+5)", "", "BQ(679)", "", "BQ(5)"] },
--!  { name: "S_AXIS_IQSMPLS_TDATA[47:32]",  wave: "x...=.=.=|=.=|....", data: ["BI(0)", "BI(n+5)", "", "BI(679)", "", "BI(5)"] },
--!  { name: "S_AXIS_IQSMPLS_TDATA[31:16]",  wave: "x...=.=.=|=.=|....", data: ["AQ(0)", "AQ(n+5)", "", "AQ(679)", "", "AQ(5)"] },
--!  { name: "S_AXIS_IQSMPLS_TDATA[15: 0]",  wave: "x...=.=.=|=.=|....", data: ["AI(0)", "AI(n+5)", "", "AI(679)", "", "AI(5)"] },
--!  { name: "S_AXIS_IQSMPLS_TKEEP[7: 0]",  wave: "x...=...=|=.=|....", data: ["0xFF", "", "0xFF", ""] },
--!  { name: "S_AXIS_IQSMPLS_TREADY", wave: "0...1....|...|...." },
--!  { name: "S_AXIS_IQSMPLS_TLAST", wave: "0........|...|...." },
--! ],
--! { name: "SMPL_NR_OUT[63: 0]",  wave: "x...=.=.=|=.=|....", data: ["0", "n+5", "", "679", ""] },
--! ['M_AXIS_IQPACKET',
--!  { name: "M_AXIS_IQPACKET_ACLK",  wave: "P........" , period: 2},
--!  { name: "M_AXIS_IQPACKET_ARESETN", wave: "0.1..............." },
--!  { name: "M_AXIS_IQPACKET_TVALID", wave: "0.......1........." },
--!  { name: "M_AXIS_IQPACKET_TDATA[127:0]",  wave: "x.......=.=.=|=.=.", data: ["HDR", "PLD(0)", "", "PLD(254)", "",] },
--!  { name: "M_AXIS_IQPACKET_TKEEP[7: 0]",  wave: "x.......=...=|=...", data: ["0xFF", "", "0xFF"] },
--!  { name: "M_AXIS_IQPACKET_TREADY", wave: "0...1............." },
--!  { name: "M_AXIS_IQPACKETS_TLAST", wave: "0............|1.0." },
--! ]
--! ],
--!
--! "config" : { "hscale" : 1 },
--!  head:{
--!     text: ['tspan',
--!           ['tspan', {'font-weight':'bold'}, 'rx_path_top module timing (MIMO DDR mode, A and B channels enabled, 12b samples, 4096B packet size)']],
--!     tick:0,
--!     every:2
--!   }}

-- ----------------------------------------------------------------------------
-- NOTES:
-- If S_AXIS_IQSMPLS_BUFFER or M_AXIS_IQPACKET_BUFFER are bypassed, respective bus
-- become synchronous to CLK clock domain
-- ----------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;
   use work.axis_pkg.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------

entity RX_PATH_TOP is
   generic (
      G_S_AXIS_IQSMPLS_BUFFER_WORDS    : integer := 16;
      G_M_AXIS_IQPACKET_BUFFER_WORDS   : integer := 512
   );
   port (
      CLK                     : in    std_logic;                     --! Sys clock
      RESET_N                 : in    std_logic;                     --! Sys active low reset
      --! @virtualbus S_AXIS_IQSMPLS @dir in AXI Stream Slave bus for IQ samples
      S_AXIS_IQSMPLS_ACLK     : in    std_logic;                     --!
      S_AXIS_IQSMPLS_ARESETN  : in    std_logic;                     --!
      S_AXIS_IQSMPLS_TVALID   : in    std_logic;                     --!
      S_AXIS_IQSMPLS_TREADY   : in    std_logic;                     --! Indicates when FIFO comes out of reset. Slave can allways accept data
      S_AXIS_IQSMPLS_TDATA    : in    std_logic_vector(127 downto 0); --!
      S_AXIS_IQSMPLS_TLAST    : in    std_logic;                     --! @end
      -- Configuration ports
      --! @virtualbus CFG @dir in Configuration signals
      CFG_CH_EN               : in    std_logic_vector(1 downto 0);  --! Channel enable. 0- Channel Disabled, 1-Channel Enabled
      CFG_SMPL_WIDTH          : in    std_logic_vector(1 downto 0);  --! Sample width selection. "10"-12bit, "01"-14bit, "00"-16bit;
      --! @virtualbus SMPL_NR_IN @dir in Sample Nr. input
      bp_sample_nr_counter    : out   std_logic_vector(63 downto 0);
      SMPL_NR_CLR             : in    std_logic;                     --! Cleas sample number
      SMPL_NR_INCR            : in    std_logic;
      SMPL_NR_LD              : in    std_logic;                     --! Load sample number
      SMPL_NR_IN              : in    std_logic_vector(63 downto 0); --! Sample Nr. when loading @end
      SMPL_NR_OUT             : out   std_logic_vector(63 downto 0) --! Sample Nr. output, synchronous to S_AXIS_IQSMPLS_ACLK
   );
end entity RX_PATH_TOP;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of RX_PATH_TOP is

   -- declare signals,  components here

   -- CHECKME: Unconstrainted Records do not seems supported by Quartus...

   signal axis_iqsmpls_fifo  : t_AXI_STREAM_128b;

   signal sample_nr_counter            : unsigned(63 downto 0);
   signal bitpacked_sample_nr_counter  : unsigned(63 downto 0);
   
begin
	bp_sample_nr_counter <= std_logic_vector(bitpacked_sample_nr_counter);

   -- ----------------------------------------------------------------------------
   -- Sample counter dedicated for TX stream synchronization with RX
   -- ----------------------------------------------------------------------------
   SAMPLE_CNT_PROC : process (S_AXIS_IQSMPLS_ACLK, S_AXIS_IQSMPLS_ARESETN) is
   begin

      if (S_AXIS_IQSMPLS_ARESETN='0') then
         sample_nr_counter <= (others => '0');
      elsif (rising_edge(S_AXIS_IQSMPLS_ACLK)) then
         if (SMPL_NR_CLR='1') then
            sample_nr_counter <= (others => '0');
         elsif (SMPL_NR_LD='1') then
            sample_nr_counter <= unsigned(SMPL_NR_IN);
         elsif (SMPL_NR_INCR ='1') then
            sample_nr_counter <= sample_nr_counter + 1;
         end if;
      end if;

   end process SAMPLE_CNT_PROC;

   axis_iqsmpls_fifo.tvalid <= S_AXIS_IQSMPLS_TVALID;
   axis_iqsmpls_fifo.tready <= S_AXIS_IQSMPLS_TREADY;
   axis_iqsmpls_fifo.tdata  <= S_AXIS_IQSMPLS_TDATA;
   axis_iqsmpls_fifo.tkeep  <= (others => '1');
   axis_iqsmpls_fifo.tlast  <= S_AXIS_IQSMPLS_TLAST;


   -- ----------------------------------------------------------------------------
   -- Sample counter dedicated for forming RX packets
   -- ----------------------------------------------------------------------------
   PACKET_SAMPLE_CNT_PROC : process (CLK, RESET_N) is
   begin

      if (RESET_N='0') then
         bitpacked_sample_nr_counter <= (others => '0');
      elsif (rising_edge(CLK)) then
         if (SMPL_NR_CLR='1') then
            bitpacked_sample_nr_counter <= (others => '0');
         elsif (SMPL_NR_LD='1') then
            bitpacked_sample_nr_counter <= unsigned(SMPL_NR_IN);
         -- elsif (axis_iqsmpls_fifo.tvalid='1' and axis_iqsmpls_fifo.tlast='1' and axis_iqsmpls_fifo.tready ='1') then
         elsif (axis_iqsmpls_fifo.tvalid='1' and axis_iqsmpls_fifo.tready ='1') then
            if (CFG_SMPL_WIDTH = "10") then -- 12 bit format
               if (axis_iqsmpls_fifo.tlast='1') then -- 12 bit format needs tlatst signal for correct count
                  -- If both channels are enabled in one frame we have packed 8 samples for each A and B channel
                  if (CFG_CH_EN = "11") then
                     bitpacked_sample_nr_counter <= bitpacked_sample_nr_counter + 8;
                  else
                     bitpacked_sample_nr_counter <= bitpacked_sample_nr_counter + 16;
                  end if;
               end if;
            else -- assume 16 bit format
               if (CFG_CH_EN = "11") then
                  bitpacked_sample_nr_counter <= bitpacked_sample_nr_counter + 2;
               else
                  bitpacked_sample_nr_counter <= bitpacked_sample_nr_counter + 4;
               end if;
            end if;
         end if;
      end if;

   end process PACKET_SAMPLE_CNT_PROC;
   
   SMPL_NR_OUT <= std_logic_vector(sample_nr_counter);

end architecture ARCH;
