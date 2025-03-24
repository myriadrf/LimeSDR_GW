
-- ----------------------------------------------------------------------------	
-- FILE: 	fft_wrap.vhd
-- DESCRIPTION:	wrapper for an fft module
-- DATE:	October, 2024
-- AUTHOR(s):	Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------	
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity fft_wrap is
  port (
      --input ports 
        CLK       		: in std_logic;
        RESET_N   		: in std_logic;
		--
		S_AXIS_TVALID	: in std_logic;
		S_AXIS_TDATA   	: in std_logic_vector(63 downto 0);
		S_AXIS_TREADY   : out std_logic;
		S_AXIS_TLAST	: in std_logic;
		S_AXIS_TKEEP    : in std_logic_vector(7 downto 0);
		--
		M_AXIS_TDATA  	: out std_logic_vector(63 downto 0);
		M_AXIS_TVALID	: out std_logic;
		M_AXIS_TREADY   : in  std_logic;
		M_AXIS_TLAST	: out std_logic;
		M_AXIS_TKEEP    : out std_logic_vector(7 downto 0);
        -- 
        done            : out std_logic;
        start           : in  std_logic
		
        );
end fft_wrap;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of fft_wrap is

signal wr_state       : std_logic; -- indicates if the fft module's fsm is in the state that accepts data
signal wr_state_valid : std_logic; -- indicates if the fft module's fsm is in any of the states related to accepting data

signal out_real       : std_logic_vector(20 downto 0); -- ful width outputs
signal out_imag       : std_logic_vector(20 downto 0); -- ful width outputs
signal out_real_trunc : std_logic_vector(15 downto 0); -- trucated fft outputs
signal out_imag_trunc : std_logic_vector(15 downto 0); -- trucated fft outputs

signal in_i           : std_logic_vector(11 downto 0);
signal in_q           : std_logic_vector(11 downto 0);
    
attribute DONT_TOUCH : string;
attribute DONT_TOUCH of instfft: label is "TRUE";

signal slv_gnd        : std_logic_vector(12 downto 0);
signal sl_gnd         : std_logic;

component fft is
    port (
        clk         : in std_logic;
        rst         : in std_logic;
        in_i        : in std_logic_vector(11 downto 0);
        in_q        : in std_logic_vector(11 downto 0);
        out_real    : out std_logic_vector(20 downto 0);
        out_imag    : out std_logic_vector(20 downto 0);
        strobe_in   : in std_logic;
        strobe_out  : out std_logic;
        --Connecting done to start makes fft module run continuously
        start       : in  std_logic;
        done        : out std_logic;
        --Window function upload unused
        wf_start    : in std_logic;
        wf_strobe   : in std_logic;   
        wf_real     : in std_logic_vector(12 downto 0);
        wf_imag     : in std_logic_vector(12 downto 0);
        --
        wr_state            : out std_logic;
        wr_state_valid      : out std_logic
        
    );
end component;


begin
    --Avoid errors with VHDL <-> Verilog (type of aggregate cannot be determined without context)
    slv_gnd <= (others => '0');
    sl_gnd  <= '0';

    --Provide A channel data to fft input
    in_i <= S_AXIS_TDATA(15 downto 4);
    in_q <= S_AXIS_TDATA(31 downto 20);

    instfft : fft
    port map(
        clk => clk,
        rst => sl_gnd,
        in_i => in_i,
        in_q => in_q,
        out_real => out_real,
        out_imag => out_imag,
        strobe_in => S_AXIS_TVALID,
        strobe_out => M_AXIS_TVALID,
        --Connecting done to start makes fft module run continuously
        start => done,
        done => done,
        --Window function upload unused
        wf_start => sl_gnd,
        wf_strobe => sl_gnd,
        wf_real => slv_gnd,
        wf_imag => slv_gnd,
        --
        wr_state       => wr_state,
        wr_state_valid => wr_state_valid
    );
    
    --When fft is accepting data, S_AXIS_TREADY is high only when the module is actually ready for data
    --When fft is not accepting data, S_AXIS_TREADY is high to throw away data and avoid clogging up FIFOs
    S_AXIS_TREADY <= wr_state when wr_state_valid ='1' else '1';

   --Truncate outputs separately to allow easy modifications/debugging
   out_real_trunc <= out_real(20 downto 5);
   out_imag_trunc <= out_imag(20 downto 5);

   --Pass through keep signal
   M_AXIS_TKEEP <= S_AXIS_TKEEP;
   --Pass A channel data as B channel data
   M_AXIS_TDATA(63 downto 48) <= S_AXIS_TDATA(31 downto 16);
   M_AXIS_TDATA(47 downto 32) <= S_AXIS_TDATA(15 downto  0); 
   --Pass FFT data as A channel data
   M_AXIS_TDATA(31 downto 16) <= out_real_trunc;
   M_AXIS_TDATA(15 downto 0)  <= out_imag_trunc;
   --TLAST signal is unused in this part of the design
   M_AXIS_TLAST <= '0';


end arch;   



