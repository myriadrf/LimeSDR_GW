-- ----------------------------------------------------------------------------
-- FILE:     bit_pack.vhd
-- DESCRIPTION:    packs data from 12 or 14 bit samples to 16 bit data
-- DATE:    Nov 15, 2016
-- AUTHOR(s):    Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity bit_pack is
  generic (
        G_PORT_WIDTH    : integer := 64;
        G_DISABLE_14BIT : boolean := false
        );
  port (
        --input ports 
        clk             : in std_logic;
        reset_n         : in std_logic;
        data_in         : in std_logic_vector(G_PORT_WIDTH-1 downto 0);
        data_in_valid   : in std_logic;
        sample_width    : in std_logic_vector(1 downto 0); --"10"-12bit, "01"-14bit, "00"-16bit;
        --output ports 
        data_out        : out std_logic_vector(G_PORT_WIDTH-1 downto 0);
        data_out_valid  : out std_logic;
        data_out_tlast  : out std_logic
        );
end bit_pack;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of bit_pack is
--Declare signals,  components here
constant C_64_MULTIPLE        : integer := G_PORT_WIDTH/64;

--inst0 signals
signal inst0_data64_out       : std_logic_vector(64*C_64_MULTIPLE-1 downto 0);
signal inst0_data48_in        : std_logic_vector(48*C_64_MULTIPLE-1 downto 0);
signal inst0_data_out_valid   : std_logic;
signal inst0_data_out_tlast   : std_logic;

--inst1 signals
signal inst1_data64_out       : std_logic_vector(64*C_64_MULTIPLE-1 downto 0);
signal inst1_data56_in        : std_logic_vector(56*C_64_MULTIPLE-1 downto 0);
signal inst1_data_out_valid   : std_logic;
signal inst1_data_out_tlast   : std_logic;

--mux signals
signal mux0_data64            : std_logic_vector(64*C_64_MULTIPLE-1 downto 0);
signal mux0_data_out_valid    : std_logic;
signal mux0_data_out_tlast    : std_logic;
signal mux0_sel               : std_logic;

signal mux0_data64_reg        : std_logic_vector(64*C_64_MULTIPLE-1 downto 0);
signal mux0_data_out_valid_reg: std_logic;
signal mux0_data_out_tlast_reg: std_logic;

--mux signals
signal mux1_data64            : std_logic_vector(64*C_64_MULTIPLE-1 downto 0);
signal mux1_data_out_valid    : std_logic;
signal mux1_data_out_tlast    : std_logic;
signal mux1_sel               : std_logic;

signal mux1_data64_reg        : std_logic_vector(64*C_64_MULTIPLE-1 downto 0);
signal mux1_data_out_valid_reg: std_logic;
signal mux1_data_out_tlast_reg: std_logic;
  
begin


-- ----------------------------------------------------------------------------
-- Component instances
-- ----------------------------------------------------------------------------
data48_in_assign: for I in 1 to G_PORT_WIDTH/16 generate
    inst0_data48_in(I*12-1 downto (I-1)*12) <= data_in(I*16-1 downto (I-1)*16+4);
end generate;

--inst0_data48_in <=   data_in(63 downto 52) & 
--                     data_in(47 downto 36) &
--                     data_in(31 downto 20) &
--                     data_in(15 downto 4);

inst0 : entity work.pack_48_to_64
generic map(
      G_BUSWIDTH        =>  G_PORT_WIDTH
      )
port map (
      clk               => clk,
      reset_n           => reset_n,
      data_in_wrreq     => data_in_valid,
      data48_in         => inst0_data48_in,
      data64_out        => inst0_data64_out,
      data_out_valid    => inst0_data_out_valid,
      data_out_tlast    => inst0_data_out_tlast
);

--14bit packing is mostly unused, adding a generic, to 
--disable this component in order to save fpga resources
DISABLER_14BIT : if G_DISABLE_14BIT = false generate
    -- 56to64 module does not support non 64bit bus
    DISABLER_14BIT_2 : if G_PORT_WIDTH = 64 generate
    inst1_data56_in <=   data_in(63 downto 50) &
                         data_in(47 downto 34) & 
                         data_in(31 downto 18) & 
                         data_in(15 downto 2);
    
    inst1 : entity work.pack_56_to_64 
    port map (
          clk            => clk,
          reset_n        => reset_n,
          data_in_wrreq  => data_in_valid,
          data56_in      => inst1_data56_in,
          data64_out     => inst1_data64_out,
          data_out_valid => inst1_data_out_valid
    );
    end generate;
end generate;

-- ----------------------------------------------------------------------------
-- MUX 0 
-- ----------------------------------------------------------------------------

mux0_sel                <= sample_width(1);

mux0_data64             <= inst0_data64_out when mux0_sel='1' else 
                        inst1_data64_out;
               
mux0_data_out_valid     <= inst0_data_out_valid when mux0_sel='1' else 
                        inst1_data_out_valid;
                        
mux0_data_out_tlast     <= inst0_data_out_tlast when mux0_sel='1' else 
                        '0';

-- ----------------------------------------------------------------------------
-- MUX 0 registers
-- ----------------------------------------------------------------------------
process(clk, reset_n)
begin 
   if reset_n = '0' then 
      mux0_data64_reg         <= (others=> '0');
      mux0_data_out_valid_reg <= '0';
      mux0_data_out_tlast_reg <= '0';
   elsif (clk'event AND clk = '1') then 
      mux0_data64_reg         <= mux0_data64;
      mux0_data_out_valid_reg <= mux0_data_out_valid;
      mux0_data_out_tlast_reg <= mux0_data_out_tlast;
   end if;
end process;

-- ----------------------------------------------------------------------------
-- MUX 1 
-- ----------------------------------------------------------------------------
mux1_sel                <= sample_width(1) OR sample_width(0);

mux1_data64             <= mux0_data64_reg         when mux1_sel='1' else 
                        data_in;

mux1_data_out_valid     <= mux0_data_out_valid_reg when mux1_sel='1' else 
                        data_in_valid;
                        
mux1_data_out_tlast     <= mux0_data_out_tlast_reg when mux1_sel='1' else
                        '1';

-- ----------------------------------------------------------------------------
-- MUX 1 registers
-- ----------------------------------------------------------------------------                                    
process(clk, reset_n)
begin 
   if reset_n = '0' then 
      mux1_data64_reg         <= (others=> '0');
      mux1_data_out_valid_reg <= '0';
      mux1_data_out_tlast_reg <= '0';
   elsif (clk'event AND clk = '1') then 
      mux1_data64_reg         <= mux1_data64;
      mux1_data_out_valid_reg <= mux1_data_out_valid;
      mux1_data_out_tlast_reg <= mux1_data_out_tlast;
   end if;
end process;

-- ----------------------------------------------------------------------------
-- Registers to output ports
-- ----------------------------------------------------------------------------
data_out       <= mux1_data64_reg;
data_out_valid <= mux1_data_out_valid_reg;
data_out_tlast <= mux1_data_out_tlast_reg;




end arch;   

