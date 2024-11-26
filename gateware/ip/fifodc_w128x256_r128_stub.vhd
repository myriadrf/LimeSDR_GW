library IEEE;
use IEEE.std_logic_1164.all;

entity fifodc_w128x256_r128 is
    port (
        Data: in  std_logic_vector(127 downto 0); 
        WrClock: in  std_logic; 
        RdClock: in  std_logic; 
        WrEn: in  std_logic; 
        RdEn: in  std_logic; 
        Reset: in  std_logic; 
        RPReset: in  std_logic; 
        Q: out  std_logic_vector(127 downto 0); 
        WCNT: out  std_logic_vector(8 downto 0); 
        RCNT: out  std_logic_vector(8 downto 0); 
        Empty: out  std_logic; 
        Full: out  std_logic);
end fifodc_w128x256_r128;

architecture Structure of fifodc_w128x256_r128 is

begin

end Structure;
