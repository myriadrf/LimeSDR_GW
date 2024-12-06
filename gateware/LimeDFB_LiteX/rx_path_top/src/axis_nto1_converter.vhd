-- ----------------------------------------------------------------------------
-- FILE:          axis_nto1_converter.vhd
-- DESCRIPTION:   Converts AXIS stream bus to specified ratio
-- DATE:          10:53 2024-05-23
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- NOTES:
-- This module works in AXIS packet mode.
-- e.g. if g_N_RATIO=2, and g_DATA_WIDTH=64 this module expects four 64bit words
-- and outputs two 128bit words.
-- Same with tlast - expects two tlast and outputs one.

-- s_axis interfaface is always ready to accept data thus s_axis_tready is connected
-- to reset_n
--
-- m_axis interface is unable to accept-backpressure. There is no need for it as
-- this module is designed for continous data stream
-- ----------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------

entity AXIS_NTO1_CONVERTER is
   generic (
      G_N_RATIO      : integer := 2; -- Available values - 2, 4, 8
      G_DATA_WIDTH   : integer := 64 -- AXIS Slave data width
   );
   port (
      ACLK           : in    std_logic;
      ARESET_N       : in    std_logic;
      -- AXIS Slave
      S_AXIS_TVALID  : in    std_logic;
      S_AXIS_TREADY  : out   std_logic;
      S_AXIS_TDATA   : in    std_logic_vector(G_DATA_WIDTH - 1 downto 0);
      S_AXIS_TLAST   : in    std_logic;
      -- AXIS Master
      M_AXIS_TVALID  : out   std_logic;
      M_AXIS_TDATA   : out   std_logic_vector(G_DATA_WIDTH * G_N_RATIO - 1 downto 0);
      M_AXIS_TLAST   : out   std_logic
   );
end entity AXIS_NTO1_CONVERTER;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of AXIS_NTO1_CONVERTER is

   -- declare signals,  components here

   type T_DATA_REG_ARRAY_TYPE is array (0 to G_N_RATIO - 1) of std_logic_vector(s_axis_tdata'length-1 downto 0);

   signal data_reg_array    : T_DATA_REG_ARRAY_TYPE;

   signal s_axis_tready_reg : std_logic;

   signal s_axis_tvalid_cnt : unsigned(3 downto 0);
   signal s_axis_tlast_cnt  : unsigned(3 downto 0);

   signal m_axis_tvalid_reg : std_logic;
   signal m_axis_tlast_reg  : std_logic;

begin

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         s_axis_tvalid_cnt <= (others => '0');
      elsif rising_edge(ACLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID = '1') then
            if (s_axis_tvalid_cnt < G_N_RATIO - 1) then
               s_axis_tvalid_cnt <= s_axis_tvalid_cnt + 1;
            else
               s_axis_tvalid_cnt <= (others => '0');
            end if;
         else
            s_axis_tvalid_cnt <= s_axis_tvalid_cnt;
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         s_axis_tlast_cnt <= (others => '0');
      elsif rising_edge(ACLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID = '1' and S_AXIS_TLAST ='1') then
            if (s_axis_tlast_cnt < G_N_RATIO - 1) then
               s_axis_tlast_cnt <= s_axis_tlast_cnt + 1;
            else
               s_axis_tlast_cnt <= (others => '0');
            end if;
         elsif (m_axis_tvalid_reg='1' and m_axis_tlast_reg = '1') then
            s_axis_tlast_cnt <= (others => '0');
         else
            s_axis_tlast_cnt <= s_axis_tlast_cnt;
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         data_reg_array <= (others => (others => '0'));
      elsif rising_edge(ACLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID = '1') then
            data_reg_array(1) <= data_reg_array(0);
            data_reg_array(0) <= S_AXIS_TDATA;
         else
            data_reg_array <= data_reg_array;
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         m_axis_tvalid_reg <= '0';
         m_axis_tlast_reg  <= '0';
      elsif rising_edge(ACLK) then
         if (s_axis_tready_reg = '1' and S_AXIS_TVALID = '1' and s_axis_tvalid_cnt = G_N_RATIO - 1) then
            m_axis_tvalid_reg <= '1';
         else
            m_axis_tvalid_reg <= '0';
         end if;

         if (s_axis_tready_reg = '1' and S_AXIS_TVALID = '1' and S_AXIS_TLAST='1' and s_axis_tlast_cnt = G_N_RATIO - 1) then
            m_axis_tlast_reg <= '1';
         else
            m_axis_tlast_reg <= '0';
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         s_axis_tready_reg <= '0';
      elsif rising_edge(ACLK) then
         s_axis_tready_reg <= '1';
      end if;

   end process;

   S_AXIS_TREADY <= s_axis_tready_reg;

   M_AXIS_TVALID <= m_axis_tvalid_reg;
   M_AXIS_TDATA  <= data_reg_array(0) & data_reg_array(1);
   M_AXIS_TLAST  <= m_axis_tlast_reg;

end architecture ARCH;


