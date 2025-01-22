-- ----------------------------------------------------------------------------
-- FILE:          iq_stream_combiner.vhd
-- DESCRIPTION:   Combines IQ samples into one bus
-- DATE:          15:11 2024-05-27
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- NOTES:
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

entity IQ_STREAM_COMBINER is
   port (
      CLK               : in    std_logic;
      RESET_N           : in    std_logic;
      mimo_en           : in    std_logic;
      ddr_en            : in    std_logic;
      S_AXIS_TVALID     : in    std_logic;
      S_AXIS_TREADY     : out   std_logic;
      S_AXIS_TDATA      : in    std_logic_vector(63 downto 0);
      S_AXIS_TKEEP      : in    std_logic_vector(7 downto 0);
      M_AXIS_TVALID     : out   std_logic;
      M_AXIS_TREADY     : in    std_logic;
      M_AXIS_TDATA      : out   std_logic_vector(63 downto 0);
      M_AXIS_TKEEP      : out   std_logic_vector(7 downto 0)
   );
end entity IQ_STREAM_COMBINER;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of IQ_STREAM_COMBINER is

   -- declare signals,  components here

   signal s_axis_tready_reg   : std_logic;
   signal m_axis_tdata_reg    : std_logic_vector(95 downto 0);
   signal m_axis_tvalid_reg   : std_logic_vector(2 downto 0);

begin

   -- ----------------------------------------------------------------------------
   -- Combining data bytes into m_axis_tdata bus register
   -- ----------------------------------------------------------------------------
   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         m_axis_tdata_reg <= (others => '0');
      elsif rising_edge(CLK) then
         if ((S_AXIS_TVALID and M_AXIS_TREADY) = '1') then
            if (S_AXIS_TKEEP = x"FF") then
               m_axis_tdata_reg(95 downto 32) <= S_AXIS_TDATA;
            elsif (S_AXIS_TKEEP = x"0F") then
               --m_axis_tdata_reg <= m_axis_tdata_reg(63 downto 0) & S_AXIS_TDATA(31 downto 0);
               m_axis_tdata_reg <= S_AXIS_TDATA(31 downto 0) & m_axis_tdata_reg(95 downto 32);
            elsif (S_AXIS_TKEEP = x"F0") then
               --m_axis_tdata_reg <= m_axis_tdata_reg(63 downto 0) & S_AXIS_TDATA(63 downto 32);
               m_axis_tdata_reg <= S_AXIS_TDATA(63 downto 32) & m_axis_tdata_reg(95 downto 32);
            else
               m_axis_tdata_reg <= m_axis_tdata_reg;
            end if;
         else
            m_axis_tdata_reg <= m_axis_tdata_reg;
         end if;
      end if;

   end process;

   -- ----------------------------------------------------------------------------
   -- m_axis_tvalid signal asserted when all m_axis_tdata bytes are combined
   -- ----------------------------------------------------------------------------
   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         m_axis_tvalid_reg <= (others => '0');
      elsif rising_edge(CLK) then
         if ((S_AXIS_TVALID and M_AXIS_TREADY) = '1') then
            if (S_AXIS_TKEEP = x"FF") then
               m_axis_tvalid_reg <= "111";
            elsif (S_AXIS_TKEEP = x"0F"  or  S_AXIS_TKEEP = x"F0") then
               if ((m_axis_tvalid_reg = "111" and mimo_en = '0' and ddr_en = '0') or
                   (m_axis_tvalid_reg = "011" and mimo_en = '0' and ddr_en = '1')) then
                  m_axis_tvalid_reg <= "00" & '1';
               else
                  m_axis_tvalid_reg <= m_axis_tvalid_reg(1 downto 0) & '1';
               end if;
            else
               m_axis_tvalid_reg <= m_axis_tvalid_reg(1 downto 0) & '0';
            end if;
         elsif ((m_axis_tvalid_reg = "111" and mimo_en = '0' and ddr_en = '0') or
                (m_axis_tvalid_reg = "011" and mimo_en = '0' and ddr_en = '1')) then
            m_axis_tvalid_reg <= "00" & '1';
         else
            m_axis_tvalid_reg <= m_axis_tvalid_reg;
         end if;
      end if;

   end process;

   process (CLK, RESET_N) is
   begin

      if (RESET_N = '0') then
         s_axis_tready_reg <= '0';
      elsif rising_edge(CLK) then
         s_axis_tready_reg <= '1';
      end if;

   end process;

   -- ----------------------------------------------------------------------------
   -- Output ports
   -- ----------------------------------------------------------------------------
   S_AXIS_TREADY <= s_axis_tready_reg;

   M_AXIS_TVALID <= m_axis_tvalid_reg(1) when mimo_en = '0' and ddr_en = '1' else m_axis_tvalid_reg(2);
   M_AXIS_TDATA  <= m_axis_tdata_reg(95 downto 32);
   M_AXIS_TKEEP  <= (others => '1');

end architecture ARCH;


