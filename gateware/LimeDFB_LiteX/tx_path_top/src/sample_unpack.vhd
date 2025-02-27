-- ----------------------------------------------------------------------------
-- FILE:   sample_unpack.vhd
-- DESCRIPTION:  Reads the datastream from pct2data_buf_wr, assumes that
--               incoming data is interleaved.
--               Checks CH_EN and SAMPLE_WIDTH only between packets.
-- DATE:  June 25, 2024
-- AUTHOR(s):  Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- Notes: If invalid CH_EN and/or SAMPLE_WIDTH values are provided,
--        this module will not start reading data and stall.
-- ----------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;

entity SAMPLE_UNPACK is
   port (
      AXIS_ACLK      : in    std_logic;
      AXIS_ARESET_N  : in    std_logic;
      RESET_N        : in    std_logic;

      S_AXIS_TDATA   : in    std_logic_vector(127 downto 0);
      S_AXIS_TREADY  : out   std_logic;
      S_AXIS_TVALID  : in    std_logic;
      S_AXIS_TLAST   : in    std_logic;

      M_AXIS_TDATA   : out   std_logic_vector(63 downto 0);
      M_AXIS_TREADY  : in    std_logic;
      M_AXIS_TVALID  : out   std_logic;

      CH_EN          : in    std_logic_vector(1 downto 0)
   );
end entity SAMPLE_UNPACK;

architecture RTL of SAMPLE_UNPACK is

   type T_STATE is (WAIT_PACKET, SISO_16BIT, MIMO_16BIT);

   signal state               : T_STATE;
 
   signal data_counter        : integer range 0 to 15;
   signal offset              : integer;
   signal int_rst_n           : std_logic;
   signal s_axis_tready_skip  : std_logic;
   signal s_axi_data32        : std_logic_vector(31 downto 0);
   signal s_axi_data64        : std_logic_vector(63 downto 0);

begin

   int_rst_n <= RESET_N and AXIS_ARESET_N;

   FSM_PROC : process (AXIS_ACLK, int_rst_n)
   begin

      if (int_rst_n = '0') then
         state <= WAIT_PACKET;
         M_AXIS_TVALID <= '0';
         data_counter <= 0;
      elsif rising_edge(AXIS_ACLK) then
         M_AXIS_TVALID <= '0';

         case state is

            when WAIT_PACKET =>
               data_counter <= 0;

               if (S_AXIS_TVALID = '1') then
                  if (CH_EN = "01") then
                     offset <= 32;                                                                                                                  
                  elsif (CH_EN = "10") then
                     offset <= 0;                                                                                                                   
                  end if;

                  if (CH_EN = "11") then                                                                                                                                     
                        state <= MIMO_16BIT;                                                                                                        
                  elsif (CH_EN = "01" or CH_EN = "10") then                                                                                                                                                                                 
                        state <= SISO_16BIT;                                                                                                        
                  end if;
               end if;

            -- Actions for MIMO_12BIT state
            when SISO_16BIT =>

                     if S_AXIS_TVALID = '1' then
                        M_AXIS_TVALID <= '1';
                        if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                           if data_counter < 3 then
                               data_counter <= data_counter + 1;
                           else
                               data_counter <= 0;
                           end if;
                        end if;
                     end if;

            -- Actions for MIMO_16BIT state
            when MIMO_16BIT =>

                     if S_AXIS_TVALID = '1' then
                        M_AXIS_TVALID <= '1';
                        if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                           if data_counter < 1 then
                               data_counter <= data_counter + 1;
                           else
                               data_counter <= 0;
                           end if;
                        end if;
                     end if;

            when others =>
               state <= WAIT_PACKET;                                                                                                                 -- Default case

         end case;

      end if;

   end process FSM_PROC;

   process(all)
   begin
     case data_counter is
        when 0 =>
           s_axi_data32 <= S_AXIS_TDATA(31 + (32 * 0) downto 0 + (32 * 0));
        when 1 =>
           s_axi_data32 <= S_AXIS_TDATA(31 + (32 * 1) downto 0 + (32 * 1));
        when 2 =>
           s_axi_data32 <= S_AXIS_TDATA(31 + (32 * 2) downto 0 + (32 * 2));
        when others =>
           s_axi_data32 <= S_AXIS_TDATA(31 + (32 * 3) downto 0 + (32 * 3));
     end case;
     if data_counter = 0 then
        s_axi_data64 <=
               S_AXIS_TDATA(15 downto  0) & -- AI
               S_AXIS_TDATA(31 downto 16) & -- AQ
               S_AXIS_TDATA(47 downto 32) & -- BI
               S_AXIS_TDATA(63 downto 48);  -- BQ
     else
        s_axi_data64 <=
               S_AXIS_TDATA( 79 downto  64) & -- AI
               S_AXIS_TDATA( 95 downto  80) & -- AQ
               S_AXIS_TDATA(111 downto  96) & -- BI
               S_AXIS_TDATA(127 downto 112);  -- BQ
     end if;
   end process;

   fsm_async : process(all)
   begin
      s_axis_tready_skip <= '0';

      case state is

         when SISO_16BIT =>
         -- Just in case - avoid invalid values
            if (data_counter <= 3) then
               if offset = 32 then
                  M_AXIS_TDATA(31 downto 16) <= (others => '0');
                  M_AXIS_TDATA(15 downto  0) <= (others => '0');
                  M_AXIS_TDATA(63 downto 48) <= s_axi_data32(15 downto  0);
                  M_AXIS_TDATA(47 downto 32) <= s_axi_data32(31 downto 16);
               else
                  M_AXIS_TDATA(63 downto 48) <= (others => '0');
                  M_AXIS_TDATA(47 downto 32) <= (others => '0');
                  M_AXIS_TDATA(31 downto 16) <= s_axi_data32(15 downto  0);
                  M_AXIS_TDATA(15 downto 0 ) <= s_axi_data32(31 downto 16);
               end if;
               if data_counter = 3 then
                  s_axis_tready_skip <= '0';
               else
                  s_axis_tready_skip <= '1';
               end if;
            else
               M_AXIS_TDATA <= (others => '0');
            end if;

         when MIMO_16BIT =>
         -- Just in case - avoid invalid values
            if (data_counter <= 1) then
               M_AXIS_TDATA <= s_axi_data64;
               if data_counter = 1 then
                  s_axis_tready_skip <= '0';
               else
                  s_axis_tready_skip <= '1';
               end if;
            else
               M_AXIS_TDATA <= (others => '0');
            end if;

         when others => 
            M_AXIS_TDATA <= (others => '0');
         
      end case;

   end process;


   S_AXIS_TREADY <= '1' when (M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' and s_axis_tready_skip='0')  else '0';

-- Implementation of the architecture goes here

end architecture RTL;
