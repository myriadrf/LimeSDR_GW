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
   signal tdata_buffer        : std_logic_vector(127 downto 0);
   signal tdata_buffer_reg    : std_logic_vector(127 downto 0);
   signal tdata_buffer_update : std_logic;
   signal offset              : integer;
   signal int_rst_n           : std_logic;
   signal s_axis_tready_skip  : std_logic;
   signal packet_end          : std_logic;

begin

   int_rst_n <= RESET_N and AXIS_ARESET_N;

   TDATA_BUF_PROC : process (AXIS_ACLK, int_rst_n) is
   begin

      if (int_rst_n = '0') then
         tdata_buffer_reg <= (others => '0');
      elsif rising_edge(AXIS_ACLK) then
         if (S_AXIS_TREADY = '1' and S_AXIS_TVALID = '1') then
            tdata_buffer_reg <= tdata_buffer;
         end if;
      end if;

   end process TDATA_BUF_PROC;

   FSM_PROC : process (AXIS_ACLK, int_rst_n) is
   begin

      if (int_rst_n = '0') then
         state <= WAIT_PACKET;
      elsif rising_edge(AXIS_ACLK) then
         packet_end    <= '0';
         M_AXIS_TVALID <= '0';
         tdata_buffer_update <= '0';

         case state is

            when WAIT_PACKET =>
               tdata_buffer <= S_AXIS_TDATA;
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

               case data_counter is

                  -- initial values
                  when 0 | 1 | 2 =>
                     if S_AXIS_TVALID = '1' then
                        M_AXIS_TVALID <= '1';
                        if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                           data_counter <= data_counter + 1;
                           if S_AXIS_TLAST = '1' then
                              packet_end <= '1';
                           end if;
                        end if;
                     end if;

                  -- last data cycle
                  when 3 =>
                     -- hold value     
                     packet_end <= packet_end;
                     if S_AXIS_TVALID = '1' or packet_end = '1' then
                        M_AXIS_TVALID <= '1';
                        if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                           if packet_end = '1' then
                              state <= WAIT_PACKET;
                           else
                              data_counter <= 0;
                           end if;
                        end if;
                     end if;               
                        
                  when others =>
                     -- This should not happen, if it does - go to reset state
                     state <= WAIT_PACKET;

               end case;

            -- Actions for SISO_16BIT state
            when MIMO_16BIT =>

               case data_counter is

                  when 0 => 
                     if S_AXIS_TVALID = '1' then
                        M_AXIS_TVALID <= '1';
                        if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                           data_counter <= data_counter + 1;
                           if S_AXIS_TLAST = '1' then
                              packet_end <= '1';
                           end if;
                        end if;
                     end if;

                  when 1 =>
                     packet_end <= packet_end;
                     if S_AXIS_TVALID = '1' or packet_end = '1' then
                        M_AXIS_TVALID <= '1';
                        if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                           if packet_end = '1' then
                              state <= WAIT_PACKET;
                           else
                              data_counter <= 0;
                           end if;
                        end if;
                     end if;

                  when others =>
                     -- This should not happen, if it does - go to reset state
                     state <= WAIT_PACKET;

               end case;

            -- Actions for MIMO_16BIT state
            when others =>
               state <= WAIT_PACKET;                                                                                                                 -- Default case

         end case;

      end if;

   end process FSM_PROC;

   fsm_async : process(all)
   begin
      s_axis_tready_skip <= '0';

      case state is

         when SISO_16BIT =>
         -- Just in case - avoid invalid values
            if (data_counter <= 3) then
               M_AXIS_TDATA(63 - offset downto 48 - offset) <= 16x"0";                                                                
               M_AXIS_TDATA(47 - offset downto 32 - offset) <= 16x"0";                                                                
               M_AXIS_TDATA(31 + offset downto 16 + offset) <= S_AXIS_TDATA(15 + (32 * data_counter) downto 0 + (32 * data_counter)); 
               M_AXIS_TDATA(15 + offset downto 0  + offset) <= S_AXIS_TDATA(31 + (32 * data_counter) downto 16 + (32 * data_counter));
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
               M_AXIS_TDATA(63 downto 48) <= S_AXIS_TDATA(15 + (64 * data_counter)  downto 0  + (64 * data_counter) );                                                                    -- AI
               M_AXIS_TDATA(47 downto 32) <= S_AXIS_TDATA(31 + (64 * data_counter)  downto 16 + (64 * data_counter) );                                                                    -- AQ
               M_AXIS_TDATA(31 downto 16) <= S_AXIS_TDATA(47 + (64 * data_counter)  downto 32 + (64 * data_counter) );                                                                    -- BI
               M_AXIS_TDATA(15 downto 0)  <= S_AXIS_TDATA(63 + (64 * data_counter)  downto 48 + (64 * data_counter) );                                                                    -- BQ
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
