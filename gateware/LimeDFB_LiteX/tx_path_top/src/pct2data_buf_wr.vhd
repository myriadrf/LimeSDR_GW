-- ----------------------------------------------------------------------------
-- FILE:   pct2data_buf_wr.vhd
-- DESCRIPTION:  Decode packet start, write packets to buffers, add TLAST.
-- DATE:  June 25, 2024
-- AUTHOR(s):  Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- Notes:
-- ----------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------

entity PCT2DATA_BUF_WR is
   generic (
      G_BUFF_COUNT      : integer := 4 -- 2,4 valid values
   );
   port (
      RESET_N              : in    std_logic;
      -- common axis signals
      AXIS_ACLK            : in    std_logic;
      --! @virtualbus s_axis_tx @dir in Transmit AXIS bus
      S_AXIS_ARESET_N      : in    std_logic;                                                                                                                        --! TX interface active low reset
      S_AXIS_TVALID        : in    std_logic;                                                                                                                        --! TX FIFO write request
      S_AXIS_TDATA         : in    std_logic_vector(127 downto 0);                                                                                                   --! TX FIFO data
      S_AXIS_TREADY        : out   std_logic;                                                                                                                        --! TX FIFO write full
      S_AXIS_TLAST         : in    std_logic;
      --! @virtualbus m_axis_rx @dir out Receive AXIS bus

      M_AXIS_ARESET_N      : in    std_logic;                                                                                                                        --! RX interface active low reset
      M_AXIS_TVALID        : out   std_logic_vector(G_BUFF_COUNT - 1 downto 0);                                                                                      --! Received data from DIQ2 port valid signal
      M_AXIS_TDATA         : out   std_logic_vector(127 downto 0);                                                                                                   --! Received data from DIQ2 port
      M_AXIS_TREADY        : in    std_logic_vector(G_BUFF_COUNT - 1 downto 0);
      M_AXIS_TLAST         : out   std_logic_vector(G_BUFF_COUNT - 1 downto 0);                                                                                      --! @end

      BUF_EMPTY            : in    std_logic_vector(G_BUFF_COUNT - 1 downto 0)
   );
end entity PCT2DATA_BUF_WR;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of PCT2DATA_BUF_WR is

   -- State machine type declaration

   type T_STATE is (
      RESET_STATE,                                                                --! Reset counters and current buffer
      WAIT_DATA,                                                                  --! Reset read counter
      DATA,                                                                       --! Keep reading data until maximum number of reads is reached
      SW_BUF                                                                      --! Switch buffer
   );

   -- State machine signal declaration
   signal state                     : T_STATE;                                    --! Current state
   signal curbuf                    : integer range 0 to G_BUFF_COUNT - 1;        --! Current buffer index
   signal conn_buf                  : std_logic;                                  --! Buffer connection signal
   signal int_rst                   : std_logic;                                  --! Internal reset signal

   signal rd_cnt                    : unsigned(15 downto 0);                      --! Read counter
   signal rd_cnt_max                : unsigned(15 downto 0);                      --! Maximum read counter
    signal m_axis_tvalid_int         : std_logic_vector(G_BUFF_COUNT - 1 downto 0);

   constant C_RD_RATIO              : integer := (S_AXIS_TDATA'left+1) / 8;       --! Read ratio (Number of bytes in bus)

begin

   int_rst <= RESET_N and M_AXIS_ARESET_N and S_AXIS_ARESET_N;

   -------------------------------------------------------------------------
   -- State machine process
   --  This process describes the behavior of the state machine.
   -------------------------------------------------------------------------
   FSM : process (AXIS_ACLK, int_rst) is
   begin

      if (int_rst = '0') then
         -- Reset state
         state <= RESET_STATE;
      elsif rising_edge(AXIS_ACLK) then
         -- Reset output signals
         conn_buf     <= '0';

         case state is

            when RESET_STATE =>
               -- Reset counters and current buffer
               rd_cnt     <= (others => '0');
               rd_cnt_max <= (others => '0');
               curbuf     <= 0;
               conn_buf   <= '0';
               -- Next state is waiting for data
               state <= WAIT_DATA;

            when WAIT_DATA =>
               -- Reset read counter
               rd_cnt <= (others => '0');
               
               if (S_AXIS_TVALID = '1') then
                  -- Determine maximum number of reads
                  rd_cnt_max <= unsigned(S_AXIS_TDATA(23 downto 8)) / C_RD_RATIO + 1; -- +1 for header
                  -- If current buffer is empty, start reading data
                  if (BUF_EMPTY(curbuf) = '1') then
                     state    <= DATA;
                     conn_buf <= '1';
                  end if;
               end if;

            when DATA =>
               -- Keep reading data until maximum number of reads is reached
               conn_buf <= '1';
               if (S_AXIS_TVALID = '1' and M_AXIS_TREADY(curbuf) = '1') then
                  rd_cnt <= rd_cnt + 1;
                  if (rd_cnt = rd_cnt_max - 1) then
                     state    <= SW_BUF;
                     conn_buf <= '0';
                  end if;
                  
               end if;

            when SW_BUF =>
               -- Reset read counter
               rd_cnt <= (others => '0');
               -- Switch to next buffer
               if (curbuf = (G_BUFF_COUNT - 1)) then
                  curbuf <= 0;
               else
                  curbuf <= curbuf + 1;
               end if;
               -- Next state is waiting for data
               state <= WAIT_DATA;

            when others =>
               -- Reset state if unknown state is reached
               state <= RESET_STATE;

         end case;

      end if;

   end process FSM;

   M_AXIS_TDATA  <= S_AXIS_TDATA;
   S_AXIS_TREADY <= M_AXIS_TREADY(curbuf) when conn_buf = '1' else
                    '0';

    GEN_ASSIGN_VALID : for i in 0 to G_BUFF_COUNT - 1 generate
        m_axis_tvalid_int(i) <= S_AXIS_TVALID when (curbuf=i and conn_buf='1') else
            '0';
            M_AXIS_TLAST(i) <= m_axis_tvalid_int(i) when (curbuf=i and (rd_cnt >= (rd_cnt_max - 1))) else
                '0';
        end generate GEN_ASSIGN_VALID;

    M_AXIS_TVALID <= m_axis_tvalid_int;

end architecture ARCH;





