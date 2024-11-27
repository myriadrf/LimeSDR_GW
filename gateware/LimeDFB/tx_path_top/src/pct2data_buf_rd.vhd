-- ----------------------------------------------------------------------------
-- FILE:   pct2data_buf_rd.vhd
-- DESCRIPTION:  Read stored packets, strip headers, perform
--               timestamp synchronisation if needed
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
   use IEEE.math_real.all;
-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------

entity PCT2DATA_BUF_RD is
   generic (
      G_BUFF_COUNT   : integer := 4      -- 2,4 valid values
   );
   port (
      RESET_N                       : in    std_logic;
      -- common axis signals
      AXIS_ACLK                     : in    std_logic;

      S_AXIS_ARESET_N               : in    std_logic;
      S_AXIS_BUF_RESET_N            : out   std_logic_vector(G_BUFF_COUNT - 1 downto 0);
      S_AXIS_TVALID                 : in    std_logic_vector(G_BUFF_COUNT - 1 downto 0);
      S_AXIS_TDATA                  : in    std_logic_vector(127 downto 0);
      S_AXIS_TREADY                 : out   std_logic_vector(G_BUFF_COUNT - 1 downto 0);
      S_AXIS_TLAST                  : in    std_logic_vector(G_BUFF_COUNT - 1 downto 0);

      M_AXIS_ARESET_N               : in    std_logic;
      M_AXIS_TVALID                 : out   std_logic;
      M_AXIS_TDATA                  : out   std_logic_vector(127 downto 0);
      M_AXIS_TREADY                 : in    std_logic;
      M_AXIS_TLAST                  : out   std_logic;

      CURR_BUF_INDEX                : out   std_logic_vector(natural(ceil(log2(real(G_BUFF_COUNT))))-1 downto 0);

      SYNCH_DIS                     : in    std_logic;
      SAMPLE_NR                     : in    std_logic_vector(63 downto 0);
      PCT_LOSS_FLG                  : out   std_logic;
      PCT_LOSS_FLG_CLR              : in    std_logic
   );
end entity PCT2DATA_BUF_RD;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of PCT2DATA_BUF_RD is

   type T_STATE is (RESET_STATE, WAIT_HEADER, HEADER, DATA, SW_BUF, CLR_BUF);

   signal state                        : T_STATE;
   signal curbuf                       : integer range 0 to G_BUFF_COUNT - 1;
   signal conn_buf                     : std_logic;
   signal int_rst                      : std_logic;

   signal int_s_axis_tready            : std_logic_vector(G_BUFF_COUNT - 1 downto 0);

   signal s_axis_tready_override       : std_logic;

   signal compare_less                 : std_logic;
   signal compare_equal                : std_logic;

   signal pct_loss_flg_int             : std_logic;

   constant CURBUF_WIDTH               : natural := natural(ceil(log2(real(G_BUFF_COUNT))));

begin

   CURR_BUF_INDEX <= std_logic_vector(to_unsigned(curbuf, CURBUF_WIDTH));

   int_rst <= RESET_N and M_AXIS_ARESET_N and S_AXIS_ARESET_N;

   FSM : process (AXIS_ACLK, int_rst) is
   begin

      if (int_rst = '0') then
         state              <= RESET_STATE;
         S_AXIS_BUF_RESET_N <= (others => '0');
      elsif rising_edge(AXIS_ACLK) then
         S_AXIS_BUF_RESET_N     <= (others => '1');
         s_axis_tready_override <= '0';
         conn_buf               <= '0';

         case state is

            when RESET_STATE =>
               curbuf <= 0;
               state  <= WAIT_HEADER;

            when WAIT_HEADER =>
               if (S_AXIS_TVALID(curbuf) = '1') then
                  -- Sync can be disabled either globally or per buffer
                  if (SYNCH_DIS = '1' or S_AXIS_TDATA(4) = '1') then
                     state <= HEADER;
                     -- Throw away header by reading it
                     s_axis_tready_override <= '1';
                  else
                     -- If timestamps are equal then start reading buffer
                     if (compare_equal = '1') then
                        state <= HEADER;
                        -- Throw away header by reading it
                        s_axis_tready_override <= '1';
                     -- If timestamp is less than rx timestamp then reset buffer
                     elsif (compare_less = '1') then
                        state <= CLR_BUF;
                     end if;
                  end if;
               end if;

            when CLR_BUF =>
               S_AXIS_BUF_RESET_N(curbuf) <= '0';
               -- Wait until buffer is reset
               if (S_AXIS_TVALID(curbuf) = '0') then
                  state <= SW_BUF;
               end if;

            when HEADER =>
               conn_buf <= '1';
               if (S_AXIS_TVALID(curbuf) = '1' and M_AXIS_TREADY = '1') then
                  state <= DATA;
               end if;

            when DATA =>
               conn_buf <= '1';
               if (S_AXIS_TLAST(curbuf) = '1' and S_AXIS_TVALID(curbuf) = '1' and M_AXIS_TREADY = '1') then
                  state <= SW_BUF;
               end if;

            when SW_BUF =>
               if (curbuf >=  G_BUFF_COUNT - 1) then
                  curbuf <= 0;
               else
                  curbuf <= curbuf + 1;
               end if;
               state <= WAIT_HEADER;

            when others =>
               state <= RESET_STATE;

         end case;

      end if;

   end process FSM;

   ---------------------------------------------------------------------------
   ---- Generate pct_loss_flg_int
   ---- pct_loss_flg_int stays on only during the CLR_BUF state
   ---------------------------------------------------------------------------
   CHECK_BUF_RESET_PROC : process (AXIS_ACLK, int_rst) is

      variable clr_req : std_logic; -- flag clear request received

   begin

      if (int_rst = '0') then
         pct_loss_flg_int <= '0';
      elsif rising_edge(AXIS_ACLK) then
         if (state = CLR_BUF) then
            if (PCT_LOSS_FLG_CLR = '1') then
               clr_req := '1';
            end if;
            pct_loss_flg_int <= '1' and not clr_req;
         else
            pct_loss_flg_int <= '0';
            clr_req          := '0';
         end if;
      end if;

   end process CHECK_BUF_RESET_PROC;

   ---------------------------------------------------------------------------
   ---- This process makes sure that pct_loss_flg remains set for as long as
   ---- a clear flag is not received
   -----------------------------------------------------------------------------
   GEN_PCT_LOSS_FLG_PROC : process (AXIS_ACLK, int_rst) is
   begin

      if (int_rst = '0') then
         PCT_LOSS_FLG <= '0';
      elsif rising_edge(AXIS_ACLK) then
         -- only set the flag when int is high
         if (pct_loss_flg_int = '1') then
            PCT_LOSS_FLG <= '1';
         -- clear the flag when clr is high
         elsif (PCT_LOSS_FLG_CLR = '1') then
            PCT_LOSS_FLG <= '0';
         end if;
      end if;

   end process GEN_PCT_LOSS_FLG_PROC;

   -----------------------------------------------------------------------------
   -- Compare timestamps
   -- Process to compare timestamps stored in tdata
   -- Sets compare_less and compare_equal signals when state is WAIT_HEADER
   -- and current buffer's tvalid signal is high
   -----------------------------------------------------------------------------
   TS_CMP : process (AXIS_ACLK, int_rst) is
   begin

      if (int_rst = '0') then
         -- Reset compare signals
         compare_less  <= '0';
         compare_equal <= '0';
      elsif rising_edge(AXIS_ACLK) then
         -- Reset compare signals
         compare_less  <= '0';
         compare_equal <= '0';
         -- Check if state is WAIT_HEADER and current buffer's tvalid signal is high
         if (state = WAIT_HEADER and S_AXIS_TVALID(curbuf) = '1') then 
            -- Compare timestamps and set compare_less and compare_equal signals accordingly
            if (unsigned(S_AXIS_TDATA(127 downto 64)) < unsigned(SAMPLE_NR)) then
               compare_less <= '1';
            elsif (unsigned(S_AXIS_TDATA(127 downto 64)) = unsigned(SAMPLE_NR)) then
               compare_equal <= '1';
            end if;
         end if;
      end if;

   end process TS_CMP;

   -- Output current buffer's data, last and valid signals to M_AXIS
   M_AXIS_TDATA  <= S_AXIS_TDATA;                                     -- Output current buffer's data to M_AXIS
   M_AXIS_TLAST  <= S_AXIS_TLAST(curbuf) when (conn_buf = '1') else   -- Output current buffer's last signal to M_AXIS if connected
                    '0';                                              -- Otherwise, output 0
   M_AXIS_TVALID <= S_AXIS_TVALID(curbuf) when (conn_buf = '1') else  -- Output current buffer's valid signal to M_AXIS if connected
                    '0';                                              -- Otherwise, output 0

   GEN_ASSIGN_READY : for i in 0 to G_BUFF_COUNT - 1 generate
      -- Assign M_AXIS ready signal to internal signal if connected and it is the current buffer
      int_s_axis_tready(i) <= M_AXIS_TREADY when (conn_buf = '1' and curbuf = i) else
                              '0';  -- Otherwise, output 0
      -- Output current buffer's ready signal to S_AXIS or override it if current buffer is connected
      S_AXIS_TREADY(i) <= int_s_axis_tready(i) or s_axis_tready_override when curbuf = i else
                          '0';  -- Otherwise, output 0
   end generate GEN_ASSIGN_READY;

end architecture ARCH;
