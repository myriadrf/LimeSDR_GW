-- ----------------------------------------------------------------------------
-- FILE:          data2packets_fsm.vhd
-- DESCRIPTION:   Forms packets with provided header
-- DATE:          10:11 2024-05-24
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
-- NOTES:
-- ----------------------------------------------------------------------------

library ieee;
   use ieee.std_logic_1164.all;
   use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------

entity DATA2PACKETS_FSM is
   port (
      ACLK                 : in    std_logic;
      ARESET_N             : in    std_logic;

      PCT_SIZE             : in    std_logic_vector(15 downto 0); -- Packet size
      PCT_HDR_0            : in    std_logic_vector(63 downto 0);
      PCT_HDR_1            : in    std_logic_vector(63 downto 0);
      -- AXIS Slave
      S_AXIS_TVALID        : in    std_logic;
      S_AXIS_TREADY        : out   std_logic;
      S_AXIS_TDATA         : in    std_logic_vector(127 downto 0);
      S_AXIS_TLAST         : in    std_logic;
      -- AXIS Master
      M_AXIS_TVALID        : out   std_logic;
      M_AXIS_TREADY        : in    std_logic;
      M_AXIS_TDATA         : out   std_logic_vector(127 downto 0);
      M_AXIS_TLAST         : out   std_logic;
      -- Misc
      WR_DATA_COUNT_AXIS   : in    std_logic_vector(8 downto 0);

      DBG_DROP_SAMPLES : out std_logic;
      DBG_WR_HEADER    : out std_logic

   );
end entity DATA2PACKETS_FSM;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of DATA2PACKETS_FSM is

   -- declare signals,  components here

   type STATE_TYPE is (IDLE, DROP_SAMPLES, WR_HEADER, WR_PAYLOAD, PCT_END);

   signal current_state, next_state : STATE_TYPE;

   signal space_required            : unsigned(15 downto 0);

   constant MAX_BUFFER_WORDS        : unsigned(15 downto 0) := x"0200";

   signal s_axis_tready_reg         : std_logic;

   signal m_axis_tvalid_reg         : std_logic;
   signal m_axis_tdata_reg          : std_logic_vector(M_AXIS_TDATA'length-1 downto 0);
   signal m_axis_tlast_reg          : std_logic;

   signal pct_wrcnt                 : unsigned(15 downto 0);

begin

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         space_required <= (others => '0');
      elsif rising_edge(ACLK) then
         space_required <= unsigned(WR_DATA_COUNT_AXIS) + unsigned(PCT_SIZE);
      end if;

   end process;

   -- ----------------------------------------------------------------------------
   -- State machine
   -- ----------------------------------------------------------------------------
   FSM_F : process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         current_state <= IDLE;
      elsif rising_edge(ACLK) then
         current_state <= next_state;
      end if;

   end process FSM_F;

   -- ----------------------------------------------------------------------------
   -- state machine combo
   -- ----------------------------------------------------------------------------
   FSM : process (all) is
   begin

      next_state <= current_state;

      case current_state is

         when IDLE => -- state

            if (S_AXIS_TVALID = '1') then
               if (space_required >= MAX_BUFFER_WORDS) then
                  next_state <= DROP_SAMPLES;
               else
                  next_state <= WR_HEADER;
               end if;
            else
               next_state <= IDLE;
            end if;

         -- Droping samples until there is enough space in buffer and making sure that whole frame of bit packed samples are droped.
         when DROP_SAMPLES =>

            if (space_required <= MAX_BUFFER_WORDS  and S_AXIS_TLAST = '1') then
               next_state <= IDLE;
            else
               next_state <= DROP_SAMPLES;
            end if;

         when WR_HEADER =>
            next_state <= WR_PAYLOAD;

         when WR_PAYLOAD =>

            if (pct_wrcnt < unsigned(PCT_SIZE)) then
               next_state <= WR_PAYLOAD;
            else
               next_state <= PCT_END;
            end if;

         when PCT_END =>
            next_state <= IDLE;

         when others =>
            next_state <= IDLE;

      end case;

   end process FSM;


   DBG_DROP_SAMPLES <= '1' when (current_state = DROP_SAMPLES) else '0';
   DBG_WR_HEADER    <= '1' when (current_state = WR_HEADER)    else '0';

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         m_axis_tvalid_reg <= '0';
      elsif rising_edge(ACLK) then
         if (current_state = WR_HEADER or (current_state = WR_PAYLOAD and S_AXIS_TVALID='1' and s_axis_tready_reg='1')) then
            m_axis_tvalid_reg <= '1';
         else
            m_axis_tvalid_reg <= '0';
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         s_axis_tready_reg <= '0';
      elsif rising_edge(ACLK) then
         if ((current_state = WR_PAYLOAD and M_AXIS_TREADY = '1' and pct_wrcnt = 1) or current_state = DROP_SAMPLES) then
            s_axis_tready_reg <= '1';
         elsif (current_state = WR_PAYLOAD and S_AXIS_TVALID='1' and s_axis_tready_reg='1' and pct_wrcnt=unsigned(PCT_SIZE) - 1) then
            s_axis_tready_reg <= '0';
         else
            s_axis_tready_reg <= s_axis_tready_reg;
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         pct_wrcnt <= (others => '0');
      elsif rising_edge(ACLK) then
         if (current_state = WR_HEADER or (current_state = WR_PAYLOAD and S_AXIS_TVALID='1' and s_axis_tready_reg='1')) then
            pct_wrcnt <= pct_wrcnt + 1;
         elsif (current_state = PCT_END) then
            pct_wrcnt <= (others => '0');
         else
            pct_wrcnt <= pct_wrcnt;
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         m_axis_tlast_reg <= '0';
      elsif rising_edge(ACLK) then
         if (current_state = WR_PAYLOAD and pct_wrcnt=unsigned(PCT_SIZE) - 1) then
            m_axis_tlast_reg <= '1';
         else
            m_axis_tlast_reg <= '0';
         end if;
      end if;

   end process;

   process (ACLK, ARESET_N) is
   begin

      if (ARESET_N = '0') then
         m_axis_tdata_reg <= (others => '0');
      elsif rising_edge(ACLK) then
         if (current_state = WR_HEADER) then
            m_axis_tdata_reg <= PCT_HDR_1 & PCT_HDR_0;
         elsif (current_state = WR_PAYLOAD and S_AXIS_TVALID = '1' and s_axis_tready_reg = '1') then
            m_axis_tdata_reg <= S_AXIS_TDATA;
         else
            m_axis_tdata_reg <= m_axis_tdata_reg;
         end if;
      end if;

   end process;

   -- ----------------------------------------------------------------------------
   -- Output ports
   -- ----------------------------------------------------------------------------
   S_AXIS_TREADY <= s_axis_tready_reg;

   M_AXIS_TVALID <= m_axis_tvalid_reg;
   M_AXIS_TDATA  <= m_axis_tdata_reg;
   M_AXIS_TLAST  <= m_axis_tlast_reg;

end architecture ARCH;


