
-- ----------------------------------------------------------------------------	
-- FILE: 	sample_padder.vhd
-- DESCRIPTION:	pads 12 bit samples to 16 bit format
-- DATE:	September 4, 2024
-- AUTHOR(s):	Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------	
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity sample_padder is
  port (
      --input ports 
        CLK       		: in std_logic;
        RESET_N   		: in std_logic;
		--
		S_AXIS_TVALID	: in std_logic;
		S_AXIS_TDATA   	: in std_logic_vector(127 downto 0);
		S_AXIS_TREADY   : out std_logic;
		S_AXIS_TLAST	: in std_logic;
		--
		M_AXIS_TDATA  	: out std_logic_vector(127 downto 0);
		M_AXIS_TVALID	: out std_logic;
		M_AXIS_TREADY   : in  std_logic;
		M_AXIS_TLAST	: out std_logic;
		--
		BYPASS			: in std_logic
        );
end sample_padder;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of sample_padder is

signal s_axis_tdata_reg  : std_logic_vector(127 downto 0);
signal s_axis_tlast_reg  : std_logic;
signal m_axis_tvalid_reg : std_logic;
signal s_axis_tready_reg : std_logic;
signal s_axis_tready_skip : std_logic;
signal packet_end 		  : std_logic;

signal s_axis_tready_int : std_logic;
signal m_axis_tdata_int  : std_logic_vector(127 downto 0);
signal m_axis_tvalid_int : std_logic;
signal m_axis_tlast_int  : std_logic;

signal packet_in_progress : std_logic := '0';
signal m_axis_tvalid_int_gate : std_logic;
signal data_sent              : std_logic;


type t_state_type is (state0, state1, state2, state3, end_packet);
signal state : t_state_type;
signal state_reg : t_state_type;
				
begin

--Avoid duplicating data if input lags
m_axis_tvalid_int_gate <= '0' when (state = state_reg) and data_sent = '1' else '1';

datareg_proc : process(CLK, RESET_N, BYPASS)
begin
	if RESET_N = '0' or BYPASS = '1' then
		S_AXIS_TDATA_reg <= (others => '0');
		s_axis_tlast_reg <= '0';
		-- do nothing
	elsif rising_edge(CLK) then
		m_axis_tvalid_reg <= m_axis_tvalid_int;
		if S_AXIS_TVALID = '1' and S_AXIS_TREADY = '1' then
			S_AXIS_TDATA_reg <= S_AXIS_TDATA;
			s_axis_tlast_reg <= S_AXIS_TLAST;
		end if;
	end if;
end process;


-- ----------------------------------------------------------------------------
-- FSM
-- ----------------------------------------------------------------------------

fsm : process(CLK, RESET_N, BYPASS)
begin
	if RESET_N = '0' or BYPASS = '1' then
		state <= state0;
		state_reg <= state0;
	elsif rising_edge(CLK) then
	    state_reg <= state;
		m_axis_tvalid_int <= '0';
		m_axis_tlast_int  <= '0';
		packet_end    <= '0';

		case state is 

			when state0 =>
                if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                    data_sent <= '1';
                end if;
                if S_AXIS_TVALID = '1' then
                    m_axis_tvalid_int <= '1';
                    if (M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1') or data_sent = '1' then
                        state     <= state1;
                        data_sent <= '0';
                    end if;
                end if;

			when state1 =>
                if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                    data_sent <= '1';
                end if;
                if S_AXIS_TVALID = '1' then
                    m_axis_tvalid_int <= '1';
                    if (M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1') or data_sent = '1' then
                        state     <= state2;
                        data_sent <= '0';
                    end if;
                end if;
				
			when state2 =>
                if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                    data_sent <= '1';
                end if;
                if S_AXIS_TVALID = '1'  then
                    m_axis_tvalid_int <= '1';
                    if (M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1')  or data_sent = '1' then
                        state     <= state3;
                        data_sent <= '0';
                        if S_AXIS_TLAST = '1' then
                            packet_end <= '1';
                        end if;
                    end if;
                end if;
				
			when state3 =>
                if M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' then
                    data_sent <= '1';
                end if;
                
                if S_AXIS_TVALID = '1' or packet_end = '1'  then
                    packet_end   <= packet_end; -- Maintain value while in state3
                    m_axis_tlast_int <= packet_end;
                    m_axis_tvalid_int <= '1';
                    if (M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1') or data_sent = '1' then
                        state     <= state0;
                        data_sent <= '0';
                    end if;
                end if;
							
			when others => state <= state0;
		end case;
	end if;
end process;

fsm_async : process(all)
begin
	s_axis_tready_skip <= '0'; 
	case state is 

		when state0 =>
				m_axis_tdata_int(127 downto 112) <= S_AXIS_TDATA(95 downto 84) & "0000";
				m_axis_tdata_int(111 downto 96 ) <= S_AXIS_TDATA(83 downto 72) & "0000";
				m_axis_tdata_int(95  downto 80 ) <= S_AXIS_TDATA(71 downto 60) & "0000";
				m_axis_tdata_int(79  downto 64 ) <= S_AXIS_TDATA(59 downto 48) & "0000";
				m_axis_tdata_int(63  downto 48 ) <= S_AXIS_TDATA(47 downto 36) & "0000";
				m_axis_tdata_int(47  downto 32 ) <= S_AXIS_TDATA(35 downto 24) & "0000";
				m_axis_tdata_int(31  downto 16 ) <= S_AXIS_TDATA(23 downto 12) & "0000";
				m_axis_tdata_int(15  downto 0  ) <= S_AXIS_TDATA(11 downto 0 ) & "0000";
			

		when state1 =>
				m_axis_tdata_int(127 downto 112) <= S_AXIS_TDATA(63 downto 52) & "0000";
				m_axis_tdata_int(111 downto 96 ) <= S_AXIS_TDATA(51 downto 40) & "0000";
				m_axis_tdata_int(95  downto 80 ) <= S_AXIS_TDATA(39 downto 28) & "0000";
				m_axis_tdata_int(79  downto 64 ) <= S_AXIS_TDATA(27 downto 16) & "0000";
				m_axis_tdata_int(63  downto 48 ) <= S_AXIS_TDATA(15 downto 4 ) & "0000";
				m_axis_tdata_int(47  downto 32 ) <= S_AXIS_TDATA(3  downto 0 ) & s_axis_tdata_reg(127 downto 120) & "0000";
				m_axis_tdata_int(31  downto 16 ) <= s_axis_tdata_reg(119 downto 108) & "0000";
				m_axis_tdata_int(15  downto 0  ) <= s_axis_tdata_reg(107 downto 96 ) & "0000";

		when state2 =>
				m_axis_tdata_int(127 downto 112) <= S_AXIS_TDATA(31 downto 20) & "0000";
				m_axis_tdata_int(111 downto 96 ) <= S_AXIS_TDATA(19 downto 8 ) & "0000";
				m_axis_tdata_int(95  downto 80 ) <= S_AXIS_TDATA(7  downto 0 ) & s_axis_tdata_reg(127 downto 124) & "0000";
				m_axis_tdata_int(79  downto 64 ) <= s_axis_tdata_reg(123 downto 112) & "0000";
				m_axis_tdata_int(63  downto 48 ) <= s_axis_tdata_reg(111 downto 100) & "0000";
				m_axis_tdata_int(47  downto 32 ) <= s_axis_tdata_reg(99  downto 88 ) & "0000";
				m_axis_tdata_int(31  downto 16 ) <= s_axis_tdata_reg(87  downto 76 ) & "0000";
				m_axis_tdata_int(15  downto 0  ) <= s_axis_tdata_reg(75  downto 64 ) & "0000";
			
		when state3 =>
				s_axis_tready_skip <= '1';
				m_axis_tdata_int(127 downto 112) <= s_axis_tdata_reg(127 downto 116) & "0000";
				m_axis_tdata_int(111 downto 96 ) <= s_axis_tdata_reg(115 downto 104) & "0000";
				m_axis_tdata_int(95  downto 80 ) <= s_axis_tdata_reg(103 downto 92 ) & "0000";
				m_axis_tdata_int(79  downto 64 ) <= s_axis_tdata_reg(91  downto 80 ) & "0000";
				m_axis_tdata_int(63  downto 48 ) <= s_axis_tdata_reg(79  downto 68 ) & "0000";
				m_axis_tdata_int(47  downto 32 ) <= s_axis_tdata_reg(67  downto 56 ) & "0000";
				m_axis_tdata_int(31  downto 16 ) <= s_axis_tdata_reg(55  downto 44 ) & "0000";
				m_axis_tdata_int(15  downto 0  ) <= s_axis_tdata_reg(43  downto 32 ) & "0000";
			

						
		when others => m_axis_tdata_int <= (others => '0');
	end case;




end process;

s_axis_tready_int <= '1' when (M_AXIS_TREADY = '1' and M_AXIS_TVALID = '1' and s_axis_tready_skip='0')  else '0';

S_AXIS_TREADY <= s_axis_tready_int when BYPASS = '0' else M_AXIS_TREADY;
M_AXIS_TDATA  <= m_axis_tdata_int  when BYPASS = '0' else S_AXIS_TDATA ;
M_AXIS_TVALID <= m_axis_tvalid_int and m_axis_tvalid_int_gate when BYPASS = '0' else S_AXIS_TVALID;
M_AXIS_TLAST  <= m_axis_tlast_int  when BYPASS = '0' else S_AXIS_TLAST ;




end arch;   



