-- ----------------------------------------------------------------------------
-- FILE:          pll_ctrl.vhd
-- DESCRIPTION:   PLL control module
-- DATE:          3:32 PM Friday, May 11, 2018
-- AUTHOR(s):     Lime Microsystems
-- REVISIONS:
-- ----------------------------------------------------------------------------

-- ----------------------------------------------------------------------------
--NOTES:
-- ----------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- ----------------------------------------------------------------------------
-- Entity declaration
-- ----------------------------------------------------------------------------
entity pll_ctrl is
   generic (
      N_PLL       : integer :=2
   );
  port (
      --to_pllcfg         : out t_TO_PLLCFG;
      pllcfg_busy_bit   : out std_logic;
      pllcfg_done_bit   : out std_logic;
      auto_phcfg_done_bit : out std_logic;
      auto_phcfg_err_bit: out std_logic;
      pll_lock_vect     : out std_logic_vector(15 downto 0);

      --from_pllcfg       : in  t_FROM_PLLCFG;
      phcfg_start_bit   : in  std_logic;
      pllcfg_start_bit  : in  std_logic;
      pllrst_start_bit  : in  std_logic;
      phcfg_updn_i      : in  std_logic; --
      cnt_ind_i         : in  std_logic_vector(4 downto 0); --
      pll_ind           : in  std_logic_vector(4 downto 0);
      phcfg_mode_i      : in  std_logic;
      phcfg_tst_i       : in  std_logic;
      cnt_phase_i       : in  std_logic_vector(15 downto 0); --
      chp_curr          : in  std_logic_vector(2 downto 0);
      pllcfg_vcodiv     : in  std_logic;
      pllcfg_lf_res     : in  std_logic_vector(4 downto 0); 
      pllcfg_lf_cap     : in  std_logic_vector(1 downto 0); 
      m_odddiv          : in  std_logic; --
      m_byp             : in  std_logic; --
      n_odddiv          : in  std_logic; --
      n_byp             : in  std_logic; --
      c0_odddiv         : in  std_logic; --
      c0_byp            : in  std_logic; --
      c1_odddiv         : in  std_logic; --
      c1_byp            : in  std_logic; --
      c2_odddiv         : in  std_logic; --
      c2_byp            : in  std_logic; --
      c3_odddiv         : in  std_logic; --
      c3_byp            : in  std_logic; --
      c4_odddiv         : in  std_logic; --
      c4_byp            : in  std_logic; --
      n_cnt             : in  std_logic_vector(15 downto 0); -- 
      m_cnt             : in  std_logic_vector(15 downto 0); -- 
      m_frac            : in  std_logic_vector(31 downto 0); -- 
      c0_cnt            : in  std_logic_vector(15 downto 0); -- 
      c1_cnt            : in  std_logic_vector(15 downto 0); -- 
      c2_cnt            : in  std_logic_vector(15 downto 0); -- 
      c3_cnt            : in  std_logic_vector(15 downto 0); -- 
      c4_cnt            : in  std_logic_vector(15 downto 0); -- 
      auto_phcfg_smpls_i: in  std_logic_vector(15 downto 0);
      auto_phcfg_step_i : in  std_logic_vector(15 downto 0);
         -- Status Inputs
      pllcfg_busy       : in  std_logic_vector(N_PLL-1 downto 0);
      pllcfg_done       : in  std_logic_vector(N_PLL-1 downto 0);	
         -- PLL Lock flags
      pll_lock          : in  std_logic_vector(N_PLL-1 downto 0);	
         -- PLL Configuratioin Related
      phcfg_mode        : out std_logic;
      phcfg_tst         : out std_logic;
      phcfg_start       : out std_logic_vector(N_PLL-1 downto 0); --
      pllcfg_start      : out std_logic_vector(N_PLL-1 downto 0); --
      pllrst_start      : out std_logic_vector(N_PLL-1 downto 0); --
      phcfg_updn        : out std_logic; --
      cnt_ind           : out std_logic_vector(4 downto 0); --
      cnt_phase         : out std_logic_vector(15 downto 0); --
      pllcfg_data       : out std_logic_vector(143 downto 0);
      auto_phcfg_done   : in  std_logic_vector(N_PLL-1 downto 0);
      auto_phcfg_err    : in  std_logic_vector(N_PLL-1 downto 0);
      auto_phcfg_smpls  : out std_logic_vector(15 downto 0);
      auto_phcfg_step   : out std_logic_vector(15 downto 0)
      
        );
end pll_ctrl;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------
architecture arch of pll_ctrl is
--declare signals,  components here

signal pllcfg_busy_vect : std_logic_vector(15 downto 0);

signal pllcfg_done_vect : std_logic_vector(15 downto 0);

signal auto_phcfg_done_vect   : std_logic_vector(15 downto 0);

signal auto_phcfg_err_vect    : std_logic_vector(15 downto 0);


signal phcfg_start_vect : std_logic_vector(15 downto 0);
signal pllcfg_start_vect: std_logic_vector(15 downto 0);
signal pllrst_start_vect: std_logic_vector(15 downto 0);


signal pllcfg_data_rev  : std_logic_vector(143 downto 0);

  
begin

pllcfg_busy_vect(N_PLL-1 downto 0)     <= pllcfg_busy;
pllcfg_busy_vect(15 downto N_PLL)      <= (others=>'0');
   
pllcfg_done_vect(N_PLL-1 downto 0)     <= pllcfg_done;
pllcfg_done_vect(15 downto N_PLL)      <= (others=>'0');

auto_phcfg_done_vect(N_PLL-1 downto 0) <= auto_phcfg_done;
auto_phcfg_done_vect(15 downto N_PLL)  <= (others=>'0');

auto_phcfg_err_vect(N_PLL-1 downto 0)  <= auto_phcfg_err;
auto_phcfg_err_vect(15 downto N_PLL)   <= (others=>'0');

pll_lock_vect(N_PLL-1 downto 0)        <= pll_lock;
pll_lock_vect(15 downto N_PLL)         <= (others=>'0');

process(pll_ind, pllcfg_busy_vect, pllcfg_done_vect) begin
   pllcfg_busy_bit<=pllcfg_busy_vect(to_integer(unsigned(pll_ind)));
   pllcfg_done_bit<=pllcfg_done_vect(to_integer(unsigned(pll_ind)));
end process;

process(pll_ind, auto_phcfg_done_vect, auto_phcfg_err_vect) begin
   auto_phcfg_done_bit  <=auto_phcfg_done_vect(to_integer(unsigned(pll_ind)));
   auto_phcfg_err_bit   <=auto_phcfg_err_vect(to_integer(unsigned(pll_ind)));
end process;


process(pll_ind, phcfg_start_bit) begin
   phcfg_start_vect<=(others=>'0');
   phcfg_start_vect(to_integer(unsigned(pll_ind)))<=phcfg_start_bit;
end process;

process(pll_ind, pllcfg_start_bit) begin
   pllcfg_start_vect<=(others=>'0');
   pllcfg_start_vect(to_integer(unsigned(pll_ind)))<=pllcfg_start_bit;
end process;

process(pll_ind, pllrst_start_bit) begin
   pllrst_start_vect<=(others=>'0');
   pllrst_start_vect(to_integer(unsigned(pll_ind)))<=pllrst_start_bit;
end process;


phcfg_start  <= phcfg_start_vect(N_PLL-1 downto 0);
pllcfg_start <= pllcfg_start_vect(N_PLL-1 downto 0);
pllrst_start <= pllrst_start_vect(N_PLL-1 downto 0);


phcfg_updn              <= phcfg_updn_i;
cnt_ind                 <= cnt_ind_i;
phcfg_mode              <= phcfg_mode_i;
phcfg_tst               <= phcfg_tst_i;
cnt_phase               <= cnt_phase_i;	 
auto_phcfg_smpls        <= auto_phcfg_smpls_i;
auto_phcfg_step         <= auto_phcfg_step_i;

pllcfg_data_rev<=      "00" & pllcfg_lf_cap & pllcfg_lf_res  & pllcfg_vcodiv  & "00000" & chp_curr &
                        n_byp       & n_cnt (15  downto 8) & --N
                        n_odddiv    & n_cnt (7 downto 0) &
                        
                        m_byp       & m_cnt (15  downto 8) & --M 
                        m_odddiv    & m_cnt (7 downto 0) &
                        
                        c0_byp      & c0_cnt (15 downto 8) & --c0
                        c0_odddiv   & c0_cnt (7  downto 0) &
                        
                        c1_byp      & c1_cnt (15 downto 8) & --c1
                        c1_odddiv   & c1_cnt (7  downto 0) & 
                        
                        c2_byp      & c2_cnt (15 downto 8) & --c2
                        c2_odddiv   & c2_cnt (7  downto 0) &
                        
                        c3_byp      & c3_cnt (15 downto 8) & --c3
                        c3_odddiv   & c3_cnt (7  downto 0) &
  
                        c4_byp      & c4_cnt (15 downto 8) & --c4
                        c4_odddiv   & c4_cnt (7  downto 0) ;
                           
                           
for_lop : for i in 0 to 143 generate
   pllcfg_data(i) <= pllcfg_data_rev(143-i);  
end generate;  
  
end arch;




