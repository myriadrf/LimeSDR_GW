-- ----------------------------------------------------------------------------
-- FILE:          gpio_top.vhd
-- DESCRIPTION:   Generic module for GPIO control
-- DATE:          Jan 27, 2024
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

entity GPIO_TOP is
   generic (
      G_GPIO_WIDTH : integer := 12 --! Number of GPIOs
   );
   port (
      -- from logic
      GPIO_DIR          : in    std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO direction (0 - Output, 1 - Input)
      GPIO_OUT_VAL      : in    std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO output value
      GPIO_IN_VAL       : out   std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO input value
      GPIO_OVERRIDE     : in    std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO output override enable
      GPIO_OVERRIDE_DIR : in    std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO output override direction (0 - Output, 1 - Input)
      GPIO_OVERRIDE_VAL : in    std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO output override value
      -- to io ports
      GPIO_I            : in    std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO input (from IO)
      GPIO_O            : out   std_logic_vector(G_GPIO_WIDTH - 1 downto 0); --! GPIO output (to IO)
      GPIO_T            : out   std_logic_vector(G_GPIO_WIDTH - 1 downto 0)  --! GPIO direction
   );
end entity GPIO_TOP;

-- ----------------------------------------------------------------------------
-- Architecture
-- ----------------------------------------------------------------------------

architecture ARCH of GPIO_TOP is

   signal internal_gpio_out : std_logic_vector(G_GPIO_WIDTH - 1 downto 0);

begin

   -- Mux to determine if override is used
   MUX : process (GPIO_DIR, GPIO_OUT_VAL, GPIO_OVERRIDE, GPIO_OVERRIDE_VAL, GPIO_I, internal_gpio_out) is
   begin

      gpio_internal_assign : for i in 0 to G_GPIO_WIDTH - 1 loop

         if (GPIO_OVERRIDE(i) = '0') then
            internal_gpio_out(i) <= GPIO_OUT_VAL(i);
            GPIO_T               <= GPIO_DIR;
         else
            internal_gpio_out(i) <= GPIO_OVERRIDE_VAL(i);
            GPIO_T               <= GPIO_OVERRIDE_DIR;
         end if;

      end loop gpio_internal_assign;

      -- Mux to always return actual value present on pin regardless of direction selected
      -- (Returns internal gpio output value if direction is out)

      gpio_in_assign : for i in 0 to G_GPIO_WIDTH - 1 loop

         GPIO_IN_VAL(i) <= GPIO_I(i) when GPIO_DIR(i) = '1' else
                           internal_gpio_out(i);

      end loop gpio_in_assign;

      GPIO_O <= internal_gpio_out;
      

   end process MUX;

end architecture ARCH;
