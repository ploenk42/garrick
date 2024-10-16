## Getting started with garricks wands
On boot, the RGB LED will light up each color for a second in the order of
- $${\color{RED}RED}$$ $${\color{GREEN}GREEN}$$ $${\color{BLUE}BLUE}$$

## update sequence

If no spell attempt is recognized within 10 seconds after boot, wifi will be activated. 
- $${\color{BLUE}BLUE}$$

### update sequence without known wifi
A unsecure accesspoint named "garrick" will appear.
If you join this wifi it will open a captive portal (http://192.168.4.1) where you can configure your wifi network.
Afterwards it will reboot into the next update sequence.

### update sequence with known wifi
If the device joins a known network it will, instead of opening an accesspoint, try to update.
- $${\color{BLUE}BLUE}$$ $${\color{BLUE}BLUE}$$ $${\color{BLUE}BLUE}$$

If the update progress succeeds it will signal this via the LED.

- $${\color{GREEN}GREEN}$$ $${\color{GREEN}GREEN}$$ $${\color{GREEN}GREEN}$$

## spells

In version < 1.0.2 any movement will light up the led and play sound

- $${\color{BLUE}BLUE}$$
- $${\color{RED}RED}$$

In version 1.0.2 the spell sequence needs to be initialized by holding the wand straight downwards

- $${\color{GREEN}GREEN}$$ $${\color{GREEN}\infty}$$ 

In version 1.0.3 the spell sequence needs to be initialized by holding the wand straight upwards

- $${\color{GREEN}GREEN}$$ $${\color{GREEN}\infty}$$ 

