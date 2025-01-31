# BinSmart-ESS

This project is about an energy storage solution (ESS) for small photovoltaic installations (German: "Balkonkraftwerke").

Having used such a "Balkonkraftwerk" with a modest capacity of 435 Wp for about a year, it showed that only a third of the
energy was consumed by our household, two thirds were fed into the grid (with no compensation whatsoever). I came up with
the idea of retrofitting an ESS solution, which should at least double the amount consumed by our household.
However, back in 2023 there was nothing available on the market that met my following criteria:
- storage capacity: at least 2 kWh
- total cost: less than €700
- charging/discharging power: at least 300 W
- indoor solution only, no additional wiring to outdoor PV installation
- silent operation (no fans)
- self-consumption of the entire system: less than 5 W
- total efficiency (end-to-end): greater than 80%

The last five criteria seemed impossible to find in one system. Affordable "AC coupled" ESS solutions were typically
inefficient, noisy and/or consumed a significant amount of energy themselves. One dealer offered me a
system with a self-consumption of 20 W, which corresponds to 175 kWh p.a. (more than a third of the entire PV output).

Some smart "zero export" solutions have been around for a while (e.g. OpenDTU, Ahoy, OpenDTU-OnBattery), however they
didn't seem to meet all of my criteria either. Today I would probably base my project on OpenDTU-OnBattery, but two years ago,
support for chargers was very limited. I must however give huge credits to Thomas Basler and everyone who supported
him with his OpenDTU project, without his and their work I would have never been able to get the RF24 communication with my
Hoymiles inverter working.
