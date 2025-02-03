# BinSmart-ESS

This project is about an energy storage solution (ESS) for small photovoltaic installations (German: "Balkonkraftwerke").

Having used a "Balkonkraftwerk" with a modest output of 435 Wp for a year, it showed that only a third of the energy was consumed by our household, two thirds were fed into the grid (with no compensation whatsoever). I came up with the idea of retrofitting an ESS solution, which should at least double the proportion consumed by our household.
However, in 2023 there was nothing available on the market that met my following criteria:
- storage capacity: at least 2 kWh
- total cost: less than €600
- charging/discharging power: at least 300 W
- indoor "AC coupled" solution, no additional wiring to outdoor PV installation
- "zero" energy export to grid (less than 1% when battery not fully charged)
- silent operation (no fans)
- self-consumption of the entire system: less than 5 W
- total efficiency (AC-to-AC): above 80%


Especially the last three criteria appeared impossible to meet. Affordable "AC coupled" ESS solutions were typically inefficient, noisy and consumed a significant amount of energy themselves. One dealer offered me a system with a self-consumption of 20 W, which corresponds to 175 kWh p.a. (more than a third of our entire PV output).

OpenDTU-OnBattery, an excellent and flexible "zero export" solution based on Thomas Basler's fabulous OpenDTU project, has been around since 2023, but in the beginning there was (or still is?) no support for Meanwell chargers, and using a Victron charger would have violated my cost criteria. I must however give credits to Thomas Basler and everyone who supported him with his OpenDTU endeavour, without his and their work I would have never been able to get the radio communication with my Hoymiles inverter working.

My solution has been up and running for about 12 months now, it over-achieves my initial goal and over-fulfils all of my criteria. I called it "BinSmart ESS" (my surname is Binder :-).

The solution looks like this:
[BinSmart_overview.pdf](https://github.com/user-attachments/files/18629652/BinSmart_overview.pdf)

Feel free to use any parts of my code and my drawing as an inspiration to start or optimize your own project. For larger installations with, say, 800 Wp (today's limit for a "Balkonkraftwerk") you would probably use a larger battery, BMS, charger and inverter, but the software config could be adapted easily.

The ESP32 code was written with Arduino IDE. I have restrained from developing a frontend app for Windows, iOS or Android. The frontend functionality is very simple, popular telnet clients like Putty or Termius are sufficient to visualize BinSmart's status and handle user commands. If the font "Cascadia Code" is chosen, all special characters are displayed correctly - see screenshots.

Some BinSmart parameters can be modified via telnet client, however the static part of the software config (e.g. IP addresses) needs to be changed in the file "BinSmart_cfg.h", which requires re-compilation and upload (via OTA) to the ESP32 after every change.

Personal note: I learned programming in the days of BASIC and Pascal, back in the 1980s. ESP32's ancestors were programmed in assembler to achieve acceptable performance. Before internet, java and object-oriented programming became mainstream, I moved to project management and didn't do any coding for 30 years. Obviously my code is not "state of the art", it's procedural and full of global variables, but it works beautifully. If anybody is willing to take my code and port it to proper C++, please go ahead!
