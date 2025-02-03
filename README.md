# BinSmart-ESS

This project is about an energy storage solution (ESS) for small photovoltaic installations (German: "Balkonkraftwerke").

Having used a "Balkonkraftwerk" with a modest output of 435 Wp for a year, it showed that only a third of the energy was consumed by our household, two thirds were exported to the grid (with no financial compensation). I came up with the idea of retrofitting an ESS solution, which should at least double the proportion consumed by our household. However, in 2023 I found nothing on the market that met my following criteria:
- storage capacity: at least 2 kWh
- total cost: less than €600
- charging/discharging power: at least 300 W
- indoor "AC coupled" solution, no additional wiring to outdoor PV installation
- "zero" energy export to grid (less than 1% when battery not fully charged)
- silent operation (no fans)
- self-consumption of the entire system: less than 5 W
- total efficiency (AC-to-AC): higher than 80%

Available "AC coupled" ESS solutions were either expensive, inefficient, noisy or consumed a significant amount of energy themselves. I concluded that I had to build a solution myself.

OpenDTU-OnBattery, a "zero export" solution based on OpenDTU, looked like a good starting point, but back in 2023 there was (or still is?) no support for Meanwell chargers, and buying a Victron would have violated my cost criteria. I must however give Thomas Basler (and everybody supporting him) credit for their OpenDTU project, without their achievements I would never have managed to get the Hoymiles radio communication working.

My solution has been up and running for 12 months now, it over-achieves my initial goal and over-fulfils all of my criteria. I called it "BinSmart ESS" (my surname is Binder :-).

The solution looks like this:
[BinSmart_overview.pdf](https://github.com/user-attachments/files/18629652/BinSmart_overview.pdf)

The ESP32 code was written with Arduino IDE. I have restrained from developing a frontend app for Windows, iOS or Android. The frontend functionality is very simple, popular telnet clients like Putty or Termius are sufficient to visualize BinSmart's status and handle user commands. If the font "Cascadia Code" is chosen, all special characters are displayed correctly - see screenshots.

Some parameters can be modified via telnet client. The static part of the software config (e.g. IP addresses) needs to be changed in the file "BinSmart_cfg.h", which requires re-compilation and upload (via OTA) to the ESP32 after every change.

Feel free to use any parts of my code or my drawing for your own project. For larger installations with, say, 800 Wp (today's limit for a "Balkonkraftwerk") you would probably use a larger battery, BMS, charger and inverter, but the software config could be adapted accordingly.

Personal note: I learned programming in the days of BASIC and Pascal, back in the 1980s. ESP32's ancestors were programmed in assembler to achieve acceptable performance. Before internet, java and object-oriented programming became mainstream, I moved to project management and didn't do any coding for 30 years. Obviously my code is not "state of the art", it's procedural and full of global variables, but it works beautifully. If anybody is willing to port my code to proper C++, please go ahead!
