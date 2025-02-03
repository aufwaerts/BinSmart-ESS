# BinSmart-ESS

This project is about a low-cost, high-efficiency energy storage solution (ESS) for small photovoltaic installations (German: "Balkonkraftwerke").

Having used a small 435 Wp "Balkonkraftwerk" for a year, it showed that only a third of the energy was used by consumers, two thirds were exported to the grid (with no financial compensation). I came up with the idea of retrofitting an ESS solution, which should at least double the proportion used by consumers. However, in 2023 I found nothing on the market that met my following criteria:
- storage capacity: at least 2 kWh
- total cost: less than €600
- charging/discharging power: at least 300 W
- indoor "AC-coupled" solution, no additional wiring to outdoor PV installation
- "zero" export to grid (less than 1% when battery not fully charged)
- silent operation (no fans)
- self-consumption of the entire system: less than 5 W
- total efficiency (AC-to-AC): higher than 80%

Available AC-coupled solutions were either expensive, inefficient, noisy or consumed a significant amount of energy themselves. I concluded that I had to build a solution myself.

There were some "home-made" solutions around, but they had their drawbacks, too:
- OpenDTU-OnBattery, a zero export solution based on OpenDTU, didn't support Meanwell chargers, and buying a Victron would have violated my cost criteria.
- The "Trucki Stick" (https://trucki.de/) would have been able to control Meanwell chargers, but on the inverter side, only Sun or Lumentree were supported by Trucki. With those inverters I wouldn't have fulfilled my efficiency & silence criteria.
- Other solutions found on various YouTube channels were not AC-coupled or didn't include controllable chargers and inverters, which would have violated my "zero export" criteria.

I must however give Thomas Basler (and everybody who supported him) credit for their OpenDTU project, without their achievements I would never have managed to get the Hoymiles radio communication working. A big "thank you" also goes to Christian Waller, who demonstrated on his excellent YouTube channel "Der Kanal" how Meanwell LED drivers can be used as controllable battery chargers.

My solution has been up and running for 12 months now, it over-achieves my initial goal and over-fulfils all of my criteria. I called it "BinSmart ESS" (my surname is Binder :-).

The solution looks like this:
[BinSmart_overview.pdf](https://github.com/user-attachments/files/18629652/BinSmart_overview.pdf)

The ESP32 code was written with Arduino IDE. I have restrained from developing a frontend app for Windows, iOS or Android. The frontend functionality is very simple, popular telnet clients like Putty or Termius are sufficient to visualize BinSmart's status and handle user commands. If the font "Cascadia Code" is chosen, all special characters are displayed correctly - see screenshots.

Some parameters can be modified on the fly, via telnet client. The software config (e.g. IP addresses) needs to be changed in the file "BinSmart_cfg.h", which requires re-compilation and upload (via OTA) after every change.

Feel free to use any parts of my code for your own project. For larger installations with, say, 800 Wp (today's limit for a "Balkonkraftwerk") you would probably use a larger battery, BMS, charger and inverter, but the software config could be adapted accordingly.

Personal note: I learned to write software in the 1980s, the days of BASIC and Pascal. ESP32's ancestors were programmed in assembler to achieve acceptable performance. Before internet, java and object-oriented programming became mainstream, I moved into project management and didn't do any coding for 30 years. My code is obviously not "state of the art", it's procedural and full of global variables, but it works beautifully. If anybody is willing to port my code to proper C++, please go ahead!
