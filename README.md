# BinSmart-ESS

This project is about an energy storage solution (ESS) for small photovoltaic installations (German: "Balkonkraftwerke").

Having used such a "Balkonkraftwerk" with a modest capacity of 435 Wp for about a year, it showed that only a third of the
energy was consumed by our household, two thirds were fed into the grid (with no compensation whatsoever). I came up with
the idea of retrofitting an ESS solution, which should at least double the amount consumed by our household.
However, back in 2023 there was nothing available on the market that met my following criteria:
- storage capacity: at least 2 kWh
- total cost: less than €600
- charging/discharging power: at least 300 W
- indoor "AC coupled" solution, no additional wiring to outdoor PV installation
- silent operation (no fans)
- self-consumption of the entire system: less than 5 W
- total efficiency (AC-to-AC): larger than 80%

The last five criteria seemed impossible to find in one system. Affordable "AC coupled" ESS solutions were typically
inefficient, noisy and/or consumed a significant amount of energy themselves. One dealer offered me a
system with a self-consumption of 20 W, which corresponds to 175 kWh p.a. (more than a third of the entire PV output).

Some smart "zero export" solutions have been around for a while (e.g. OpenDTU, Ahoy, OpenDTU-OnBattery), however they
didn't seem to meet all of my criteria either. Today I would probably base my project on OpenDTU-OnBattery, but two years ago,
support for chargers was very limited. I must however give huge credits to Thomas Basler and everyone who supported
him with his OpenDTU project, without his and their work I would have never been able to get the RF24 communication with my
Hoymiles inverter working.

My solution has been up and running for about 12 months now, it over-achieves my initial goal and over-fulfils all
of my criteria. I called it "BinSmart ESS" (my surname is Binder :-).

The solution looks like this:
[ESS_Seabed.pdf](https://github.com/user-attachments/files/18629639/ESS_Seabed.pdf)

I assume that nobody out there wants to re-build exactly the same system, after all prices for PV installations keep coming
down and a "Balkonkraftwerk" now typically produces 800 Wp. A storage capacity of 2 kWh and 300 W charging/discharging power
are probably not sufficent.

However, feel free to use any parts of my code and my drawings as an inspiration to start or optimize your own project.
I spent such an amount of time and effort on this project that it wouldn't feel right not to share the details and
the result.

The ESP32 code was written with Arduino IDE. I have restrained from developing GUIs for Windows, iOS or Android. The frontend
functionality is very simple, popular telnet clients like Putty or Termius are sufficient to visualize ESP32 output and
handle user commands. If the font "Cascadia Code" is chosen, all special characters are displayed correctly - see screenshots.

Dynamic BinSmart parameters can be modified via telnet client, however the static part of the software config
(e.g. IP addresses) needs to be changed in the "definitions" section of the source code, which requires re-compilation
and upload (via OTA) to the ESP32 after every change. A config file would of course have been slicker.

Final note: I learned programming in the days of BASIC and Pascal, back in the 1980s. Microcontrollers like Intel's 8051
were programmed in assembler to achieve acceptable performance. When object-oriented programming came along with
Internet & Java I moved to project management and didn't do any coding for almost 30 years.
My code is procedural and uses a lot of global variables, it might look awkward to the trained eye of  young OOP programmer. However it works beautifully and should only be seen as a collection of algorithms. If anybody is willing to take my code and port it to proper C++, please go ahead.



