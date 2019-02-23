# COFFEE ROASTER

<img src="/images/IMG_2012.JPG" alt="Roaster" width="420"> <img src="/images/IMG_2027.JPG" alt="Roaster" width="420">

<img src="/images/profile.png" alt="Roaster" width="850">

This repository contains the code for a PID controlled coffee roaster, run on a Teensy 3.5/3.6 development board.

A quick summary of the features:
* Auto mode which will control the temperature (bean or environment) and fan speed to a settable profile.  
* Manual mode where temperature and fan speed can be controlled in real time.
* Measured temperatures, set value, fan speed, and heater duty cycle saved to an SD card as a CSV file at 2 Hz while roasting.  See image above showing data from a roast.  I've also included a python script used to plot this data from the SD card.  The script requires the python module "matplotlib".   
* The roasting profile can be set as a sequence of up to 9 segments.  The set temperature and fan speed are linearly interpolated from one segment to the next.  This profile can be set and saved to EEPROM via the LCD screen interface.  
* Separate tuneable PID values for the environment temperature and bean temperature.  A tuning mode for each controller is available allowing one to monitor the step response and tune the gains in real time.

I loosely based my roaster build off of a few of the fluid bed roasters over at the Home Roaster's forum using a Pyrex Bake-A-Round baking tube and inverted cocktail shaker lid.  For example, see Old Gear Head's fluid bed roaster design at: https://forum.homeroasters.org/forum/viewthread.php?thread_id=2207.   

Here are the parts needed for the electronics board:
* PCB (Order using gerber files in Eagle directory, or ask me and I'll send you one for a small amount.  Alternatively, the electronics can be soldered to a stripboard)
* Teensy 3.5 (or 3.6), $24.25 from pjrc.com + MicroSD card
* 20x4 character I2C LCD screen ($9 on eBay)
* 2x MAX31855KASA+ thermocouple ICs ($5 each)
* 4x SMD ferrite beads (I used 470 ohms 0805 package)
* 3x BSS138 MOSFET transistors
* IRLB8721PBF MOSFET + heat sink (very important!!)
* 2N3904 transistor
* SB1245 Schottky diode
* 7805 5V regulator 
* Various resistors and capacitors (see schematic in Eagle directory)

Here are the additional parts I used:
* Two K-type thermocouples
* Metal thermocouple probe shield (e.g., look for the 5mm x 50 mm thermocouple probe from Uxcell on Amazon, and cut off the end of the metal shield)
* Fotek SSR-40 DA Solid State Relay + Heat Sink ($12.50 on eBay)
* 12V, 25A power regulated switching power supply ($19 at eBay)
* Pyrex Bake-A-Round baking tube ($20 at eBay)
* Lid from Winco 16-oz coctail shaker, 1.5" and 3.5" diam ($7.50 at Amazon)
* 1.5" x 12" chrome tube ($10 at Amazon)
* HAS-043K 1700 Watt heating element ($22 at Amazon)
* Airhead "Air Pig" 12V, 22 Amp air pump ($42 at Amazon)
* 340 Silicone O-Rings 3.5" diam

The DC motor is driven by the 12V power supply, with PWM speed control from the microcontroller.  It is vitally important that a few small capacitors be soldered directly to the motor leads to reduce voltage noise on the DC power, otherwise the inevitable voltage spikes from the DC motor will reset the microcontroller in the middle of your roast.  Not good!  I used a 22 nF capacitor across the leads, and then two 10 nF capacitors from each lead directly to the motor housing.   

The 12V, 22 Amp DC motor I used draws a lot of current, so be sure to use beefy wires (e.g., 14 AWG) all the way to the MOSFET.  Be sure to use a heat sink on the IRLB8721PBF MOSFET.  The wide traces on the back of the PCB are inadequate to handle 22A.  I recommend laying down a heavy layer of solder directly to these traces.  Better yet, solder exposed 14 AWG wire directly to the traces.   

I enclosed the DC motor in my own 3D printed case and threw away the pig shaped case on the Airhead "Air Pig".  I've included the 3D printed case model in this repository, which encloses the motor and is sealed with a 3.5" O-ring.  I'm sure the pig shaped case would work too, but don't use the corrugated hose which produces too much drag.  

Two temperature measurements are used.  I placed the Environment Temperature thermocouple close to the center of the chamber in the line of fire of the hot air, and the Bean Temperature probe off to the side and blocked from the upcoming air.  A metal thermocouple shield is used to hold the probes in place.  I purchased a thermocouple probe from Uxcell on Amazon that came with a metal shield, and sawed off the end of the metal shield.  I have included a photograph below showing the thermocouple placement, and a drawing as well.  The bean temperature thermocouple and wires are taped to the top of the thermocouple metal shield using Kapton tape, which allows this thermocouple to register the temperature of the beans descending along the side of the shaker lid, and outside of the stream of hot air.  

<img src="/images/IMG_2029.JPG" alt="Roaster" width="400"> <img src="/images/TC_placement.JPG" alt="Roaster" width="450">

I have also included a few 3D printed parts I used, which may be useful.  This repository includes a front panel for the LCD screen, and also the buttresses to hold the Bake-A-Round vertical. 

The plans for a 1/4" ply laser cut case are also included if you have access to a laser cutter.  Or build your own!  

<img src="/images/IMG_2028.JPG" alt="Roaster"> 

