# COFFEE ROASTER

<img src="/images/IMG_2012.JPG" alt="Roaster" width="420"> <img src="/images/IMG_2027.JPG" alt="Roaster" width="420">

<img src="/images/profile.png" alt="Roaster">

This repository contains the code for a PID controlled coffee roaster.  The code was designed to run on the STM32F103C8 ARM development board known as the "Blue Pill", and programmed through the Arduino IDE via USB.  There are plenty of "how to" guides on the web for uploading the bootloader and loading the STM32 package into the Arduino IDE.  For example, see: https://onetransistor.blogspot.com/2017/11/stm32-bluepill-arduino-ide.html

Note this code does not require the Blue Pill, it is simply what I used.  With minimal modifications, it certainly could be adapted to a number of other Arduino-compatible boards such as the Mega 2560 or Teensy 3.2.  It currently uses 55KB program space and 5.6KB RAM on the Blue Pill, so is unlikely to fit on smaller boards such as the Uno without significant modification to the code.

A quick summary of the features:
* Auto mode which will control the temperature (bean or environment) and fan speed to a settable profile.  
* Manual mode where temperature and fan speed can be controlled in real time.
* Measured temperatures, set value, fan speed, and heater duty cycle saved to an SD card as a CSV file at 2 Hz while roasting.  See image above showing data from a roast.  I've also included a python script used to plot this data from the SD card.  The script requires the python module "matplotlib".   
* The roasting profile can be set as a sequence of up to 9 segments.  The set temperature and fan speed are linearly interpolated from one segment to the next.  This provile can be set and saved to EEPROM (flash memory on the Blue Pill) via the LCD screen interface.  
* Separate tuneable PID values for the environement temperature and bean temperature.  A tuning mode for each controller is available allowing one to monitor the step response and tune the gains in real time.

I loosly based my roaster build off of a few of the fluid bed roasters over at the Home Roaster's forum using a Pyrex Bake-A-Round baking tube and inverted coctail shaker lid.  For example, see Old Gear Head's fluid bed roaster design at: https://forum.homeroasters.org/forum/viewthread.php?thread_id=2207.   

A few of the parts I sourced:
* Blue Pill STM32 development board (<$3 on AliExpress)
* 20x4 character I2C LCD screen ($9 on eBay)
* 2x Adafruit Thermocouple Amplifiers, MAX31855 or MAX31856 ($15 each)
* Adafruit MicroSD Card breakout board ($8)
* MicroSD Card
* Two K-type thermocouples
* Metal thermocouple probe shield (e.g., look for the 5mm x 50 mm thermocouple probe from Uxcell on Amazon, and cut off the end of the metal shield)
* Fotek SSR-40 DA Solid State Relay + Heat Sink ($12.50 on eBay)
* IRLB8721PBF MOSFET (used for fan speed control)
* 7805 5V regulator
* Flyback Schottky diode for motor (I used an SB1245)
* 12V, 25A power regulated switching power supply ($19 at eBay)
* Pyrex Bake-A-Round baking tube ($20 at eBay)
* Lid from Winco 16-oz coctail shaker, 1.5" and 3.5" diam ($7.50 at Amazon)
* 1.5" x 12" chrome tube ($10 at Amazon)
* HAS-043K 1700 Watt heating element ($22 at Amazon)
* Airhead "Air Pig" 12V, 22 Amp air pump ($42 at Amazon)
* 340 Silicone O-Rings 3.5" diam
* Various resistors and capacitors

The DC motor is driven by the 12V power supply, with PWM speed control from the microcontroller.  It is vitally important that a few small capacitors be soldered directly to the motor leads to reduce voltage noise on the DC power, otherwise the inevitable voltage spikes from the DC motor will reset the microcontroller in the middle of your roast.  Not good!  I used a 22 nF capacitor across the leads, and then two 10 nF capacitors from each lead directly to the motor housing.  I also used a large 3300 uF capacitor from the motor + to ground, as well as plenty of 0.1 uF capacitors at the input of each IC power input.  The 7805 datasheet also has recommended capacitors at the input and output.  And don't forget the flyback diode, which should run from the MOSFET drain to the 12V supply close to the MOSFET (not the motor).  

The 12V, 22 Amp DC motor I used draws a lot of current, so be sure to use beefy wires (e.g., 14 AWG) all the way to the MOSFET.  Be sure to use a heat sink on the MOSFET.  Finally, although the IRLB8721PBF MOSFET gate can be driven by 3.3V (from the Blue Pill PWM output pin PA1), it has a lower Rds resistance when driving it with a higher voltage, and therefore generates less heat.  Consider bumping up the PWM to 5V or higher using a logic level converter or transistor.  

I enclosed the DC motor in my own 3D printed case and threw away the pig shaped case.  I've included the 3D printed case model in this repository, which encloses the motor and is sealed with a 3.5" O-ring.  I'm sure the pig shaped case would work too, but don't use the corrogated hose which produces too much drag.  

Two temperature measurements are used.  I placed the Environment Temperature thermocouple close to the center of the chamber in the line of fire of the hot air, and the Bean Temperature probe off to the side and blocked from the upcoming air.  A metal thermocouple shield is used to hold the probes in place.  I purchased a thermocouple probe from Uxcell on Amazon that came with a metal shield, and sawed off the end of the metal shield.  I have included a photograph below showing the thermocouple placement, and a drawing as well.  The beam temperature thermocouple and wires are taped to the top of the thermocouple metal shield using Kapton tape, which allows this thermocouple to register the temperature of the beans decending along the side of the shaker lid, and outside of the stream of hot air.  

<img src="/images/IMG_2029.JPG" alt="Roaster" width="450"> <img src="/images/TC_placement.JPG" alt="Roaster" width="450">

I purchased one of each type of Adafruit Thermocouple amplifier (MAX31855 and MAX31856) since I wasn't sure which would work out of the box with the Blue Pill.  It turns out they both work with a few tweaks to the libraries, so my code uses both.  The code can easily be modified to use one or the other.  Both the MAX31855 and MAX31856 libraries required a small amount of changes to work correctly.  The MAX31855 library required a slight modification to the SPI communication, while the MAX31856 required a small change to eliminate the need for the 0.25 sec delay in the code.  Both modified libraries are included in this repository.

I have also included a few 3D printed parts I used, which may be useful.  This repository includes a front panel for the LCD screen, and also the buttresses to hold the Bake-A-Round vertical. 

The plans for a 1/4" ply laser cut case are also included if you have access to a laser cutter.  Or build your own!  

<img src="/images/IMG_2028.JPG" alt="Roaster"> 

