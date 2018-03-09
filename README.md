# COFFEE_ROASTER

<img src="/images/IMG_1713.JPG" alt="Roaster" width="420" height="315"> <img src="/images/IMG_1714.JPG" alt="Clock" width="420" height="315">

<img src="/images/IMG_1732.JPG" alt="Roaster" width="420" height="315"> <img src="/images/RoastingProfile.jpg" alt="Clock" width="420" height="315">

This repository contains the code for a PID controlled coffee roaster.  The code was designed to run on the STM32F103C8 ARM development board known as the "Blue Pill", and programmed through the Arduino IDE via USB.  There are plenty of "how to" guides on the web for uploading the bootloader and loading the STM32 package into the Arduino IDE.  For example, see: https://onetransistor.blogspot.com/2017/11/stm32-bluepill-arduino-ide.html

Note this code does not require the Blue Pill, it is simply what I used.  With minimal modifications, it certainly could be adapted to a number of other Arduino-compatible boards such as the Mega 2560 or Teensy 3.2.  It currently uses 55KB program space and 5.6KB RAM on the Blue Pill, so is unlikely to fit on smaller boards such as the Uno without significant modification to the code.

A quick summary of the features:
* Auto mode which will control the temperature (bean or environment) and fan speed to a settable profile.  
* Manual mode where temperature and fan speed can be controlled in real time.
* Measured temperatures, set value, fan speed, and heater duty cycle saved to an SD card as a CSV file at 2 Hz while roasting.  See image above showing data from a roast, plotted from the CSV file using Excel.   
* The roasting profile can be set as a sequence of up to 9 segments.  The set temperature and fan speed are linearly interpolated from one segment to the next.  This provile can be set and saved to EEPROM (flash memory on the Blue Pill) via the LCD screen interface.  
* Separate tuneable PID values for the environement temperature and bean temperature.  A tuning mode for each controller is available allowing one to monitor the step response and tune the gains in real time.

I haven't provided any schematics for the electronics on this page because I think this depends on the parts sourced for the specific project.  I can leave some bread crumbs on what I did.  I decided to use a popcorn popper for small batch coffee roasting (80 grams per roast).  I selected the Poppery II because many others before me have had success modifying this popcorn popper.  I found a used one on eBay for <$20.  If I were to do it again, I might look at finding a similar new popper for the same price, since many popcorn poppers appear to be essentially the same design.  These two websites show some details on the modification required: http://popperyii.blogspot.com/2011/01/completing-hiros-journey-poppery-ii-mod.html and https://hackaday.com/2018/01/23/build-an-excellent-coffee-roaster-with-a-satisfyingly-low-price-tag/.

A few of the parts I sourced:
* Used Poppery II popcorn popper ($19 on eBay)
* Blue Pill STM32 development board (<$3 on AliExpress)
* 20x4 character I2C LCD screen ($9 on eBay)
* 2x Adafruit Thermocouple Amplifiers, MAX31855 or MAX31856 ($15 each)
* Adafruit MicroSD Card breakout board ($8)
* Two K-type thermocouples
* Fotek SSR-40 DA Solid State Relay + Heat Sink ($12.50 on eBay)
* IRLB8721PBF MOSFET (used for fan speed control)
* 20VDC, 3A laptop power supply
* 7805 5V regulator
* Flyback Schottky diode for motor (I used an SB1245)
* Various resistors and capacitors

The DC motor is driven by the 20V power supply, with PWM speed control from the microcontroller.  The diodes on the Poppery II motor will first need to be removed.  It is vitally important that a few small capacitors be soldered directly to the motor leads to reduce voltage noise on the DC power, otherwise the inevitable voltage spikes from the DC motor will reset the microcontroller in the middle of your roast.  Not good!  I used a 22 nF capacitor across the leads, and then two 10 nF capacitors from each lead directly to the motor housing.  I also used a large 3300 uF capacitor from the motor + to ground, as well as plenty of 0.1 uF capacitors at the input of each IC power input.  The 7805 datasheet also has recommended capacitors at the input and output.  And don't forget the flyback diode, which should run from the MOSFET drain to the 20V supply close to the MOSFET (not the motor).

Two temperature measurements are used.  The Environment Temperature thermocouple is close to the heat source (at the very bottom of the roasting chamber) and the Bean Temperature prope is directed into the middle of the bean mass.  I purchased one of each type of Adafruit Thermocouple amplifier (MAX31855 and MAX31856) since I wasn't sure which would work out of the box with the Blue Pill.  It turns out they both work with a few tweaks to the libraries, so my code uses both.  The code can easily be modified to use one or the other.  Both the MAX31855 and MAX31856 libraries required a small amount of changes to work correctly.  The MAX31855 library required a slight modification to the SPI communication, while the MAX31856 required a small change to eliminate the need for the 0.25 sec delay in the code.  Both modified libraries are included in this repository.

I have also included a few 3D printed parts I used, which may be useful.  This repository includes a front panel for the LCD screen, and a mount for the Poppery II roasting chamber.   



