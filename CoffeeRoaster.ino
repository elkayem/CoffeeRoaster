/* 
 *    Coffee Roaster 
 *    Copyright (C) 2019  Larry McGovern
 *  
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License <http://www.gnu.org/licenses/> for more details.
 */
 
#include <SD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <EEPROM.h>
#include "Adafruit_MAX31855.h"
#include <Bounce2.h>

//
// Teensy 3.5/3.6 pin definitions
//
#define B_STARTSTOP 33
#define B_MODE      34
#define B_SEL       35
#define B_INC       36
#define B_DEC       37
#define FAN_PWM     2
#define SD_CS       BUILTIN_SDCARD
#define ENVTEMP_CS  9
#define BEANTEMP_CS 10
#define HEATER      32

Adafruit_MAX31855 beanTempSens(BEANTEMP_CS),envTempSens(ENVTEMP_CS);

LiquidCrystal_I2C lcd(0x27,20,4);  

Bounce startStopButton = Bounce();
Bounce modeButton = Bounce();
Bounce selButton = Bounce();
Bounce incButton = Bounce();
Bounce decButton = Bounce();

enum Modes {
  AUTO,
  MANUAL,
  SETTINGS,
  PIDTUNE_E,
  PIDTUNE_B,
  INIT
} mode;

enum EditVal {
  TEMP,
  FAN,
  CVAR,
  SEG,
  TIME,
  KP,
  KI,
  KD
} editVal;

enum ControlVar {
  ENV,
  BEAN
};

ControlVar controlVarMan = BEAN, controlVarAuto = BEAN;

// EEPROM Addresses
#define ADDR_KP_E 0  
#define ADDR_KI_E 2
#define ADDR_KD_E 4
#define ADDR_KP_B 6
#define ADDR_KI_B 8
#define ADDR_KD_B 10
#define ADDR_CVAR_MAN 12
#define ADDR_CVAR_AUTO 14
#define ADDR_SEG_TIME 16 // Address 16 - 33 (9 segments x 2 bytes)
#define ADDR_SEG_TEMP 34 // Addresses 34 - 51
#define ADDR_SEG_FAN  52 // Addresses 52 - 69

// Default gains.  These work well with a popcorn popper
#define KP_E_DEFAULT 100
#define KI_E_DEFAULT 150
#define KD_E_DEFAULT 200
#define KP_B_DEFAULT 100
#define KI_B_DEFAULT 150
#define KD_B_DEFAULT 200 

// Control gains are represented as integers ranging from 0 - 999, with LSBs below
#define KP_LSB 0.02  // LSB = 0.02% per deg, 20% per deg full range
#define KI_LSB 0.0005  // LSB = 0.0005% per deg-sec, 0.5% per deg-sec full range
#define KD_LSB 0.05   // LSB = 0.05% per deg/sec, 50% per deg/sec full range

#define BUFFERSIZE 6  // FIR Averaging filter for derivative filter.  Difference sample 6 and 1 for 5 second rolling average
class PidController {
  public:
  uint16_t Kp, Ki, Kd;
  void setPidGains(uint16_t Kp_in, uint16_t Ki_in, uint16_t Kd_in);
  double calcControl(double measTemp);
  void reset();
  
  private:
  double intErr = 0, errBuffer[BUFFERSIZE] = {0}; 
  int bIndx = 0; 
};

PidController envController, beanController;

// Profile global variables -- 9 segments used to define roasting profile for auto roast
int currentSeg = 1;
int currentSegTime;
double currentSetTemp, saveSetTemp;

// Default profile
uint16_t segTime[9] = {30,  90,  120, 60,  60,  90,  90,  330, 30}; // Time (in sec) for each segment
uint16_t segTemp[9] = {130, 210, 280, 310, 335, 360, 380, 430, 40}; // Final temperature (deg F) after completion of segment
uint16_t segFan[9]  = {85, 80, 70,  65,  65,  65,  65,  60,  100};  // Final fan speed (0 - 100) at completion of temperature

// Environment, bean, and ambient temperature measurements
double envTemp =  0; 
double beanTemp = 0;
double ambTemp = 0;

int envTempErrCtr = 0, beanTempErrCtr = 0; // Error counters
#define ERR_CTR_TIMEOUT 6

// Global variables for heat duty cycle (0-100) and fan duty cycle (0-100)
double heat = 0;
double fan = 0;

#define MIN_FAN_SPEED 50  // Minimum fan speed when heater is on, for safety

uint8_t heaterState;

unsigned long int secTimer = 0;

char filename[] = "LOG0000.CSV";
File logfile;
bool sdPresent = true;

/*
 * Setup
 */
void setup() {
  Serial.begin(115200);
  pinMode(B_STARTSTOP, INPUT_PULLUP);
  pinMode(B_MODE, INPUT_PULLUP);
  pinMode(B_SEL, INPUT_PULLUP);
  pinMode(B_INC, INPUT_PULLUP);
  pinMode(B_DEC, INPUT_PULLUP);

  pinMode(HEATER, OUTPUT);
  digitalWrite(HEATER, LOW);
  heaterState = LOW;
  
  startStopButton.attach(B_STARTSTOP);
  startStopButton.interval(25);

  modeButton.attach(B_MODE);
  modeButton.interval(25);

  selButton.attach(B_SEL);
  selButton.interval(25);

  incButton.attach(B_INC);
  incButton.interval(25);

  decButton.attach(B_DEC);
  decButton.interval(25);

  analogWriteFrequency(FAN_PWM, 10000); // 10 kHz PWM for fan control
  analogWriteResolution(12);  // 12-bit resolution on analogWrite commands
  setFanSpeed();

  lcd.init();
  lcd.setBacklight(HIGH);
  lcd.setCursor(0,1);
  lcd.print("Looking for SD Card");
  lcd.setCursor(0,3);
  lcd.print("Press SEL to bypass");

  while (SD.begin(SD_CS) != true)
  {
    if (digitalRead(B_SEL)== LOW) { // SEL pressed, bypass SD card
      sdPresent = false;
      break;
    }
    delay(10);
  }
  lcd.clear();
  
  lcd.setCursor(3, 0);
  lcd.print("COFFEE ROASTER");
  lcd.setCursor(9, 2);
  lcd.print("by");
  lcd.setCursor(3,3);
  lcd.print("Larry McGovern");

  delay(2000);
  
  // Load control gains
  bool useDefaults = false;
  uint16_t Kp = readEepromUint16(ADDR_KP_E);  
  uint16_t Ki = readEepromUint16(ADDR_KI_E);
  uint16_t Kd = readEepromUint16(ADDR_KD_E);

  // If all of the gains are zero or any are out of bounds, assume EEPROM isn't initialized
  // and use default values
  if (((Kp == 0) && (Ki == 0) && (Kd == 0)) || Kp > 999 || Ki > 999 || Kd > 999) {
    useDefaults = true;
    Kp = KP_E_DEFAULT;
    Ki = KI_E_DEFAULT;
    Kd = KD_E_DEFAULT;    
  }
  envController.setPidGains(Kp, Ki, Kd);
  
  Kp = readEepromUint16(ADDR_KP_B);  
  Ki = readEepromUint16(ADDR_KI_B);
  Kd = readEepromUint16(ADDR_KD_B);

  if (useDefaults || ((Kp == 0) && (Ki == 0) && (Kd == 0)) || Kp > 999 || Ki > 999 || Kd > 999) {
    useDefaults = true;
    Kp = KP_B_DEFAULT;
    Ki = KI_B_DEFAULT;
    Kd = KD_B_DEFAULT;    
  }
  beanController.setPidGains(Kp, Ki, Kd);

  // Load segment times and temperatures
  if (!useDefaults) {
    for (int i = 0; i < 9; i++) {
      segTime[i] = readEepromUint16(ADDR_SEG_TIME + 2*i);
      segTemp[i] = readEepromUint16(ADDR_SEG_TEMP + 2*i);
      segFan[i]  = readEepromUint16(ADDR_SEG_FAN + 2*i);
      if (segTime[i] > 900) segTime[i] = 30;
      if (segTemp[i] > 500) segTemp[i] = 100;
      if (segFan[i] > 100) segFan[i] = 100;
    }
  }
  currentSeg = 1;
  currentSegTime = segTime[0];
  currentSetTemp = (double)segTemp[0];
  saveSetTemp = currentSetTemp;

  // Load default control variable
  if (!useDefaults) {
    uint16_t cv = readEepromUint16(ADDR_CVAR_MAN); 
    if (cv == 0) controlVarMan = ENV;
    else controlVarMan = BEAN;
    cv = readEepromUint16(ADDR_CVAR_AUTO);
    if (cv == 0) controlVarAuto = ENV;
    else controlVarAuto = BEAN;
  }

  if (useDefaults) { // Save defaults to EEPROM
    mode = INIT;
    saveSettings();
  }

  mode = AUTO;
  editVal = FAN;
  
  lcd.clear();
  secTimer = millis();
  updateDisplay();
}

unsigned long elapsedTimeStart = 0;
float elapsedTime = 0.0;

bool roast = false, savedata = false, artisanPid = false, artisanPidPrev = false, endAutoRoast = false;;
int deltaFan;  // User adjust on fan speed made during auto roast

/*
 * Main Loop
 */
void loop() {
  static bool incState, decState;
  static unsigned long segTimeStart;
  static unsigned long incButtonPressTimeStamp1, decButtonPressTimeStamp1;
  static unsigned long incButtonPressTimeStamp2, decButtonPressTimeStamp2;
  static uint16_t fan0;
  static double setTemp0;
  uint16_t rate;

  getSerialCmd();  // Look for incoming serial commands from Artisan
  
  /*
   * Button Logic
   */
  startStopButton.update();
  modeButton.update();
  selButton.update();
  incButton.update();
  decButton.update();

  if (endAutoRoast) {  // Auto roast has ended.  We are waiting for the START/STOP button
                       // to be pushed to reset auto roast.  During this transition period,
                       // the fan speed can still be adjusted up and down, but all other 
                       // buttons are disabled
    if (startStopButton.fell()) { // START/STOP button was pushed
      endAutoRoast = false;
      fan = 0; 
      setFanSpeed();   // Turn off fan
      lcd.clear();
      updateDisplay(); // and update display
      return;          // Return and skip any other button logic
    }
  } // end if(endAutoRoast)
  
  if (startStopButton.fell() ||            // Called when start/stop button pressed,
      (artisanPid != artisanPidPrev)) {    // or Artisan PID On or Off command received in Manual or PID TUNE mode
    if (artisanPid != artisanPidPrev) { // Received a command from Artisan
      artisanPidPrev = artisanPid;
      if (roast == artisanPid) return;  // Roast state was already in correct state, do nothing
      roast = artisanPid;
    }
    else { // Button was pushed
      roast = !roast; // Toggle roast state
      artisanPid = roast;  // Treat a start/stop button push as if the command came from Artisan
      artisanPidPrev = artisanPid; 
    }
    if (roast) {    // If starting new roast
        elapsedTimeStart = millis();
        if (sdPresent) {
          for (uint16_t i = 0; i < 9999; i++) {  // Pick a new file name
            filename[3] = i / 1000 + '0';
            filename[4] = i / 100 % 10 + '0';
            filename[5] = i / 10 % 10 + '0';
            filename[6] = i % 10 + '0';
            if (!SD.exists(filename)) {
              // only open a new file if it doesn't exist
              logfile = SD.open(filename, FILE_WRITE);
              break;  
            }
        }
        logfile.println("Time, Control Temp, Env Temp, Bean Temp, Heat, Fan");
        logfile.flush();
      }
      if (mode == AUTO) {
        currentSetTemp = (double)segTemp[0];
        currentSegTime = segTime[0];
        fan0 = segFan[0];
        deltaFan = 0;
        setTemp0 = (double)segTemp[0];
        currentSeg = 1;
        segTimeStart = elapsedTimeStart; 
      }
    }
    else {  // End the roast
      heat = 0;  // Turn heat off, but leave fan running
      envController.reset();
      beanController.reset();
      if (sdPresent) logfile.close();
      
      if (mode == AUTO) { // Reset auto roast to initial settings
        endAutoRoast = true;  
        currentSeg = 1;  
        currentSetTemp = (double)segTemp[0];
        currentSegTime = segTime[0];   
        lcd.clear();
        updateDisplay();
      }
    }
  }  // end if (startStopButton.fell())

  if (modeButton.fell()) {  // Switch Modes
    if (!roast && !endAutoRoast) { // Only switch modes if not roasting and not in the endAutoRoast transition
      saveSettings(); // Save settings to EEPROM when switching modes
      switch (mode) {
        case AUTO:
          mode = MANUAL;
          editVal = TEMP;
          break;
        case MANUAL:
          mode = SETTINGS;
          editVal = SEG;
          saveSetTemp = currentSetTemp;
          currentSeg = 1;
          currentSegTime = segTime[0];
          currentSetTemp = (double)segTemp[0];
          break;
        case SETTINGS:
          mode = PIDTUNE_E;
          editVal = KP;
          currentSetTemp = saveSetTemp;
          break;
        case PIDTUNE_E:
          mode = PIDTUNE_B;
          break;
        case PIDTUNE_B:
          mode = AUTO;
          editVal = FAN;
          break;
        default:
          break;
      }
      lcd.clear();
      updateDisplay();
    }
  } // end if (modeButton.fell())

  if (selButton.fell()) {  // Make new selection (MANUAL and SETTINGS modes)
    switch (mode) {
      case MANUAL:
        if (editVal == TEMP)  editVal = FAN;
        else if (editVal == FAN) editVal = CVAR;
        else editVal = TEMP;
        break;
      case SETTINGS:
        if (editVal == SEG) editVal = TIME;
        else if (editVal == TIME) editVal = TEMP;
        else if (editVal == TEMP) editVal = FAN;
        else if (editVal == FAN) editVal = CVAR;
        else editVal = SEG;
        break;
      case PIDTUNE_E:
      case PIDTUNE_B:
        if (editVal == KP) editVal = KI;
        else if (editVal == KI) editVal = KD;
        else if (editVal == KD) editVal = TEMP;  
        else if (editVal == TEMP) editVal = FAN;  
        else if (editVal == FAN) editVal = KP;  
        break;  
      default:  
        break;
    }
    updateDisplay();
  } // end if (selButton.fell())

  if (incButton.fell()) { // Increment button was pushed.  Increment selection
      if (editVal == CVAR) {
        if (mode == SETTINGS) controlVarAuto = (controlVarAuto == ENV ? BEAN : ENV);
        else controlVarMan = (controlVarMan == ENV ? BEAN : ENV);
        envController.reset();
        beanController.reset();
        savedata = true;
        updateDisplay();
      }
      else if (editVal == SEG) {
        currentSeg++;
        if (currentSeg > 9) currentSeg = 9;
        currentSegTime = segTime[currentSeg-1];
        currentSetTemp = (double)segTemp[currentSeg-1];
        updateDisplay();
      }
      else {  // editVal = TEMP, FAN, TIME, KP, KI, or KD
        incState = true;
        incButtonPressTimeStamp1 = millis();
        incButtonPressTimeStamp2 = incButtonPressTimeStamp1;       
      }
  } // end if (incButton.fell())
  else if (incButton.read() == HIGH) { // Button has been released
      incState = false;
  }

  if (incState) { // used when increment button pressed while editVal = TEMP, FAN, TIME, KP, KI, or KD
    if (millis() - incButtonPressTimeStamp1 >= 3000)  // If button depressed for longer than 3 seconds, then increment faster
      rate = 50;
    else
      rate = 500;
    if (millis() - incButtonPressTimeStamp2 >= rate) {
      lcd.setCursor(0,0);
      incDecSelection(1);
      incButtonPressTimeStamp2 = millis();
    }
  }
  
  if (decButton.fell()) { // Decrement button was pushed.  Decrement selection
      if (editVal == CVAR) {
        if (mode == SETTINGS) controlVarAuto = (controlVarAuto == ENV ? BEAN : ENV);
        else controlVarMan = (controlVarMan == ENV ? BEAN : ENV);
        envController.reset();
        beanController.reset();
        savedata = true;
        updateDisplay();
      }
      else if (editVal == SEG) {
        currentSeg--;
        if (currentSeg < 1) currentSeg = 1;
        currentSegTime = segTime[currentSeg-1];
        currentSetTemp = (double)segTemp[currentSeg-1];
        updateDisplay();
      }
      else { // editVal = TEMP, FAN, TIME, KP, KI, or KD
        decState = true;
        decButtonPressTimeStamp1 = millis();
        decButtonPressTimeStamp2 = decButtonPressTimeStamp1;       
      }
  }
  else if (decButton.read() == HIGH) { // Button has been released
      decState = false;
  }

  if (decState) { // used when increment button pressed while editVal = TEMP, FAN, TIME, KP, KI, or KD
    if (millis() - decButtonPressTimeStamp1 >= 3000)
      rate = 50;
    else
      rate = 500;
    if (millis() - decButtonPressTimeStamp2 >= rate) {
      incDecSelection(-1);
      decButtonPressTimeStamp2 = millis();
    }
  }

  /*
   *  Read sensors and update display.  If roasting, update control and save to SD Card
   */

  static bool firstMeas = false;  // Temperatures measured twice every second and averaged
  static double envTempMeas1 = NAN, beanTempMeas1 = NAN;
  static double envTempAve = 0, beanTempAve = 0;

  if ((millis() - secTimer >= 500) && (firstMeas == false)) { // First measurement, called at the half-second mark
    
    readTempSensors();
    
    if (envTempErrCtr == 0)  envTempMeas1 = envTemp;    
    else envTempMeas1 = NAN;  
    
    if (beanTempErrCtr == 0) beanTempMeas1 = beanTemp;    
    else beanTempMeas1 = NAN;  

    updateDisplay();
    updateSD();
    
    firstMeas = true;
  }
  
  if (millis() - secTimer >= 1000) { // Second measurement and duty cycle update, called at the 1 second mark 
    secTimer = millis();
    
    // If the heat duty cycle > 0 and we're roasting,
    // then turn on heater at beginning of 1 second interval
    if ((heat > 0) && roast) heaterState = HIGH;
    else  heaterState = LOW;
    digitalWrite(HEATER, heaterState);  
    
    readTempSensors();
    
    if (!isnan(envTempMeas1) && (envTempErrCtr == 0) ) // Two valid measurements
      envTempAve = 0.5 * (envTempMeas1 + envTemp);    // Calculate average
    else
      envTempAve = envTemp;  // Otherwise, set to last valid measurement

    if (!isnan(beanTempMeas1) && (beanTempErrCtr == 0) ) // Two valid measurements
      beanTempAve = 0.5 * (beanTempMeas1 + beanTemp);    // Calculate average
    else
      beanTempAve = beanTemp;  // Otherwise, set to last valid measurement

    // Calculate feedback control
    heat = 0;  // Heat off by default
    if (roast) {  // If we're roating, calculate temperature set values and heater/fan duty cycles
      if (mode == AUTO) { // Auto roast follows segment profiles
        currentSegTime = (int)segTime[currentSeg-1] - (int)(millis() - segTimeStart)/1000; // Counts down in seconds from segTime to 1
        if (currentSegTime < 1) { // If we reach 0, it is time to move to the next segment
          setTemp0 = (double)segTemp[currentSeg-1];
          fan0 = segFan[currentSeg-1];
          currentSeg++;

          // End the roast if we are at the end of the 9th segment, or the next segment length is 0
          if (currentSeg > 9) endAutoRoast = true;
          else if (segTime[currentSeg-1] == 0) endAutoRoast = true;
          
          if (endAutoRoast == true) { // End the roast, and display endAutoRoast transition screen
            roast = false;
            heat = 0;    // Turn heater off
            fan = fan0;  // Turn fan speed to the final segment fan speed value
            setFanSpeed();
            currentSeg = 1;  // Reset auto roast to initial settings
            currentSetTemp = (double)segTemp[0];
            currentSegTime = segTime[0];   
            envController.reset();
            beanController.reset();
            if (sdPresent) logfile.close();
            lcd.clear();
            updateDisplay();
            return;
          }
          currentSegTime = segTime[currentSeg-1];
          segTimeStart = millis(); 
        }  // end if (currentSegTime < 1)

        // Linearly interpolate the current set temperature and fan duty cycle 
        currentSetTemp = setTemp0 + ((double)segTemp[currentSeg-1] - setTemp0) * (double)(millis() - segTimeStart)/(1000*(double)segTime[currentSeg-1]);
        fan = deltaFan + (int)((double)fan0 + ((double)segFan[currentSeg-1] - (double)fan0) * (double)(millis() - segTimeStart)/(1000*(double)segTime[currentSeg-1]));
      }
      if ((mode == PIDTUNE_E) || ((mode == MANUAL) && (controlVarMan == ENV)) || ((mode == AUTO) && (controlVarAuto == ENV))) { // Env temp control
        if (envTempErrCtr < ERR_CTR_TIMEOUT) {
          heat = envController.calcControl(envTempAve);
        }
      }
      else { // Bean temp control
        if (beanTempErrCtr < ERR_CTR_TIMEOUT) {
          heat = beanController.calcControl(beanTempAve);
        }
      }
    }
    setFanSpeed();
    updateDisplay();
    updateSD();
    firstMeas = false;
  }

  heaterDutyCycleOff(); 
}

/*
 * Save settings to EEPROM.  This is called whenever switching modes.
 */
void saveSettings() {
  uint16_t cv;
  if(savedata || (mode == INIT)) { // Save if one of the settings has changed, or if initializing EEPROM
    if ((mode == MANUAL) || (mode == INIT)) {
      if (controlVarMan == ENV)  cv = 0;
      else cv = 1;
      writeEepromUint16(ADDR_CVAR_MAN, cv);
    }
    if ((mode == SETTINGS) || (mode == INIT)) {
      if (controlVarAuto == ENV)  cv = 0;
      else cv = 1;
      writeEepromUint16(ADDR_CVAR_AUTO, cv);
      for (int i = 0; i < 9; i++) {
         writeEepromUint16(ADDR_SEG_TIME + 2*i, segTime[i]);
         writeEepromUint16(ADDR_SEG_TEMP + 2*i, segTemp[i]);
         writeEepromUint16(ADDR_SEG_FAN  + 2*i, segFan[i]);
      }
    }
    if ((mode == PIDTUNE_E) || (mode == INIT)) {
      writeEepromUint16(ADDR_KP_E, envController.Kp);
      writeEepromUint16(ADDR_KI_E, envController.Ki);
      writeEepromUint16(ADDR_KD_E, envController.Kd);
    }
    if ((mode == PIDTUNE_B) || (mode == INIT)) {
      writeEepromUint16(ADDR_KP_B, beanController.Kp);
      writeEepromUint16(ADDR_KI_B, beanController.Ki);
      writeEepromUint16(ADDR_KD_B, beanController.Kd);
    }
    if (mode != INIT) {
      lcd.clear();
      lcd.setCursor(1,1);
      lcd.print("Saving Settings...");
      delay(1000);
    }
  }
  savedata = false;
}

/*
 * heaterDutyCycleOff() -- Check to see if heater needs to be turned off
 */
void heaterDutyCycleOff() {
  uint16_t dc = (uint16_t)(10*heat);  // Duty cycle integer from 0 - 1000
  if (heat >= 100.0) return;  // Will leave heater on no matter what
  if ((millis() - secTimer >= dc) && (heaterState == HIGH)) { 
    heaterState = LOW;
    digitalWrite(HEATER, LOW);
  }
}

/*
 * readTempSensors() -- Read temperature sensors
 */
void readTempSensors() {

  ambTemp = 0.5 * (envTempSens.readInternal() + beanTempSens.readInternal());
  ambTemp = 32.0 + 1.8 * ambTemp;
  
  double envTempC = envTempSens.readCelsius();
  if (isnan(envTempC)) {  
    envTempErrCtr++;
  }
  else {
    envTemp = 32.0f + 1.8f * envTempC;
    envTempErrCtr = 0;
  }
  
  double beanTempC = beanTempSens.readCelsius();

  if (isnan(beanTempC)) {  
    beanTempErrCtr++;
  }
  else {
    beanTemp = 32.0 + 1.8 * beanTempC;
    beanTempErrCtr = 0;
  }
  
  if (envTempErrCtr >= ERR_CTR_TIMEOUT) envTemp = NAN;
  if (beanTempErrCtr >= ERR_CTR_TIMEOUT) beanTemp = NAN;
}
 
/*
 * updateSD() -- Update SD Card (called every 0.5 seconds when roasting)
 */
void updateSD() {
    if (roast && sdPresent) {  //logfile.println("Time, Env Temp, Bean Temp, Heat, Fan");
      elapsedTime = (float)(millis() - elapsedTimeStart)/1000.0f;
      logfile.print(elapsedTime,1);  
      logfile.print(", ");  
      logfile.print(currentSetTemp);       
      logfile.print(", ");
      logfile.print(envTemp,1);
      logfile.print(", ");
      logfile.print(beanTemp,1);
      logfile.print(", ");
      logfile.print(heat);
      logfile.print(", ");
      logfile.println(fan);
      logfile.flush();
    }
}
/*
 * updateDisplay() -- Update LCD
 */
void updateDisplay() {
  char timeStr[5];
  
   if (mode == SETTINGS) {  // Unique display for settings
      lcd.setCursor(0,0);
      lcd.print("AUTO SETTINGS"); 
      showSelection();
   }
    
   if (mode == AUTO) { // Unique display for AUTO
      lcd.setCursor(0,0);
      if (endAutoRoast) lcd.print("AUTO ROAST COMPLETE");
      else lcd.print("AUTO ROAST");      
   }

   if (mode == AUTO || mode == SETTINGS) { // Common for both AUTO and SETTINGS
      lcd.setCursor(1,1);
      if (endAutoRoast) lcd.print(" Press START/STOP");
      else {
        lcd.print(currentSeg);
        lcd.print("/9");
        lcd.setCursor(5,1);
        formattedTime(timeStr,currentSegTime); 
        lcd.print(timeStr);
      } 
   }
   
   if (mode == MANUAL) {
      // Unique display for MANUAL
      lcd.setCursor(0,0);
      lcd.print("MANUAL ROAST");  
      showSelection();
   }

   if ((mode == PIDTUNE_E) || (mode == PIDTUNE_B)) {
      // Unique display for PIDTUNE
      char str[3];
      uint16_t Kp, Ki, Kd;
      lcd.setCursor(0,0);
      if (mode == PIDTUNE_E) {
        lcd.print("ENV ");
        Kp = envController.Kp;
        Ki = envController.Ki;
        Kd = envController.Kd;
      }
      else {
        lcd.print("BEAN ");
        Kp = beanController.Kp;
        Ki = beanController.Ki;
        Kd = beanController.Kd;
      }
      lcd.print("PID TUNE"); 
      lcd.setCursor(1,1);
      lcd.print("P");
      sprintf(str,"%03d", Kp);
      lcd.print(str);
      lcd.setCursor(6,1);
      lcd.print("I");
      sprintf(str,"%03d", Ki);
      lcd.print(str);
      lcd.setCursor(11,1);
      lcd.print("D");
      sprintf(str,"%03d", Kd);
      lcd.print(str);
      lcd.setCursor(16,1);
      lcd.print("T");
      sprintf(str,"%03d", (int)currentSetTemp);
      lcd.print(str);
      
      showSelectionPID();
   }

   if ((mode == MANUAL) || (mode == AUTO) || (mode == SETTINGS)) {
      if (!endAutoRoast) {
        lcd.setCursor(12,1); 
        lcd.print("ST");
        lcd.setCursor(15,1);
        printTemp(currentSetTemp,0);
      }
      
      lcd.setCursor(11,2);
      if (((mode == MANUAL) && (controlVarMan == ENV)) || ((mode != MANUAL) && (controlVarAuto == ENV))) lcd.print("*");
      else lcd.print(" ");
      lcd.setCursor(11,3);
      if (((mode == MANUAL) && (controlVarMan == ENV)) || ((mode != MANUAL) && (controlVarAuto == ENV))) lcd.print(" ");
      else lcd.print("*");        
   }     

   if (mode != SETTINGS) {
      if (!endAutoRoast) {
        lcd.setCursor(15,0);
        formattedTime(timeStr,(int)elapsedTime); 
        lcd.print(timeStr); 
      }
      
      lcd.setCursor(0,2);
      lcd.print("HEAT");
      lcd.setCursor(5,2);
      printPercent((int)heat);
   }

   lcd.setCursor(0,3);
   lcd.print("FAN");
   lcd.setCursor(5,3);
   if (mode == SETTINGS) printPercent(segFan[currentSeg-1]);
   else printPercent((int)fan); 
 
   lcd.setCursor(12,2);
   lcd.print("ET");
   lcd.setCursor(15,2);
   printTemp(envTemp,envTempErrCtr); 
    
   lcd.setCursor(12,3);
   lcd.print("BT"); 
   lcd.setCursor(15,3);
   printTemp(beanTemp,envTempErrCtr);  
}

void formattedTime(char *t, int seconds)
{
  int minutes = seconds / 60;
  seconds -= minutes * 60;
  
  sprintf(t, "%s%d%s%d", minutes < 10 ? " " : "", minutes, 
          seconds < 10 ? ":0" : ":", seconds);  // Minutes, seconds
}

void printTemp(double temp, int errCtr) {
  if (errCtr < ERR_CTR_TIMEOUT) {
    if (temp < 100)  lcd.print(" ");
    lcd.print((int)temp);
  }
  else  lcd.print("---");
  lcd.print((char)223); // Deg symbol
  lcd.print("F");
}

void printPercent(int p) {
  if (p < 100) lcd.print(" ");
  if (p < 10)  lcd.print(" ");
  lcd.print(p);
  lcd.print("%");
}

/*
 * showSelection() -- Move the ">" character whenever the SEL button is pressed, indicating which
 *                    variable can be edited
 */
void showSelection() {
  switch (editVal) {
    case SEG:
      lcd.setCursor(0,1);
      lcd.print(">");
      lcd.setCursor(4,1);
      lcd.print(" ");
      lcd.setCursor(14,1); 
      lcd.print(" ");
      lcd.setCursor(10,2);
      lcd.print(" ");
      lcd.setCursor(4,3);
      lcd.print(" ");
      lcd.setCursor(10,3);
      lcd.print(" ");
      break;
    case TIME:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(4,1);
      lcd.print(">");
      lcd.setCursor(14,1); 
      lcd.print(" ");
      lcd.setCursor(10,2);
      lcd.print(" ");
      lcd.setCursor(4,3);
      lcd.print(" ");
      lcd.setCursor(10,3);
      lcd.print(" ");
      break;    
    case TEMP:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(4,1);
      lcd.print(" ");
      lcd.setCursor(14,1); 
      lcd.print(">");
      lcd.setCursor(10,2);
      lcd.print(" ");
      lcd.setCursor(4,3);
      lcd.print(" ");
      lcd.setCursor(10,3);
      lcd.print(" ");
      break; 
    case FAN:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(4,1);
      lcd.print(" ");
      lcd.setCursor(14,1); 
      lcd.print(" ");
      lcd.setCursor(10,2);
      lcd.print(" ");
      lcd.setCursor(4,3);
      lcd.print(">");
      lcd.setCursor(10,3);
      lcd.print(" ");
      break; 
    case CVAR:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(4,1);
      lcd.print(" ");
      lcd.setCursor(14,1); 
      lcd.print(" ");
      lcd.setCursor(10,2);
      lcd.print(">");
      lcd.setCursor(4,3);
      lcd.print(" ");
      lcd.setCursor(10,3);
      lcd.print(">");
      break;
    default:
      break;
  }
}

/*
 * showSelectionPID() -- Show Selection function for PID tuning screen
 */
void showSelectionPID() {
   switch (editVal) {
    case KP:
      lcd.setCursor(0,1);
      lcd.print(">");
      lcd.setCursor(5,1);
      lcd.print(" ");
      lcd.setCursor(10,1);
      lcd.print(" ");   
      lcd.setCursor(15,1); 
      lcd.print(" ");  
      lcd.setCursor(4,3);
      lcd.print(" ");
      break;
    case KI:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(5,1);
      lcd.print(">");
      lcd.setCursor(10,1);
      lcd.print(" ");   
      lcd.setCursor(15,1); 
      lcd.print(" ");  
      lcd.setCursor(4,3);
      lcd.print(" ");  
      break; 
    case KD:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(5,1);
      lcd.print(" ");
      lcd.setCursor(10,1);
      lcd.print(">");   
      lcd.setCursor(15,1); 
      lcd.print(" ");  
      lcd.setCursor(4,3);
      lcd.print(" ");
      break;
    case TEMP:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(5,1);
      lcd.print(" ");
      lcd.setCursor(10,1);
      lcd.print(" ");   
      lcd.setCursor(15,1); 
      lcd.print(">");  
      lcd.setCursor(4,3);
      lcd.print(" ");
      break;
    case FAN:
      lcd.setCursor(0,1);
      lcd.print(" ");
      lcd.setCursor(5,1);
      lcd.print(" ");
      lcd.setCursor(10,1);
      lcd.print(" ");   
      lcd.setCursor(15,1); 
      lcd.print(" ");  
      lcd.setCursor(4,3);
      lcd.print(">");
      break;
    default:
      break;
   }
}

/*
 * incDecSelection(incDec) -- Increment or decrement selection
 */
void incDecSelection(int incDec) { 
  uint16_t Kp, Ki, Kd;
  
  switch (editVal) {
    case TIME:
      currentSegTime += 5*incDec;
      if (currentSegTime < 0) currentSegTime = 0;
      if (currentSegTime > 900) currentSegTime = 900;
      segTime[currentSeg-1] = currentSegTime;
      savedata = true;
      break;
    case TEMP:
      double tempIncSize;
      if ((mode == PIDTUNE_E) || (mode == PIDTUNE_B))
        tempIncSize = 10; // Use larger step sizes when tuning PID for sharper step response
      else tempIncSize = 5;
      currentSetTemp += tempIncSize*incDec;
      if (currentSetTemp > 500) currentSetTemp = 500;
      else if (currentSetTemp < 20) currentSetTemp = 20;
      if (mode == SETTINGS) {
        segTemp[currentSeg-1] = (uint16_t)currentSetTemp;
        savedata = true;
      }
      break;
    case FAN:
      if (mode == SETTINGS) {
        if (segFan[currentSeg-1] == 0 && incDec < 0) break;
        segFan[currentSeg-1] += 5*incDec;
        if (segFan[currentSeg-1] > 100) segFan[currentSeg-1] = 100;
        savedata = true;
      }
      else {
        if (roast) {  // If roasting,increment fan 1% at a time for precision
          deltaFan += incDec;
          fan += incDec;          
        }
        else fan += 5*incDec;  // Otherwise increment fan 5% at a time
        if (fan > 100) fan = 100;
        else if (fan < 0) fan = 0;
        setFanSpeed();
      }
      break;
    case KP:
      if (mode == PIDTUNE_E) Kp = envController.Kp;
      else Kp = beanController.Kp;
      if (Kp == 0 && incDec < 0) break;
      Kp += incDec;
      if (Kp > 999) Kp = 999;
      if (mode == PIDTUNE_E) envController.Kp = Kp;
      else beanController.Kp = Kp;
      savedata = true;
      break;
    case KI:
      if (mode == PIDTUNE_E) Ki = envController.Ki;
      else Ki = beanController.Ki;
      if (Ki == 0 && incDec < 0) break;
      Ki += incDec;
      if (Ki > 999) Ki = 999;
      if (mode == PIDTUNE_E) envController.Ki = Ki;
      else beanController.Ki = Ki;
      savedata = true;
      break;   
    case KD:
      if (mode == PIDTUNE_E) Kd = envController.Kd;
      else Kd = beanController.Kd;
      if (Kd == 0 && incDec < 0) break;
      Kd+= incDec;
      if (Kd > 999) Kd = 999;
      if (mode == PIDTUNE_E) envController.Kd = Kd;
      else beanController.Kd = Kd;
      savedata = true;
      break; 
    default:
      break;
  }
  updateDisplay();
}

/*
 * setFanSpeed() -- Set fan duty PWM duty cycle to the current value of global variable fan
 */
void setFanSpeed() {
  if ((heat > 0) && (fan < MIN_FAN_SPEED)) {  // Always make sure fan is on and at minimum fan speed if the heater is on
      fan = MIN_FAN_SPEED;
  }
  uint16_t fan_12b = (uint16_t) (fan * 40.96); // Map fan speed (0-100) to 12-bit integer (0 to 4095), or 4096 to force pin high
  if (fan_12b > 4096) fan_12b = 4096;           
  analogWrite(FAN_PWM, fan_12b);
}

/*
 * PID Controller functions
 */
double PidController::calcControl(double measTemp) {
  double err = currentSetTemp - measTemp;
 
  intErr += err;  // Integrated error

  errBuffer[bIndx] = err;
  bIndx = (bIndx+1)%BUFFERSIZE; // Increment circular buffer index
  double dErr = (err - errBuffer[bIndx])/(BUFFERSIZE-1); // Rate error averaged over (BUFFERSIZE-1) frames

  double intErrLim = 100 / (KI_LSB*Ki);
  if (intErr > intErrLim) intErr = intErrLim;  // Don't allow integrated error to contribute more than 100% duty cycle
  if (intErr < 0) intErr = 0;
      
  double ctrlVal = KP_LSB * Kp * err + KI_LSB * Ki * intErr + KD_LSB * Kd * dErr;

  if (ctrlVal > 100) ctrlVal = 100;
  if (ctrlVal < 0) ctrlVal = 0;

  return ctrlVal;
}      

void PidController::setPidGains(uint16_t Kp_in, uint16_t Ki_in, uint16_t Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in; 
}

void PidController::reset() {
   intErr = 0;
   for (int i=0; i < BUFFERSIZE; i++) errBuffer[i] = 0;
}

/*
 * getSerialCmd() -- Look for incoming serial commands from Artisan, and parse as needed
 * The coffee roaster will need to be configured as a TC4.  
 * 
 * Current TC4 commands supported:
 * READ -- reply: ambTemp, envTemp, beanTemp, 0, 0, heat, fan, currentSetTemp
 * PID;ON -- Turns on roast if in Manual or PID tune mode
 * PID;OFF -- Turns off roast if in Manual or PID tune mode
 * PID;SV;vvv -- Sets control temperature to vvv
 * DCFAN;vvv -- Sets fan duty cycle to vvv
 * CHAN, UNIT, FILT -- Not supported, but Artisan expects a "#" reply so this is sent
 * 
 * All other commands currently not supported
 */
#define MAXSERIALCMD 40   // Maximum length of commands from Artisan  
void getSerialCmd(){
  static char serialCmd[MAXSERIALCMD];
  static int indx = 0;
  char *tok;
  
  if (Serial.available() > 0) {
    char c = Serial.read();
    serialCmd[indx] = toupper(c);
    
    if ((c == '\n') || (c == '\r')){  // Time to parse the command
      serialCmd[indx] = '\0';  // Null-terminate string
      
      if (strncmp(serialCmd,"READ",4)==0) {   // READ -- return data to Artisan
        Serial.print(ambTemp); Serial.print(",");
        Serial.print(envTemp); Serial.print(",");
        Serial.print(beanTemp); Serial.print(",");
        Serial.print("0,0,");
        Serial.print(heat); Serial.print(",");
        Serial.print(fan); Serial.print(",");
        Serial.println(currentSetTemp);
      }
      
      else if ((strncmp(serialCmd,"CHAN",4)==0) |  // CHAN, UNITS, and FILT commands
               (strncmp(serialCmd,"UNIT",4)==0) |  // are not supported by Coffee Roaster,
               (strncmp(serialCmd,"FILT",4)==0)){  // but Artisan still expects a "#" reply
        Serial.println("#");
       }
       
       else if (strncmp(serialCmd,"PID",3)==0) {   // PID command
         tok = strtok(serialCmd,";, "); // Pointer to first string "PID"
         tok = strtok(NULL,";, ");      // Pointer to second string

         if (strncmp(tok,"ON",2)==0) { // PID;ON command -- Start roast if in Manual or PID Tune mode
           if ((mode == MANUAL) || (mode == PIDTUNE_B) || (mode == PIDTUNE_E)) artisanPid = true;
         }

         else if (strncmp(tok,"OFF",2)==0) { // PID;OFF command -- Stop roast if in Manual or PID Tune mode
           if ((mode == MANUAL) || (mode == PIDTUNE_B) || (mode == PIDTUNE_E)) artisanPid = false;
         }
         
         if (strncmp(tok,"SV",2)==0) {       // PID;SV;vvv -- Set temperature command if in Manual or PID Tune mode
           tok = strtok(NULL,";, ");  // Pointer to third string
           if ((mode == MANUAL) || (mode == PIDTUNE_B) || (mode == PIDTUNE_E)) currentSetTemp = atof(tok);
         }
       }
       else if (strncmp(serialCmd,"DCFAN",3)==0) { // DCFAN;vvv -- Set fan duty cycle to vvv
         tok = strtok(serialCmd,";, "); // Pointer to first string "PID"
         tok = strtok(NULL,";, ");      // Pointer to second string  
         if ((mode == MANUAL) || (mode == PIDTUNE_B) || (mode == PIDTUNE_E)) fan = atof(tok);
       }
       indx = 0; // Set index to zero in preparation for a new command
    }
    else if (indx <= MAXSERIALCMD - 2) indx++;
  }
}


// Read two consecutive EEPROM addresses and return uint16
inline uint16_t readEepromUint16(int addr) {
  return (((uint16_t)EEPROM.read(addr + 0) << 8) | (uint16_t)EEPROM.read(addr + 1));
}
// Write uint16 to two consecutive EEPROM addresses
inline void writeEepromUint16(int addr, uint16_t data) {   
  EEPROM.write(addr,   (uint8_t)(data >> 8));
  EEPROM.write(addr+1, (uint8_t)data);
}
