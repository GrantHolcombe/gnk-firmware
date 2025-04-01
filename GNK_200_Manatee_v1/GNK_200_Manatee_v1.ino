//Manatee GNK-200 firmware, edited by Will Terry from Beef Squad
//With pieces/ideas borrowed from Torukmakto's T-19 code and Matthew Bregg's Orb Weaver code
//Features and functionality description:
//Cache trigger mode so that if you pull trigger at least one dart comes out or X in burst mode, or add up burst/semi totals
//Use timers instead of delays wherever possible
//Dynamic rev/braking tracking for fast spinup no matter what the blaster is doing
//Rev shelf for quicker followup shots within ~1 second
//hang mode for a short time so flywheels stay at full rev 
//Uses same pinouts as the original GNK-200 code to make swapping easier
//Default speeds were tested with a fully charged 3s lipo, using a different battery will need speed re-testing currently.

//Changelog:
//V0.1 - First beta release, includes all base features I wanted for a normally wired and single-trigger wired GNK-200
//V0.2 - Second beta release, focusing on bug fixes/stability and tuning of constants for ideal performance.
//V0.3 - Third beta release, small updates from feedback.
//v1.0 - First full release, code is tuned and on 20+ blasters, no known issues at this time.

//Next planned Release:
//v1.1 - Updates from broader feedback?

// Libraries, make sure you have the latest blaster_save_eeprom version in the same arduino folder as this code or this won't compile.
#include <Servo.h> //For controlling flywheels
#include <debounce.h>
//#include "blaster_save_eeprom.h" //To allow us to save the settings, reminder be careful with this as you only get ~100,000 reads/writes total per register.

//---------------------------------Editable section of the code -------------------------------------

//Programming Preset Options, most likely what you'll want to edit. Current values are for testing.
//Tested values are averages with axisflying motors, Machined wheels, PCAR, fresh worker 1.0g darts, and extra .5mm cage spacing. Your results may vary
//1400 =~100fps /1450 =~120fps /1500 =~140 /1550 = 
//1600 =~150fps /1650 = /1700 =~160fps /1750 =~fps /1800 =~fps /1850 =~fps /1900 =~fps /1950 =~195fps /2000 =~200fps
const int speed1 = 1425;  //default 1425 for HvZ 110-120fps
const int speed2 = 1975;  //default 1975 for Comp ~190-195fps
const int speed3 = 2000;  //default 2000 for Comp ~195-200fps

//RPM settings that correspond to the above speeds.
const int dps1 = 8;       //HvZ Speed
const int dps2 = 14;      //Stable, current settings' max
const int dps3 = 14;      //Stable, current settings' max

//Editable Constants:
const int burstSetting = 2; //Amount of darts that will be fired in burst mode. Default 2
const int minFullAutoFire = 1; //the minimum amount of darts full auto mode fires on a quick trigger pull. Can be 0-60. Default = 1.
const int batteryCells = 3; //for tracking voltage changes in the future. Default 3.
const bool alwaysEnforceDPS = true; //false will only enforce dps in full auto, true will enforce dps in all modes. Default true. //WT not actually implemented yet
const int motorMax = 2000;  // Motor Maximum speed check, don't set above 2000. Just to keep the signal from being funny.

//Constants I'm still playing with
const int solenoidOn = 36;        // Solenoid  On Delay, default 36ms. (Using a Neutron solenoid with both springs, you may need to adjust these values if you are using a different solenoid or spring)
const int solenoidOff = 36;       // Solenoid  Off Delay, default 36ms.
const int solenoidCold = 8;       //Added to the first shot of the solenoid, default 8ms.
int revTime = 330;                //Amount of time in ms the flywheels need to rev, might take motor speed into account later which would make this not a constant. Default 220ms
const int hangTime = 700;         //Amount of time in ms the flywheels hang at full power after a shot. default = 500  15951ms/half second.
const double shelfPowerPct  = .3;     // Power percent for the rev-down shelf. Default 60%. DO NOT set below what would be 1050
const double shelfPowerAmt  = 1075;      //Power amount for rev-down shelf, default 0(uses Pct instead). If you want the shelfPower to be the same for all speeds set it here.
const long shelfDuration = 0;       // Amount of time in ms to hold the rev-down shelf. Default 3000ms(three seconds) or 180000ms(3 minutes)  //Turning it off for now as it was annoying
const int shelfOffset = revTime*(shelfPowerPct);  // Amount of time it takes to rev from shelf back to full in ms. Can hardcode later if the math is off
const int brakingConstant = 1;   // Size of braking step-downs, lower number = slower braking, higher number = faster braking
const int startupDelay = 4500;   //Startup delay in ms to let the ESC's startup. Can be edited if you have a shorter or longer startup tone. Default = 3000.

// Switch Pins
#define REV 3 //No rev switch needed for this code, but if you do have a rev switch installed change the next line to TRUE.
const bool revInstalled = false;
#define TRIGGER 4
#define PROG_1 5 //Right side Switch. Swap REV and SELECT pins if you're lefty or physically swap the switches
#define PROG_2 6
#define SELECT_1 11 //Left side Switch.
#define SELECT_2 12

// Solenoid Pin
#define MOSFET 2 //Pin for the mosfet signal

//--------------------------------Edit code past here at your own risk--------------------------

//For state-based logic and whatever

int speedSetting;  //Current flywheel power setting, should be static in normal operation
int dpsSetting;    //Current dps setting, should be static in normal operation
int shotDelay; //calculated off of DPS and solenoid timings
String revState = "idle";       // What da flywheels doin?     Options are: "idle", "pre-rev", spooling", "powered", "hang", "highBraking" "shelf"
int long timeInRevState = 0;    // How long the flywheels been doin it?
String pushState = "idle";      // What da pusher doin?        Options are: "idle", "thrusting", "retracting"
int long timeInPushState = 0;   // How long the pushers been doin it?
int wheelSlowing = 1000;        //Variable for tracking the braking
int shelfMotor;                 //Variable that represents the actual esc setting for the shelf power
const int escLow = 1000;        //Minimum reference for the ESC speed, effectively '0'. Do not change unless you know how ESC communication works.
const int escHigh = 2000;       //max reference for the ESC speed, effectively 100%. Do not change unless you know how ESC communication works.
bool firstRun = true;
int long shelfTime = 0; //logging how long we need to be in the shelf for
int revTriggerState = LOW;
int fireSetting; 
int owedDarts; 
int firedDarts;

// ESCs
Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

static void triggerHandler(uint8_t btnId, uint8_t btnState) {
  if(millis() < startupDelay){ //debouncing the trigger initialization for the first 4 seconds on startup
    owedDarts = 0;
    return;
  }

  if (btnState != BTN_PRESSED) { //This is actually inverted because pullup stuff
    if(fireSetting == 3){
        if(firedDarts < minFullAutoFire){ //If we're in full auto and have not fired any darts(very quick trigger tap) make sure we fire a set number
          owedDarts = minFullAutoFire;
        } else {
          owedDarts = 0;  //Otherwise set the count to 0
        }
    }
  } else {
    switch(fireSetting){  //Case statement for different fire modes. Checking this here shoulf allow switching fire modes while firing without causing issues.
      case 1: //semiAuto  //swap 1 and 3 if your switch is backwards, but also switch the firesetting above from 3 to 1
        owedDarts++;
        break;
      case 2: //Burst
        owedDarts += burstSetting;
        break;
      case 3: //fullAuto
        owedDarts = 60;
        break;
      default: //Something wierd happened, default to Semi-auto
        owedDarts++;
        break;
    }
  }
}
static void revHandler(uint8_t btnId, uint8_t btnState) {
  if(!revInstalled){
    return;
  }
  if(millis() < 4000){ //debouncing the rev trigger initialization for the first 4 seconds on startup
    owedDarts = 0;
    return;
  }
  if (btnState == BTN_PRESSED) {
    revTriggerState = LOW;
  } else {
    revTriggerState = HIGH;
  }
}

static void progHandler1(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    progRobust();
    //Serial.println("prog1 depressed");
  } else {
    //Serial.println("prog1 pressed");
     progRobust();
  }
}
static void progHandler2(uint8_t btnId, uint8_t btnState) {
  if (btnState == BTN_PRESSED) {
    //Serial.println("prog2 depressed");
    progRobust();
  } else {
   // Serial.println("prog2 pressed");
    progRobust();
  }
}

static Button triggerButton(0, triggerHandler);  //0 is the unique buttonID and triggerHandler is the handler function
static Button revButton(1, revHandler);
static Button progButton1(2,progHandler1);
static Button progButton2(3,progHandler2);

int counter = 0;

void setup() {

  ESC1.attach(7, escLow, escHigh); //ESC's on pins 7-10, all should have the same value though
  ESC2.attach(8, escLow, escHigh);
  ESC3.attach(9, escLow, escHigh);
  ESC4.attach(10, escLow, escHigh);
  setESC(escLow); //Make sure we send initialization to the ESCs so they can boot correctly

 delay(50);

  pinMode(TRIGGER, INPUT_PULLUP); //Initializing all of the pins to their respective type
  pinMode(REV, INPUT_PULLUP);  //again this is not required
  pinMode(PROG_1, INPUT_PULLUP);
  pinMode(PROG_2, INPUT_PULLUP);
  pinMode(SELECT_1, INPUT_PULLUP);
  pinMode(SELECT_2, INPUT_PULLUP);
  pinMode(MOSFET, OUTPUT);
  
//  pinMode(LED_BUILTIN, OUTPUT); //Onboard LED for testing

  //Set rising and falling intervals all to 20ms
  triggerButton.setPushDebounceInterval(20); // [ms] Should be able to go up to 40ms without missing any inputs
  revButton.setPushDebounceInterval(20);
  progButton1.setPushDebounceInterval(20);
  progButton2.setPushDebounceInterval(20);
  triggerButton.setReleaseDebounceInterval(20); 
  revButton.setReleaseDebounceInterval(20);
  progButton1.setReleaseDebounceInterval(20);
  progButton2.setReleaseDebounceInterval(20);

  //Serial.begin(9600); //Serial Test code
  //delay(500);
  //Serial.println("booting...");

  progRobust(); //read the programming switch once

  shelfMotor = shelfMotorCalc();
  shotDelay = shotDelayCalc(); //Some math to figure out the shot delay

  delay(50); //give the outputs/inputs aquick 50ms to settle.
  pollButtons(); //give those buttons a quick read
  delay(startupDelay);    //Give the ESC's a chance to start up
  owedDarts = 0;  //Start us at -1 darts to fire before we go into main because of debounce errors

}

int shelfMotorCalc(){
  if(shelfPowerAmt > 0){
    return shelfPowerAmt;
  } else {
    return (((speedSetting-escLow)*shelfPowerPct)+escLow); //Calculating the actual motor speed for the shelf based on the speedsetting and shelfPower constant
  }

}

int shotDelayCalc(){
  int calc = (1000/dpsSetting) - solenoidOn - solenoidOff;
  if(calc < 0){
    return 0;
  }
  return calc;
}

void setESC(int speedValue){   //This way we only need one line for setting the ESC's and we can have easy input validation
  if(speedValue < escLow) { //Too low input validation
    speedValue = escLow;
  }
  if(speedValue > motorMax){ //Too high input validation
    speedValue = motorMax;
  }
  ESC1.write(speedValue); //Send same thing to all ESC's
  ESC2.write(speedValue);
  ESC3.write(speedValue);
  ESC4.write(speedValue);
  //Serial.print("/Flywheels to: " + String(speedSetting));
}

// Check Programming Switch(Right switch by default)
int programmingMode() { 
  if (digitalRead(PROG_1) == HIGH && digitalRead(PROG_2) == LOW) {  // mode 2
    return 1; //Normal mode
  } else if (digitalRead(PROG_1) == HIGH && digitalRead(PROG_2) == HIGH) {  // mode 2
    return 2; //dps prog
  } else if (digitalRead(PROG_1) == LOW && digitalRead(PROG_2) == HIGH) {  // mode 2
    return 3; //fps prog
  }
}

// Check Select Fire Switch
int selectFire() {
  if (digitalRead(SELECT_1) == HIGH && digitalRead(SELECT_2) == LOW) {  // Semi Auto
    return 3;
  } else if (digitalRead(SELECT_1) == HIGH && digitalRead(SELECT_2) == HIGH) {  // Burst Mode
    return 2;
  } else if (digitalRead(SELECT_1) == LOW && digitalRead(SELECT_2) == HIGH) {  // Full Auto
    return 1  ;
  }
}

void fire(){ //Use a bunch of timers and state checks to fire darts

  setESC(speedSetting);  //The flywheels stay ON during fire()!

  //Rev up Controls
  if(revState == "idle"){  //The flywheels are not moving at all, let's get them going
    revState = "spooling";  //We'll only get here right when the trigger is pressed so no need to check a timer before moving to next state
    timeInRevState = millis(); //Coming from a standstill so use full rev timer
  }
  if(revState == "shelf"){
    revState = "spooling";
    timeInRevState = millis() - shelfOffset; //
  }
  if(revState == "highBraking"){ //The flywheels are already moving above shelf speed, shorter delay than from idle
    revState = "spooling";
    timeInRevState = millis() - shelfOffset; //Could be anywhere between full power and shelf, just use the shelf offset for now
  }
  if(revState == "pre-rev"){ // the flywheels have already started
      revState = "spooling"; //this timer would work the same as the spooling time, just switch to spooling with no timer update, ez.
  }
  if(revState == "hang"){   
    revState = "powered";
    pushState = "idle";  //Testing if we always want the pushstate to start fresh if we're just spooling up
    timeInRevState = millis();  //I don't think we need to know how long we've been powered but we might as well store it for failure checks
    timeInPushState = millis(); //We need to initialize the time_in_push_state so there's no delay on the first shot
  }
  if(revState == "spooling"){
    if((timeInRevState + revTime) < millis()){ //check if we're spooled up, we can change state to "powered" so the solenoid knows we're good to fire
      revState = "powered";
      pushState = "idle";  //Testing if we always want the pushstate to start fresh if we're just spooling up
      timeInRevState = millis();  //I don't think we need to know how long we've been powered but we might as well store it for failure checks
      timeInPushState = millis(); //We need to initialize the time_in_push_state so there's no shot delay on the first shot
    }
  }
  
  //Solenoid Controls 
  if(revState == "powered"){ //If the flywheels are powered we're good to do pusher stuff
    if(pushState == "idle"){  //Pusher isn't moving
        digitalWrite(MOSFET, HIGH);
        pushState = "thrusting";
        timeInPushState = millis();
    }
    if(pushState == "thrusting"){ //Pusher is going forward
      if((timeInPushState + solenoidOn + timingMod() + (shotDelay/2)) < millis()){  //If it's all the way forward retract it
        digitalWrite(MOSFET, LOW);
        pushState = "retracting";
        timeInPushState = millis();
      }
    }
    if(pushState == "retracting"){  //Pusher is coming back
      if((timeInPushState + solenoidOff + (shotDelay/2)) < millis()){ //Checking if pusher is all the way back now, also use this timer to track shot delay for DPS limiting so we don't compromise the first shot
        owedDarts--;   //We have fired One(1) dart, count it
        firedDarts++;
        pushState = "idle"; //Go back to idle and let the main loop sort out whether we need to fire another dart. Technically might introduce a few microseconds of delay but the testing done on the solenoid timings should cancel this out so whatever
        timeInPushState = millis(); 
      }
    }
  } else {
    digitalWrite(MOSFET, LOW); //Failsafe in case the flywheels aren't powered because we should not be pushing a dart
  }

  wheelSlowing = speedSetting; //Ensure that whenever we go to brake the wheelSlowing is up to date
}

void revDown(){  //Controls the rev braking and the rev shelf
  
  digitalWrite(MOSFET, LOW); //Failsafe in case we somehow end up braking with the mosfet powered
  pushState = "idle";        //and make sure we record idle
  if(revState == "idle"){ //If we're already at idle then we're good, failsafe the flywheels and return to main
    setESC(escLow);
    return;
  }
  if(revState == "pre-rev"){ //only time we aren't firing darts but need to be powered.
    if(revTriggerState == HIGH){
      wheelSlowing = speedSetting;
      setESC(speedSetting);
      return; //No need to check the rest of the revDown method as we're not technically revving down
    } else {
      revState = "hang";
      timeInRevState = millis();
    }
  }
  if(revState == "powered"){ //We have just entered braking from full speedSetting
    revState = "hang";
    timeInRevState = millis();
  }
  if(revState == "spooling"){ //We have just entered braking from spooling mode? shouldn't ever happen but need to cover all cases
    wheelSlowing = speedSetting; //Just pretend like we're coming from full power, whatever
    revState = "hang";
    timeInRevState = millis();
  }
  if (revState == "hang"){ //stay at full power for a short time so the flywheels don't try to brake between quick shots
    setESC(speedSetting);
    if((timeInRevState + hangTime) < millis()){ //We've been hung for long enough, start the braking process
      revState = "highBraking";
      wheelSlowing = speedSetting;
      timeInRevState = millis();
    }
  }
  if(revState == "highBraking"){  //braking down from full power to shelf speed
    wheelSlowing -= brakingConstant;
    delay(2); //adding in some slight delay, not enough to mess with input signals though
    //Serial.print(" /Braking Constant:" + String(wheelSlowing));
    setESC(wheelSlowing);
    if(wheelSlowing <= shelfMotor){ //if we've slowed down to shelf speed move to shelf state
      revState = "shelf";
      timeInRevState = millis();
      shelfTime = millis();
    }
  }
  if(revState == "shelf"){ //We're in shelf mode, just vibe for a bit
    setESC(shelfMotor);
    if(millis() > (shelfTime + shelfDuration)){ //We've been shelved for long enough, continue to the rest of the braking
      revState = "idle";
      timeInRevState = millis();
    }
  }
}

int timingMod(){ //The solenoid seems to take a little bit longer to engage on the first shot of a mag, this accounts for that. the extra few ms should be negligible on one shot.
  if(firedDarts > 0){
    return 0;
  } else {
    return solenoidCold;
  }
}

void pollButtons(){
  triggerButton.update(digitalRead(TRIGGER)); 
  revButton.update(digitalRead(REV));
  fireSetting = selectFire();
  progButton1.update(digitalRead(PROG_1));
  progButton2.update(digitalRead(PROG_2));
}

void loop() {  //Main loop

  pollButtons();

/* test code
  if(counter >= 100){
    //Serial.println("100 loops of main /owedDarts = " + String(owedDarts) + " /time = " + String(millis()));
    counter = 0;
  } else {
    counter++;
  }
*/
  //Serial.print(" /SelectSwitch: " + String(selectFire()) + " / progSwitch: " + String(programmingMode()));
  
  if (owedDarts > 0) {  // We have darts we gotta fire, go into the fire sequence
    //Serial.println("RevState: " + revState + " /PushState: " + pushState + " /Before fire");
    fire();
    //Serial.println("RevState: " + revState + " /PushState: " + pushState + " /After fire");
  } else { // We don't need to fire any darts, go into revDown sequence unless rev trigger is pressed
    firedDarts = 0;  //This seemed like the best place to track that we're not firing any more darts for now
    if(revTriggerState == HIGH){
        revState = "pre-rev";
        digitalWrite(MOSFET, LOW); //while in Pre-Rev have to make sure the solenoid is idle
        pushState = "idle";        //and make sure we record that change
        timeInRevState = millis();
    } else {
        //Serial.println("RevState: " + revState + " /PushState: " + pushState + " /Before revDown");
        revDown();
        //Serial.println("RevState: " + revState + " /PushState: " + pushState + " /After revDown");
        digitalWrite(MOSFET, LOW); //Make really sure the solenoid is off
    }
  }

//Serial.println("RevState: " + revState + " /PushState: " + pushState + " /");

}

//putting the programming methods down here so I don't have to look at them

void progRobust(){ //Only called when the programming mode changes
  //Serial.println("Robust prog hit, mode: " + String(programmingMode()));
  switch(programmingMode()){  //have to update the number we're currently using so the setting change applies without a restart
    case 1:
      speedSetting = speed1;
      dpsSetting = dps1;
      break;
    case 2:
      speedSetting = speed2;
      dpsSetting = dps2;
      break;
    case 3:
      speedSetting = speed3;
      dpsSetting = dps3;
      break;
    default:
      speedSetting = speed1; //if we get a wierd value just save the default FPS.
      dpsSetting = dps1;
      break;
  }
  //revTime = speedSetting / 5;
  shotDelay = shotDelayCalc(); //dps has changed so we need to recalc the shotDelay
  shelfMotor = shelfMotorCalc(); //speedSetting has changed so we need to recalculate the shelfpower.
}
