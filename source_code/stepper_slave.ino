/*
   stepper_slave.ino

   For use with Arduino Mega. Main code (for the SLAVE controller).
   It moves a stepper motor to a desired user angle, giving feedback.

   Authors: Maxi Mira Gallbrecht, TU Delft
            Tiago Sousa Costa, TU Delft

*/


/*
   LIBRARIES
*/
#include <MotorStepper.h>    // Stepper motor
#include <LiquidCrystal.h>   // LCD
#include <TimerOne.h>        // Arduino timer 
#include <Wire.h>            // Serial communication between arduinos


/*
   CONSTANTS
*/
#define STEPS 4096             // Steps to total revolution
#define STEPS_TOLERANCE  0     // Tolerance to potentiometer input
#define DEGREES_TOLERANCE 15   // Angle drift tolerance between potentiometer and stepper motors (degrees)
#define POT_AGREE 15           // Tolerance between both potentiometers output (degrees)
#define RPM 10                 // Stepper speed [RPM]
#define MAX_ANGLE 270          // Maximum allowed angle (from the rotation of the potentiometers)
#define MIN_POT 100            // minimum potientometer margin value
#define MAX_POT 900            // maximum potientometer margin value
#define POT 1023               // physical maximum potientometer value
#define FEEDBACK1 8            // potentiometer #1 feedback
#define FEEDBACK2 9            // potentiometer #2 feedback
#define TIME 1                 // timer reset constant (in seconds)     
#define WAITING 5              // waiting time to take action

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;   // LCD screen digital pins
const int red1 = 30, green1 = 31, red2 = 32, green2 = 33;     // LEDs digital pins
const int rst = 21;                                           // reset digital pin
const int safe = 20;                                          // safe mode digital pin
const int safe_angle = 100;                                   // safe angle value


/*
   Steppers and LCD initialization
*/
MotorStepper stepper1(7, 8, 9, 10);
MotorStepper stepper2(22, 23, 24, 25);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/*
   GLOBAL VARIABLES
*/
boolean moveClockwise = true;          // moving direction
boolean moveCounterClockwise = false;  // moving direction

// FLAGS
int show = 0;              // To only show in serial monitor when 1 sec (or other predefined time) has passed
int check = 0;             // Force (or not) the verification of the current angle
int reset_on = 0;          // flag: 1 = reset going on, 0 = reset finished
int safe_on = 0;           // flag: 1 = safe mode deployed, 0 = normal process
int try_reset = 0;         // flag: 1 = already tried reseting the master board; 0 = did not try yet
int try_reset_motor = 0;   // flag: 1 = already tried reseting the motor angle; 0 = did not try yet
int force_reset = 0;       // variable that is 1 when a reset is intended (called from another function instead of the button)
int slave = 0;             // slave is 1 when it needs to send commands
int master_fail = 0;       // 1 if master failed

// OTHER VARIABLES
int reply = 10;            // reply from the master (assume it's fine at first)
int minitime = 0;          // counter for timer
int current1 = 0;          // Stepper1 current value
int current2 = 0;          // Stepper2 current value
float c_degree = 0;        // (active) Stepper current angle
float val_degree = 0.0;    // Steppers desired value in degrees
int val = 0;               // Steppers desired value
float pot_degree = 0.0;    // potentiometer 1 angle
float pot_degree_2 = 0.0;  // potentiometer 2 angle
int opt = 3;               // option for how to initialize the stepper motors
int active = 1;            // Defines which stepper is active
int read_chars = 0;        // User serial monitor input
char numStr[6];            // the “number” input (as characters and null)


/*
   INITIAL COMPUTATIONS
*/
const int step_270 = int(map(MAX_ANGLE * 1000.0, 0, 359.92 * 1000, 0, STEPS - 1));                 // computing the step that corresponds to 270 degrees
const int max_step = int(map(MAX_POT, 0, POT, 0, step_270));                                       // calculating maximum possible step WITH MARGIN
const float max_angle_margin = float(map(max_step, 0, step_270, 0, MAX_ANGLE * 1000.0) / 1000.0);  // converting the last to an angle in degrees
const int min_step = int(map(MIN_POT, 0, POT, 0, step_270));                                       // calculating minimum possible step WITH MARGIN
const float min_angle_margin = float(map(min_step, 0, step_270, 0, MAX_ANGLE * 1000.0) / 1000.0);  // converting the last to an angle in degrees

const long int t_i = 1000000 * TIME;   // timer reset period (converted constant to microseconds)


/*
   INITIALIZATION
*/
void setup() {
  Serial.begin(9600);   // Serial monitor setup
  Serial2.begin(9600);  // Slave board

  // Give user initial messages in the monitor
  Serial.println("SLAVE CONTROLLER");
  Serial.println(); Serial.print("Triggering action at "); Serial.print(WAITING); Serial.println(" seconds.");
  Serial.flush();

  // Setup motor speeds
  stepper1.setRpm(RPM);  // Setup motor speed
  stepper2.setRpm(RPM);  // Setup motor speed

  // Initialize LED pins
  pinMode(red1, OUTPUT);
  pinMode(green1, OUTPUT);
  pinMode(red2, OUTPUT);
  pinMode(green2, OUTPUT);

  // Verification message
  // Serial.print("Step_270= "); Serial.println(step_270);
  // Serial.print("max_step= "); Serial.println(max_step);
  // Serial.print("max_angle= "); Serial.println(max_angle_margin);
  // Serial.print("min_step= "); Serial.println(min_step);
  // Serial.print("min_angle= "); Serial.println(min_angle_margin);

  // Initialize timer
  Timer1.initialize(t_i);     // t_i seconds for timer to reset
  Timer1.attachInterrupt(timer);

  // Configure reset pin as an interrupt
  pinMode(rst, INPUT);
  attachInterrupt(digitalPinToInterrupt(rst), reset, HIGH);  // start interrupt function when 'rst' becomes HIGH

 // Configure safe mode pin as an interrupt
  pinMode(safe, INPUT);
  attachInterrupt(digitalPinToInterrupt(safe), safe_mode, HIGH);  // start interrupt function when 'safe' becomes HIGH

  // Initialize LCD
  lcd.begin(16, 2);
}


/*
   init_value

   INPUT: int option (defines how the initial angle will be calculated)
            option = 1: potentiometer 1 value is used
            option = 2: potentiometer 2 value is used
            option = 3: both potentiometers values are used (average)
   OUTPUT: none
   FUNCTION: Initializes the stepper motors angles

*/
void init_value(int option) {
  float init = 0;

  if (option == 1)
    init = 0.2717 * analogRead(FEEDBACK1) + 2.1263;   // compute angle according to estimated function

  if (option == 2)
    init = 0.2784 * analogRead(FEEDBACK2) - 41.3101;

  if (option == 3) {
    pot_degree = 0.2717 * analogRead(FEEDBACK1) + 2.1263;
    pot_degree_2 = 0.2784 * analogRead(FEEDBACK2) - 41.3101;

    init = (pot_degree + pot_degree_2) / 2;  // average angle
  }

  int init_steps = map(init, 0, MAX_ANGLE, 0, step_270); // convert to steps

  val = init_steps;                //  force initial user input as the initial feedback value

  // Configure stepper motor initial angles
  stepper1.setStep(init_steps);
  stepper2.setStep(init_steps);

  current1 = stepper1.getStep();
  val_degree = map(current1, 0, STEPS - 1, 0, 359.92 * 1000) / 1000.0;  // To obtain a float angle, multiply and divide by 1000.0
}


/*
   timer

   INPUT: none
   OUTPUT: none
   FUNCTION: used as an interrupt. Every 'x' seconds, this function is invoked.
             it also controlls the watchdog, to check if the MASTER is working properly.

*/
void timer() {
  show = 0;

  if (reply == 10) { // if the correct message is received from the MASTER board
    try_reset = 0;  // make sure new resets can be performed as the system is working normally
    Serial.println("Master working properly.");
    reply = 0;      // reset response and counter
    minitime = 0;
  }
  else if (master_fail == 0) {
    minitime++;
    if (try_reset == 0)          // if it is the first time attempting to reset, print the counter
      Serial.println(minitime);

    if (minitime >= WAITING) {   // if it has gone for too long, try to reset the MASTER
      if (try_reset == 0) {
        try_reset = 1;
        Serial.println("Fault detected in master. Attempting reset...");
        Serial2.write(2);
        minitime = WAITING - 4;  // wait for 3 more seconds to check the results
      }
      else {                     // Use the SLAVE as the controller
        Serial2.write(3);
        minitime = WAITING - 4;
        master_fail = 1;
      }
    }
  }
}


/*
   reading

   INPUT: none
   OUTPUT: none
   FUNCTION: reads the user input from the serial monitor.

*/
void reading() {
  if (Serial.available() > 0) {
    numStr[read_chars] = Serial.read();

    if ( numStr[read_chars] == '\n' || numStr[read_chars] == '\r' ) {
      numStr[read_chars] = '\0';
      val_degree = atof(numStr);        // convert to float
      check_input();
      val = round(val_degree * (STEPS) / 360); // Round up if above x.5
      read_chars = 0;
    }
    else
      read_chars++;
  }
}


/*
   reading_analog

   INPUT: none
   OUTPUT: none
   FUNCTION: reads the steppers' and potentiometers' angles

*/
void reading_analog() {
  current1 = stepper1.getStep();
  current2 = stepper2.getStep();

  // convert potentiometer feedback to an angle, according to predefined functions
  pot_degree = 0.2717 * analogRead(FEEDBACK1) + 2.1263;
  pot_degree_2 = 0.2784 * analogRead(FEEDBACK2) - 41.3101;
}


/*
   check_input()

   INPUT: none
   OUTPUT: none
   FUNCTION: Checks the user input according to the protective boundaries.
             Adjusts the input if it is outside of these boundaries.
             Prints messages to the user in the monitor

*/
void check_input() {
  // if the input is valid
  if (val_degree >= min_angle_margin && val_degree <= max_angle_margin) {
    Serial.println(); Serial.print("Input: ");  Serial.print(val_degree); Serial.println("º"); Serial.println("Rotating...");
    Serial.print("\nInput (new) desired angle ("); Serial.print(min_angle_margin); Serial.print("º to "); Serial.print(max_angle_margin); Serial.println("º) and press ENTER: ");
  }

  // if the input is above the maximum
  if (val_degree > max_angle_margin) {
    Serial.println(); Serial.print("Input: ");  Serial.print(val_degree); Serial.print("º"); Serial.println();
    val_degree = max_angle_margin;  // adjust the input to the maximum

    Serial.print("Input must not be higher than "); Serial.print(max_angle_margin); Serial.print("º\n");
    Serial.print("Input defined as "); Serial.print(max_angle_margin); Serial.print("º\n"); Serial.println("Rotating...");
    Serial.print("\nInput (new) desired angle ("); Serial.print(min_angle_margin); Serial.print("º to "); Serial.print(max_angle_margin); Serial.println("º) and press ENTER: ");
  }

  // if the input is below the minimum
  if (val_degree < min_angle_margin) {
    Serial.println(); Serial.print("Input: ");  Serial.print(val_degree); Serial.print("º"); Serial.println();
    val_degree = min_angle_margin; // adjust the input to the minimum

    Serial.print("Input must not be lower than "); Serial.print(min_angle_margin); Serial.print("º\n");
    Serial.print("Input defined as "); Serial.print(min_angle_margin); Serial.print("º\n"); Serial.println("Rotating...");
    Serial.print("\nInput (new) desired angle ("); Serial.print(min_angle_margin); Serial.print("º to "); Serial.print(max_angle_margin); Serial.println("º) and press ENTER: ");
  }
}


/*
   reset()

   INPUT: none
   OUTPUT: none
   FUNCTION: Used as an interrupt. If a (predefined) pin is HIGH, it tries to reset the stepper angle.
             If no reference value can be used, it switches to the redundant  motor.

*/
void reset() {
  // if no reset callout is currently undergoing and the reset pin is high OR
  //    the arduino detected a drift (force_reset is 1)
  if ((reset_on == 0 && digitalRead(rst) == 1) || force_reset == 1) {
    int flag = 0;   // flag to prevent part of this interrupt from executing if the angle of the main stepper cannot be reseted

    reset_on = 1;
    Serial.println(); Serial.println("Reset in progress...");

    if (abs(pot_degree - pot_degree_2) < POT_AGREE) {   // if both potentiometers agree on the angle of the stepper
      Serial.println("The potentiometers are reading approximately the same angle. Potentiometers average value will be used");
      opt = 3;
    }
    else {
      if (abs(c_degree - pot_degree) < DEGREES_TOLERANCE) {  // if potentiometer 1 has a relatively close angle to the current stepper angle
        Serial.println("The potentiometers are reading different values.");
        Serial.println("Potentiometer #1 will be used for showing the closest value to the stepper motor");
        opt = 1;
      }
      else if (abs(c_degree - pot_degree_2) < DEGREES_TOLERANCE) {  // if potentiometer 2 has a relatively close angle to the current stepper angle
        Serial.println("The potentiometers are reading different values.");
        Serial.println("Potentiometer #2 will be used for showing the closest value to the stepper motor");
        opt = 2;
      }
      else { // no feedback is matching
        Serial.println("The potentiometers are reading different values.");
        Serial.println("No value can be used. Switching to second stepper motor."); Serial.flush();
        active = 2;
        flag = 1;     // prevent the rest of the function from executing
      }
    }

    if (flag == 0) {
      active = 1;        // force stepper 1 as the active one
      init_value(opt);   // force the stepper to a new angle based on the feedback

      Serial.print("Angle reseted. The new angle is "); Serial.print(val_degree); Serial.println("º.");
      try_reset_motor = 0;   // reset is successful
    }

    Serial.println();
    if (force_reset == 0)   // show this message if the user pressed the reset trigger only
      Serial.println("Please remove the reset trigger.");
    Serial.flush();

    int count = 0;           // counter
    int r = 0;               // reset pin value
    while (count < 10000) {  // 10000 times to prevent the arduino from calling several times in a row the interrupt function
      r = digitalRead(rst);
      if (r == 0 || force_reset == 1)
        count++;
      if (r == 1 && force_reset == 0)
        count = 0;
    }
    reset_on = 0;
  }
}


/*
   safe_mode()

   INPUT: none
   OUTPUT: none
   FUNCTION: Used as an interrupt. If a (predefined) pin is HIGH, it triggers the safe mode.
             The active stepper is forced to the minimum angle and the system is put in standby

*/
void safe_mode() {
  if (safe_on == 0 && digitalRead(safe) == 1) {
    int flag = 0;

    safe_on = 1;
    Serial.println(); 
    Serial.println("##################################################");
    Serial.println("SAFE MODE DEPLOYED. SYSTEM WILL BE PUT IN STANDBY.");

    if (abs(pot_degree - pot_degree_2) < POT_AGREE) {   // if both potentiometers agree on the angle of the stepper
      opt = 3;
    }
    else {
      if (abs(c_degree - pot_degree) < DEGREES_TOLERANCE) {  // if potentiometer 1 has a relatively close angle to the current stepper angle
        opt = 1;
      }
      else if (abs(c_degree - pot_degree_2) < DEGREES_TOLERANCE) {  // if potentiometer 2 has a relatively close angle to the current stepper angle
        opt = 2;
      }
      else { // no feedback is matching
        flag = 1;
        active = 2;
        opt = 3;      // there are no potentiometers on the second stepper, assume average to exemplify
      }
    }

    if (flag == 0) {
      active = 1;        // force stepper 1 as the active one
    }

    init_value(opt);   // force the stepper to a new angle based on the feedback
    val_degree = safe_angle;  // move the stepper to the safe angle
    val = round(val_degree * (STEPS) / 360); // Round up if above x.5

    Serial.println(); Serial.print("Moving stepper "); Serial.print(active);
    Serial.print(" to the safe angle ("); Serial.print(safe_angle); Serial.println("º)...");
    Serial.println(); Serial.flush();

    int c = 0;              // counter
    int s = 0;              // safe pin value
    int print_once = 1;     // only print message of the
    int print_exit = 1;     // same as above for exiting safe mode
    while (c < 10000 || check == 1) {     // 10000 times to prevent the arduino from calling several times in a row the interrupt function
      reading_analog();     // check angle
      movement();           // allow stepper to rotate

      if (check == 0 && print_once == 1){
        Serial.print("Stepper "); Serial.print(active); 
        Serial.print(" is in the safe angle ("); Serial.print(safe_angle); Serial.println("º).");
        Serial.println(); Serial.println("To leave the safe mode, please remove the safe trigger.");
        Serial.flush();
        print_once = 0;
      }
      
      s = digitalRead(safe);
      if (s == 0){
        c++;
        if (print_exit == 1){
          Serial.println("Exiting safe mode..."); 
          Serial.flush();
          print_exit = 0;
        }
      }
      else{
        c = 0;
        print_exit = 1;  // allow the message of exiting to come up again
      }
    }
    Serial.println("##################################################");
    Serial.println();
    safe_on = 0;
  }
}


/*
   movement()

   INPUT: none
   OUTPUT: none
   FUNCTION: moves the active stepper to the user input angle

*/
void movement() {  // rotates continuosly until the desired value is obtained
  switch (active) {
    case 1:  // stepper 1 active
      // adjust LEDs (stepper ready)
      digitalWrite(red2, LOW);
      digitalWrite(green2, HIGH);

      // if there is a difference between the user input and the current angle
      if (abs(current1 - val) > STEPS_TOLERANCE) {
        check = 1;                // don't allow to check while it's rotating to prevent errors

        // adjust LEDs (stepper moving)
        digitalWrite(red1, HIGH);
        digitalWrite(green1, HIGH);
        if (current1 < val)       // move in the only possible direction (prevent rotating over 360 degrees)
          stepper1.step(moveClockwise);
        else
          stepper1.step(moveCounterClockwise);
      }
      else {
        // adjust LEDs (stepper ready)
        digitalWrite(red1, LOW);
        digitalWrite(green1, HIGH);
        check = 0;
      }
      break;

    case 2:  // stepper 2 active
      // adjust LEDs (stepper ready)
      digitalWrite(red1, HIGH);
      digitalWrite(green1, LOW);

      // if there is a difference between the user input and the current angle
      if (abs(current2 - val) > STEPS_TOLERANCE) {
        check = 1;                  // don't allow to check while it's rotating

        // adjust LEDs (stepper moving)
        digitalWrite(red2, HIGH);
        digitalWrite(green2, HIGH);
        if (current2 < val)       // move in the only possible direction (prevent rotating over 360 degrees)
          stepper2.step(moveClockwise);
        else
          stepper2.step(moveCounterClockwise);
      }
      else {
        // adjust LEDs (stepper ready)
        digitalWrite(red2, LOW);
        digitalWrite(green2, HIGH);
        check = 0;
      }
      break;
  }
}


/*
   verify_active()

   INPUT: none
   OUTPUT: none
   FUNCTION: verifies if the stepper drifted from the potentiometers feedback
             Tries to reset the angle first.
             If that is not successful, switches to redundant unit.

*/
void verify_active() {
  switch (active)
  {
    case 1:
      if ((abs(val_degree - pot_degree) > DEGREES_TOLERANCE) && (abs(val_degree - pot_degree_2) > DEGREES_TOLERANCE))
        if (try_reset_motor == 0) {
          try_reset_motor = 1;
          force_reset = 1;
          Serial.println(); Serial.println("The motor angle has drifted. A reset will be attempted...");
          reset();
          force_reset = 0;
        }
        else {
          Serial.println(); Serial.println("The motor angle has drifted and a reset attempt has failed.");
          Serial.println("Switching to stepper motor 2..."); Serial.flush();
          active = 2;
        }
      break;
    case 2:   // due to the way the system is mechanically designed, it is not possible to have feedback from the 2nd motor
      break;
  }
}


/*
   print_all()

   INPUT: none
   OUTPUT: none
   FUNCTION: prints in the monitor the state of the system (active stepper; current angle; potentiometers feedback)
             Prints in the LCD as well.

*/
void printall() {
  switch (active) {
    case 1:
      c_degree = map(current1, 0, STEPS - 1, 0, 359.92 * 1000) / 1000.0; // To obtain a float angle
      break;

    case 2:
      c_degree = map(current2, 0, STEPS - 1, 0, 359.92 * 1000) / 1000.0; // To obtain a float angle
      break;
  }

  lcd.setCursor(0, 0);
  lcd.print("Angle:"); lcd.print(c_degree); lcd.print((char)178); lcd.print("  \0");
  lcd.setCursor(0, 1);
  lcd.print("Active:"); lcd.print(active);


  Serial.print("Angle="); Serial.print(c_degree);
  Serial.print("     Potentiometers="); Serial.print(pot_degree); Serial.print(" | "); Serial.print(pot_degree_2);
  Serial.print("     Active="); Serial.println(active);
}


/*
   watchdog

   INPUT: none
   OUTPUT: none
   FUNCTION: reads the master messages and if the MASTER failed, initiate the commands
             for the SLAVE to take over

*/
void watchdog() {
  if (Serial2.available()) {
    reply = Serial2.read();  // read the master
    Serial2.flush();
  }

  if (master_fail == 1) {
    Serial.println();
    Serial.println("###########################################");
    Serial.println("MASTER CONTROLLER FAILED. SLAVE IN CONTROL.");
    Serial.println("###########################################");
    Serial.println();

    Serial.print("Input desired angle ("); Serial.print(min_angle_margin); Serial.print("º to "); Serial.print(max_angle_margin); Serial.println("º) and press ENTER: ");
    init_value(3);   // initiate the steppers as if it was the beggining of the program
    slave = 1;       // activate slave as the controller
  }
}


/*
   deactivate_master

   INPUT: none
   OUTPUT: none
   FUNCTION: sends a message to the MASTER to ensure that it is not controlling the system along with the SLAVE board

*/
void deactivate_master() {
  Serial2.write(3);  // send a message to make sure the master's off
  Serial2.flush();
}


/*
   loop()

   INPUT: none
   OUTPUT: none
   FUNCTION: Main loop

*/
void loop() {

  watchdog();    // checks the MASTER
  delay(100);    // waits 100ms to ensure that the messages are be sent and read between the boards

  while (slave == 1) {    // if the slave is in command
    deactivate_master();  // ensure the master is not working once the slave takes over
    reading();            // reads the user input
    reading_analog();     // reads the stepper and potentiometers feedback
    movement();           // moves the stepper motor

    if (show == 0 && check == 0 && reset_on == 0) {
      reading();          // reads the user input
      reading_analog();   // reads the stepper and potentiometers feedback
      verify_active();    // verifies if everything is working properly
      printall();         // prints the current state of the system

      show = 1;           // to wait for the next clock cycle
    }
  }
}
