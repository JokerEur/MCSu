//TODO : добавить метод в computeNewSpeed в метод moveToD

#include <PS4USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#include <AccelStepper.h>
#include <MultiStepper.h>

#define JoyX A0       // Joystick X pin
#define JoyY A1       // Joystick Y pin
#define FocusSpeedPot A2 // Focuse speed potentiometer
#define SpinerSpeedPot A3   // Spiner speed potentiometer
#define SJoyY A4  // Second Joystick that control focus stepper
#define SliderSpeedPot A5 // Slider Speed potentiometer
#define JoySwitch 10  // Joystick switch connected
#define InOutSet 12   // Set Button
#define inLED 8  // Diod that do something
#define outLED 9  // Diod that do something
#define MAX_SPEED 500 // Maximum speed

// Define the stepper motors and the pins the will use
AccelStepper Focus(1, 5, 4); // (Type:driver, STEP, DIR)
AccelStepper Slider(1, 3, 2);
AccelStepper Spiner(1,7,6);

MultiStepper StepperControl;  // Create instance of MultiStepper

long gotoposition[3]; // An array to store the In or Out position for each stepper motor

int JoyXPos = 0; // X Joy position
int JoyYPos = 0; // Y Joy position
int SecondJoyPos = 0; // Seocnd joystick position

 int FocusSpeed; //Focus stepper speed
 int SpinerSpeed; //Spiner stepper speed
 int SliderSpeed; // Slider stepper speed

// IN positions
int XInPoint = 0; 
int YInPoint = 0;
int ZInPoint = 0;

//OUT positions
int XOutPoint = 0;
int YOutPoint = 0;
int ZOutPoint = 0;

int InandOut = 0;


USB Usb;
PS4USB PS4(&Usb);

bool printAngle, printTouch;
uint8_t oldL2Value, oldR2Value;

void setup() {
  Serial.begin(9600);
  Focus.setMaxSpeed(MAX_SPEED);
  Focus.setSpeed(200);
  Spiner.setMaxSpeed(MAX_SPEED);
  Spiner.setSpeed(200);
  Slider.setMaxSpeed(MAX_SPEED);
  Slider.setSpeed(200);

  // Focus.setAcceleration(MAX_SPEED);
  // Slider.setAcceleration(MAX_SPEED);
  // Spiner.setAcceleration(MAX_SPEED);

  pinMode(JoySwitch, INPUT_PULLUP);
  pinMode(InOutSet, INPUT_PULLUP);
  pinMode(inLED, OUTPUT);
  pinMode(outLED, OUTPUT);

  // Create instances for MultiStepper - Adding the 3 steppers to the StepperControl instance for multi control
  StepperControl.addStepper(Focus);
  StepperControl.addStepper(Spiner);
  StepperControl.addStepper(Slider);

}

void loop(){
// Polling Spiner potentiometer
  static uint32_t TMR3;
  if(millis() - TMR3 > 50){
    TMR3 = millis();
    static float val;
    val += (analogRead(SpinerSpeedPot) - val) * 0.08;
    SpinerSpeed = map(val,0,239,0,MAX_SPEED); 
  }
// Polling Focus potentiometer
  static uint32_t TMRF;
  if(millis() - TMRF > 50){
    TMRF = millis();
    static float valF;
    valF += (analogRead(FocusSpeedPot) - valF) * 0.08;
    FocusSpeed = map(valF,0,239,0,MAX_SPEED);
  }
// Polling Slider potentiometer
  static uint32_t TMRS;
  if(millis() - TMRS > 50){
    TMRS = millis();
    static float valSL;
    valSL += (analogRead(SliderSpeedPot) - valSL) * 0.08;
    SliderSpeed = map(valSL,0,239,0,MAX_SPEED);
    
  }

  // If Set button is pressed - toggle between the switch cases
  if (digitalRead(InOutSet) == 0) {
    delay(500);
    // If we hold set button pressed longer then half a second, reset the in and out positions
    if (digitalRead(InOutSet) == 0) {
      InandOut = 4;
    }
    switch (InandOut) { 
      case 0:   
        InandOut = 1;

        // Set IN position
        XInPoint = Focus.currentPosition(); // Set the IN position for Focus
        YInPoint = Spiner.currentPosition(); // Set the IN position for Spiner
        ZInPoint = Slider.currentPosition(); // Set the IN position for Slider

        digitalWrite(inLED, HIGH); // Light up inLed
        
        break;

      case 1: // Set OUT position
        InandOut = 2;
        //  Set the OUT Points for both steppers
        XOutPoint = Focus.currentPosition(); 
        YOutPoint = Spiner.currentPosition();
        ZOutPoint = Slider.currentPosition();

        digitalWrite(outLED, HIGH);

        break;

      case 2: // Move to IN position / go to case 3
        InandOut = 3;

        // Place the IN position into the Array
        gotoposition[0] = XInPoint;
        gotoposition[1] = YInPoint; 
        gotoposition[2] = ZInPoint;

        Focus.setMaxSpeed(FocusSpeed);
        Spiner.setMaxSpeed(SpinerSpeed);
        Slider.setMaxSpeed(SliderSpeed);

        StepperControl.moveTo(gotoposition); // Move back to at the same time
        StepperControl.runSpeedToPosition(); // Blocks until all steppers are in position
        
        break;

      case 3: // Move to OUT position / go back to case 2
        InandOut = 2;

        // Place the OUT position into the Array
        gotoposition[0] = XOutPoint;
        gotoposition[1] = YOutPoint;
        gotoposition[2] = ZOutPoint;

        Focus.setMaxSpeed(FocusSpeed);
        Spiner.setMaxSpeed(SpinerSpeed);
        Slider.setMaxSpeed(SliderSpeed);

        StepperControl.moveToD(gotoposition); // Move motor with their own speed
        StepperControl.runSpeedToPosition(); // Blocks until all are in position
        break;

      case 4: // If Set button is held longer then half a second go back to case 0
        InandOut = 0;
        digitalWrite(inLED, LOW);
        digitalWrite(outLED, LOW);
        delay(1000);
        break;
    }
  }



  JoyYPos = analogRead(JoyY);
  // if Joystick is moved left, move Spiner to left
  if (JoyYPos > 600) {
    static uint32_t TMR1;
  if(millis() - TMR1 > 50){
    TMR1 = millis();
    static float val2;
    val2 += (analogRead(SpinerSpeedPot) - val2) * 0.08;
    SpinerSpeed = map(val2,0,239,0,MAX_SPEED);
    Spiner.setSpeed(SpinerSpeed);
   }
  }
  // if Joystick is moved right, move Spiner to right
  else if (JoyYPos < 400) {
     static uint32_t TMR2;
  if(millis() - TMR2 > 50){
    TMR2 = millis();
    static float val3;
    val3 += (analogRead(SpinerSpeedPot) - val3) * 0.08;
    SpinerSpeed = map(val3,0,239,0,MAX_SPEED);
    Spiner.setSpeed(-SpinerSpeed);
    }
  }
  // if Joystick stays in middle, no movement
  else {
    Spiner.setSpeed(0);
  }


  JoyXPos = analogRead(JoyX);
// if Joystick is moved UP, move Sliter to UP
  if (JoyXPos > 600) {
    static uint32_t TMRY2;
    if(millis() - TMRY2 > 50){
      TMRY2 = millis();
      static float valY2;
      valY2 += (analogRead(SliderSpeedPot) - valY2) * 0.08;
      SliderSpeed = map(valY2,0,239,0,MAX_SPEED);
      Slider.setSpeed(SliderSpeed);
    }
  }
// if Joystick is moved DOWN, move Sliter to DOWN
  else if (JoyXPos < 400) {
    static uint32_t TMRY3;
    if(millis() - TMRY3 > 50){
      TMRY3 = millis();
      static float valY3;
      valY3 += (analogRead(SliderSpeedPot) - valY3) * 0.08;
      SliderSpeed = map(valY3,0,239,0,MAX_SPEED);
      Slider.setSpeed(-SliderSpeed);
    }
  }
  // if Joystick stays in middle, no movement
  else {
    Slider.setSpeed(0);
  }

  
  // Second joystick
  SecondJoyPos = analogRead(SJoyY);
 // if Joystick is moved left, move Focus to left
  if (SecondJoyPos > 600) {
       static uint32_t FTMR2;
      if(millis() - FTMR2 > 50){
        FTMR2 = millis();
        static float ValF2;
        ValF2 += (analogRead(FocusSpeedPot) - ValF2) * 0.08;
        FocusSpeed = map(ValF2,0,239,0,MAX_SPEED);
        Focus.setSpeed(FocusSpeed);
    }
  }
 // if Joystick is moved right, move Focus to right
  else if (SecondJoyPos < 400 ) {
       static uint32_t FTMR3;
      if(millis() - FTMR3 > 50){
        FTMR3 = millis();
        static float ValF3;
        ValF3 += (analogRead(FocusSpeedPot) - ValF3) * 0.08;
        FocusSpeed = map(ValF3,0,239,0,MAX_SPEED);
        Focus.setSpeed(-FocusSpeed);
    }
  }
  // if Joystick stays in middle, no movement
  else {
    Focus.setSpeed(0);
  }

  // Execute the above commands - run the stepper motors
  Focus.runSpeed();
  Spiner.runSpeed();
  Slider.runSpeed();
}