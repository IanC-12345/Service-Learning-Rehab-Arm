#include <Arduino.h>
#include <AccelStepper.h>
#include <ezButton.h>
#include "ACS712.h"
// include <LiquidCrystal_I2C>

// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define motorInterfaceType 1
#define dirPin 2  // direction pin
#define stepPin 3 // Step/ CP pin

#define end_ang_plus 5 // adjust end angle +
#define end_ang_min 6  // adjust end angle -

#define sp_plus 7 // adjust speed +
#define sp_min 8  // adjust speed -

#define start_motor 9 // start arm movement pin9
#define stop_motor 10 // stop arm movement

#define LM1 11 // Lower limit switch
#define LM2 12 // Upper limit switch

#define zeroing 13 // Zeroing button

const int BUTTON_NUM = 9;

int over_current_counter = 0;
_Bool reset_bt = false;

ezButton buttonArray[] = {
    ezButton(end_ang_plus), // 0
    ezButton(end_ang_min),  // 1
    ezButton(sp_plus),      // 2
    ezButton(sp_min),       // 3
    ezButton(start_motor),  // 4
    ezButton(stop_motor),   // 5
    ezButton(LM1),          // 6
    ezButton(LM2),          // 7 
    ezButton(zeroing),      // 8
};

struct
{
    _Bool initial_state = false;
    _Bool zeroing_state = false;
    _Bool start_state = false;
    _Bool stop_state = false;
    _Bool cramp_state = false;
    _Bool estop_state = false;

    _Bool LM1_state, LM2_state = false;
    _Bool forward = true;

    int limiter[2][3] = {{30, 5000, 1500},
                         {5, 2000, 500}};
    /*
    {start angle upper limit, end angle upper limit, speed upper limit}
    {start angle lower limit, end angle lower limit, speed lower limit}
    */

    int start_angle;
    int end_angle = limiter[1][1];
    int speed = limiter[1][2];
    int max_speed, min_speed;
    _Bool init_set;

} arm;

void zeroing_stepper();
void debugging();
// void estop_motor();
void adjust();

AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin); // Create a new instance of the AccelStepper class:
// LiquidCrystal_I2C lcd(0x27, 16, 2);

ACS712 ACS(A0, 5.0, 1023, 185);

void setup()
{
    Serial.begin(9600);
    for (byte i = 0; i < BUTTON_NUM; i++)
    {
        buttonArray[i].setDebounceTime(100); // set debounce time to 50 milliseconds
    }
    stepper.setMaxSpeed(3000);     // limiter
    stepper.setAcceleration(1000); // limiter
    while (!Serial)
        ;
    Serial.println(__FILE__);
    Serial.print("ACS712_LIB_VERSION: ");
    Serial.println(ACS712_LIB_VERSION);

    ACS.autoMidPoint();
    // timed_event = 50; // after 1000 ms trigger the event
    Serial.println(F("Power On"));
}

void estop_motor()
{
    if (arm.zeroing_state == true)
    {
    }
}

void adjust()
{
    if (arm.zeroing_state == true)
    {
        // Serial.println("Ready for motion");
        if (buttonArray[0].isPressed())
        { // end angle plus
            arm.end_angle += 500;
            if (arm.end_angle > arm.limiter[0][1])
            {
                arm.end_angle = arm.limiter[0][1];
            }
        }

        if (buttonArray[1].isPressed())
        { // end angle minus
            arm.end_angle -= 500;
            if (arm.end_angle < arm.limiter[1][1])
            {
                arm.end_angle = arm.limiter[1][1];
            }
        }

        if (buttonArray[2].isPressed())
        { // speed plus
            arm.speed += 100;
            if (arm.speed > arm.limiter[0][2])
            {
                arm.speed = arm.limiter[0][2];
            }
        }

        if (buttonArray[3].isPressed())
        { // speed minus
            arm.speed -= 100;
            if (arm.speed < arm.limiter[1][2])
            {
                arm.speed = arm.limiter[1][2];
            }
        }

        while (buttonArray[4].isPressed())
        { //Serial.println(F("Start")); // debug


            if (arm.forward == true)
            {
                stepper.moveTo(arm.end_angle);
                stepper.setSpeed(arm.speed);
                stepper.runSpeedToPosition();
                if (stepper.currentPosition() == arm.end_angle)
                {
                    arm.forward = false;
                }
            }

            if (arm.forward == false)
            {
                stepper.moveTo(200);
                stepper.setSpeed(arm.speed);
                stepper.runSpeedToPosition();
                if (stepper.currentPosition() == 200)
                {
                    arm.forward = true;
                }
            }

            float mA = ACS.mA_DC() / 1000;
            if (mA > 0.8)
            {
                over_current_counter += 1;
                if (over_current_counter > 4)
                {
                    Serial.println(mA);
                    Serial.println(F("Cramp Detected!")); // debug
                    stepper.stop();
                    arm.zeroing_state = false;
                    arm.initial_state = false;
                    arm.LM1_state = false;
                    arm.LM2_state = false;
                    over_current_counter = 0;
                    break;
                }
            }
            else
            {

                over_current_counter = 0;
            }

            buttonArray[5].loop();
            buttonArray[6].loop();
            buttonArray[7].loop();
            if (buttonArray[5].isPressed())
            {
                Serial.println(F("Moving Back to Origin")); // debug
                stepper.stop();
                stepper.runToNewPosition(200);
                Serial.println(F("Moved to Origin"));
                arm.forward = true;
                break;
            }

            if (reset_bt == false)
            {
                Serial.println(F("reseted limit switch"));
                buttonArray[6].reset();
                buttonArray[7].reset();
                reset_bt = true;
            }

            if (buttonArray[6].isPressed() || buttonArray[7].isPressed())
            {
                Serial.println(F("Estop Activiaed")); // debug
                stepper.stop();
                arm.zeroing_state = false;
                arm.initial_state = false;
                arm.LM1_state = false;
                arm.LM2_state = false;
                over_current_counter = 0;
                break;
            }
        }
    }
}

void zeroing_stepper()
{
    while (buttonArray[8].isPressed())
    {
        buttonArray[6].loop();
        buttonArray[7].loop();
        if (arm.initial_state == false)
        { // User define button to press and do zeroing
            // Serial.println(F("Zeroing")); // debug
            stepper.setSpeed(-1000);
            stepper.runSpeed();
        }

        if (buttonArray[6].isPressed() && arm.LM1_state == false)
        { // Limit switch 1

            Serial.println(F("Limit Switch 1 Touched"));
            arm.start_angle = 0;
            stepper.stop();
            stepper.setCurrentPosition(arm.start_angle);
            // delay(1000);
            stepper.setSpeed(1000);
            stepper.runSpeed();
            // Serial.println(F("motor running backward")); // debug
            arm.LM1_state = true;
            // Serial.println((arm.LM1_state)); //debug
            arm.initial_state = true;
        }

        if (buttonArray[7].isPressed() && arm.LM1_state == true && arm.LM2_state == false)
        {
            Serial.println(F("Limit Switch 2 Touched"));
            // Serial.println(F("motor running forward"));
            //   arm.end_angle = 120;
            stepper.stop();
            arm.LM2_state = true;
            // delay(1000);
        }

        if (arm.LM1_state == true && arm.LM2_state == true && arm.zeroing_state == false)
        {
            // stepper.setSpeed(900);
            stepper.runToNewPosition(200);
            // stepper.runToPosition();
            Serial.println(F("Finish Zeroing"));
            arm.zeroing_state = true;
            reset_bt = false;
            // Serial.println("Ready for motion");
            buttonArray[8].reset();
            break;
        }
        stepper.runSpeed();
    }
}

void debugging()
{
    // static int last_time = millis();
    // int current_time = millis();
    // if(current_time - last_time >= 750){
    // Serial.println(stepper.currentPosition());
    // Serial.println(arm.end_angle);
    // Serial.println(arm.speed);
    //   Serial.println(arm.zeroing_state);
    //   Serial.println(arm.start_state);
    //   buttonArray[5].loop();
    //   if(buttonArray[5].isPressed()){
    //   stepper.stop();
    //   Serial.println(F("Stopped")); //debug
    //   }
    // last_time = current_time;
    //
    //}
}

void loop()
{

    for (byte i = 0; i < 10; i++)
    {
        buttonArray[i].loop();
    }
    zeroing_stepper();
    adjust();
    estop_motor();
    // debugging();
}
