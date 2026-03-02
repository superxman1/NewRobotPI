#include <FEH.h>
#include <Arduino.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>

//Pivot Constants
#define Apple_Pickup_ANGLE 90
#define Apple_Dropoff_ANGLE 0
#define Window_ANGLE 45
#define Lever_Down_ANGLE 90
#define Lever_Up_ANGLE 0
#define Servo_Max_Angle 180
#define Servo_Min_Angle 0

//Compost Mechanism Constants
#define Compost_Speed 25.0

//Hello
// Declare things like Motors, Servos, etc. here
// For example:
// FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
// FEHServo servo(FEHServo::Servo0);

FEHMotor rightdrive(FEHMotor::Motor1,9.0);
FEHMotor leftdrive(FEHMotor::Motor0,9.0);
FEHMotor compost(FEHMotor::Motor2,5.0);
FEHServo arm(FEHServo::Servo0);

//Pivot funtions
void Pivot_Set_Angle(int degree);
//Compost mechanism functions
void Compost_Set_Speed(float percent);
//Crayola Bot Drive Functions
void Drive_Forward();
void Drive_Back();
void Turn_Right();
void Turn_Left();
void Stop();

//Pivot functions
void Pivot_Set_Angle(int degree){
    arm.SetDegree(degree);
    return;
}

//Compost mechanism functions
void Compost_Set_Speed(float percent){
    compost.SetPercent(percent);
    return;
}

//Crayola Bot Drive Functions//

/*
void Drive_Forward(){
    rightdrive.SetPercent(-25.);
    leftdrive.SetPercent(25.);
    return;
}

void Drive_Back(){
    rightdrive.SetPercent(-25.);
    leftdrive.SetPercent(-25.);
    return;
}

void Turn_Right(){
    rightdrive.SetPercent(-15.);
    leftdrive.SetPercent(15.);
    Sleep(3.0);
    Stop();
    return;
}

void Turn_Left(){
    rightdrive.SetPercent(15.);
    leftdrive.SetPercent(-15.);
    Sleep(3.0);
    Stop();
    return;
}

void Stop(){
    rightdrive.SetPercent(0.);
    leftdrive.SetPercent(0.);
    return;
}
*/

    enum LineStates {
        MIDDLE,
        RIGHT,
        LEFT
    };



    void ERCMain()
    {
        int x, y; //for touch screen

        // Declarations for crayola bot bump switches//
        
        /*
        DigitalInputPin fr_switch(FEHIO::Pin6);
        DigitalInputPin fl_switch(FEHIO::Pin7);
        DigitalInputPin br_switch(FEHIO::Pin8);
        DigitalInputPin bl_switch(FEHIO::Pin9);
        */

        // Declarations for analog optosensors
        AnalogInputPin right_opto(FEHIO::Pin0);
        AnalogInputPin middle_opto(FEHIO::Pin1);
        AnalogInputPin left_opto(FEHIO::Pin2);

        //Calibrate the arm servo to get the min and max values for the servo. This only needs to be done once per servo, so you can comment this out after you get the values and set them with SetMin() and SetMax()
        arm.TouchCalibrate();

        //Set the min and max values for the servo so that it can function properly.
        arm.SetMin(Servo_Min_Angle);
        arm.SetMax(Servo_Max_Angle);

        //Initialize the screen
        LCD.Clear(BLACK);
        LCD.SetFontColor(WHITE);

        LCD.WriteLine("Analog Optosensor Testing");
        LCD.WriteLine("Touch the screen");
        while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
        while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed

        /* Drive_Forward();
        while(fr_switch.Value() == 1 && fl_switch.Value() == 1){ */

        // Record values for optosensors on and off of the straight line
        // Left Optosensor on straight line

        while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
        while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
        // Write the value returned by the optosensor to the screen

        int state = MIDDLE;
        /* while(1) {
            Drive_Forward();
        } */

        /* while(1) {
            float rightOptosensorValue = right_opto.Value();
            float middleOptosensorValue = middle_opto.Value();
            float leftOptosensorValue = left_opto.Value();

            LCD.Write("Left Optosensor Value:");
            LCD.WriteLine(leftOptosensorValue);
            LCD.Write("Middle Optosensor Value:");
            LCD.WriteLine(middleOptosensorValue);
            LCD.Write("Right Optosensor Value:");
            LCD.WriteLine(rightOptosensorValue);

            Sleep(0.5);

        } */


        //Working State machine for line following//
       
        /*
        while(1) {
            float rightOptosensorValue = right_opto.Value();
            float middleOptosensorValue = middle_opto.Value();
            float leftOptosensorValue = left_opto.Value();

            switch(state) {
                case MIDDLE:
                    Drive_Forward();

                    if(rightOptosensorValue > 2.4) {
                        state = RIGHT;
                        LCD.WriteLine(state);
                    }
                    else if (leftOptosensorValue > 2.4) {
                        state = LEFT;
                        LCD.WriteLine(state);
                    }
                    break;

                case RIGHT:
                    leftdrive.SetPercent(25.);
                    rightdrive.SetPercent(-10.);

                    if(rightOptosensorValue < 2.5) {
                        state = MIDDLE;
                        LCD.WriteLine(state);
                    }
                    break;

                case LEFT:
                    leftdrive.SetPercent(10.);
                    rightdrive.SetPercent(-25.);

                    if(leftOptosensorValue < 2.5) {
                        state = MIDDLE;
                        LCD.WriteLine(state);
                    }
                    break;
             } 

            Sleep(0.03);

             
        } */
       
     } 
    
        

        // Left Optosensor off straight line
        /* LCD.Clear(BLACK);
        LCD.WriteLine("Place left optosensor off straight line");
        Sleep(0.25); // Wait to avoid double input
        LCD.WriteLine("Touch screen to record value (2/12)");
        while(!LCD.Touch(&x,&y)); //Wait for screen to be pressed
        while(LCD.Touch(&x,&y)); //Wait for screen to be unpressed
        // Write the value returned by the optosensor to the screen
        // <ADD CODE HERE>

        // Repeat process for remaining optosensors, and repeat all three for the curved line values
        // <ADD CODE HERE>

        // Print end message to screen
        LCD.Clear(BLACK);
        LCD.WriteLine("Test Finished"); */
    
