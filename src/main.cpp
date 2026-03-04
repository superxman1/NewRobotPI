#include <FEH.h>
#include <Arduino.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>

//Hello
// Declare things like Motors, Servos, etc. here
// For example:
// FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
// FEHServo servo(FEHServo::Servo0);

FEHMotor frontdrive(FEHMotor::Motor2,9.0); 
FEHMotor rightdrive(FEHMotor::Motor1,9.0);
FEHMotor leftdrive(FEHMotor::Motor0,9.0);

DigitalEncoder left_encoder(FEHIO::Pin9); 
DigitalEncoder right_encoder(FEHIO::Pin8); 
DigitalEncoder front_encoder(FEHIO::Pin8); 

AnalogInputPin CdS_cell(FEHIO::Pin3);

void Drive(Direction dir, int8_t speed, float distance); //takes input direction (see diagram), speed (in percent), and distance (inches) 
void StopAll(); //stops the motion of all motors 
void Stop(FEHMotor motor); //stops the motion of a specific motor 
void Turn_Right(); 
void Turn_Left(); 
void startButton();

/* void Drive_Forward();
void Drive_Back();
void Turn_Right();
void Turn_Left();
void Stop(); */

//after testing we will change these values to their correct encodings per inch, but just placeholders for now 
#define R_ENCODE_P_IN 1 
#define L_ENCODE_P_IN 1 
#define F_ENCODE_P_IN 1 

enum Direction{ 
    FORWARD, 
    REVERSE, 
    LEFT_F, 
    LEFT_R, 
    RIGHT_F, 
    RIGHT_R 
}; 


#define START_LIGHT 1
void startButton() {
    float startTime = TimeNow();
    float startCondition = 0;

    while(startTime < 10) {
        float lightReading = CdS_cell.Value();

        if(lightReading < START_LIGHT) {
            Drive(REVERSE, 0.25, 5);
            startCondition = 1;
        }
        startTime = TimeNow();
    }

    if(startCondition == 0) {
        Drive(REVERSE, 0.25, 5);
        startCondition = 1;
    }
}

void Drive(Direction dir, int8_t speed, float distance){ 

//will eventually need a correction factor for momentum 

    switch (dir) {  
        case FORWARD: 
            //reset count
            left_encoder.ResetCounts(); 
            right_encoder.ResetCounts(); 

            //turn motor on to specified speed  
            rightdrive.SetPercent(speed); 
            leftdrive.SetPercent(speed); 
            //wait until distance has been driven 

            while(left_encoder.Counts() < (distance*L_ENCODE_P_IN) || right_encoder.Counts() < (distance*R_ENCODE_P_IN)); 
                //stop all motors (can eventually be changed for correct motors but im lazy rn) 
                StopAll(); 
            break; 

        case REVERSE: 
            left_encoder.ResetCounts(); 
            right_encoder.ResetCounts(); 
            rightdrive.SetPercent((-1)*speed); 
            leftdrive.SetPercent((-1)*speed); 

            while(left_encoder.Counts() < (distance*L_ENCODE_P_IN) || right_encoder.Counts() < (distance*R_ENCODE_P_IN)); 
                StopAll(); 
            break; 

        case LEFT_F: 
            front_encoder.ResetCounts(); 
            left_encoder.ResetCounts(); 
            frontdrive.SetPercent(speed); 
            leftdrive.SetPercent(speed); 

            while(left_encoder.Counts() < (distance*L_ENCODE_P_IN) || front_encoder.Counts() < (distance*F_ENCODE_P_IN)); 
                StopAll(); 
            break; 

        case LEFT_R: 
            front_encoder.ResetCounts(); 
            left_encoder.ResetCounts(); 
            frontdrive.SetPercent((-1)*speed); 
            leftdrive.SetPercent((-1)*speed); 

            while(left_encoder.Counts() < (distance*L_ENCODE_P_IN) || front_encoder.Counts() < (distance*F_ENCODE_P_IN)); 
                StopAll(); 
            break; 

        case RIGHT_F: 
            front_encoder.ResetCounts(); 
            right_encoder.ResetCounts(); 
            rightdrive.SetPercent(speed); 
            frontdrive.SetPercent(speed); 

            while(front_encoder.Counts() < (distance*F_ENCODE_P_IN) || right_encoder.Counts() < (distance*R_ENCODE_P_IN)); 
                StopAll(); 
            break; 

        case RIGHT_R: 
            front_encoder.ResetCounts(); 
            right_encoder.ResetCounts(); 
            rightdrive.SetPercent((-1)*speed); 
            frontdrive.SetPercent((-1)*speed); 

            while(front_encoder.Counts() < (distance*F_ENCODE_P_IN) || right_encoder.Counts() < (distance*R_ENCODE_P_IN)); 
                StopAll(); 
            break; 

        default: 
            LCD.WriteLine("Direction not specidified during drive function"); 
        break; 
    } 
} 

void Turn_Left(){ 
    rightdrive.SetPercent(-15.); 
    leftdrive.SetPercent(15.); 
    Sleep(2.0); 
    Stop(); 
    return; 
} 

 

void Turn_Right(){ 
    rightdrive.SetPercent(15.); 
    leftdrive.SetPercent(-15.); 
    Sleep(2.0); 
    Stop(); 
    return; 
} 

 

void Stop(FEHMotor motor){ 
    motor.SetPercent(0.); 
    return; 
} 

 

void StopAll(){ 
    rightdrive.SetPercent(0.); 
    leftdrive.SetPercent(0.); 
    frontdrive.SetPercent(0.); 
    return; 
} 


    enum LineStates {
        MIDDLE,
        RIGHT,
        LEFT
    };


    void ERCMain()
    {
        int x, y; //for touch screen

        DigitalInputPin fr_switch(FEHIO::Pin6);
        DigitalInputPin fl_switch(FEHIO::Pin7);
        DigitalInputPin br_switch(FEHIO::Pin8);
        DigitalInputPin bl_switch(FEHIO::Pin9);

        // Declarations for analog optosensors
        AnalogInputPin right_opto(FEHIO::Pin0);
        AnalogInputPin middle_opto(FEHIO::Pin1);
        AnalogInputPin left_opto(FEHIO::Pin2);

    

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

                /* default:
                    leftdrive.Stop();
                    rightdrive.Stop();
                    break; */
             } 

            Sleep(0.03);


        } 
     } 