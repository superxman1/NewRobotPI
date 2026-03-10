#include <FEH.h>
#include <Arduino.h>


// Declare things like Motors, Servos, etc. here
// For example:
// FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
// FEHServo servo(FEHServo::Servo0);
FEHMotor front(FEHMotor::Motor0, 9.0);
FEHMotor right(FEHMotor::Motor1, 9.0);
FEHMotor left(FEHMotor::Motor2, 9.0);

DigitalEncoder left_encoder(FEHIO::Pin12);
DigitalEncoder right_encoder(FEHIO::Pin10);
DigitalEncoder front_encoder(FEHIO::Pin8);

void frontmove(float percent, float time);
void linleftmove(float percent);
void linrightmove(float percent, float time);
void frightmove(float percent, float time);
void fleftmove(float percent, float time);
void brightmove(float percent, float time);
void bleftmove(float percent, float time);
void backmove(float percent, float time);
void stop();

//This function allows the robot to move forward with the front side leading
//It takes a percent and time
void frontmove(float percent, float time){
    right.SetPercent(percent);
    left.SetPercent(-percent);

    Sleep(time);

    return;
}

//This function allows the robot to move to the left
//The nose does not lead for this function
void linleftmove(float percent, float time){
    right.SetPercent(-percent);
    left.SetPercent(-percent);
    front.SetPercent(2*percent);

    Sleep(time);

    return;
}

//This function allows the robot to move to the right
//The nose does not lead for this function
void linrightmove(float percent, float time){
    right.SetPercent(percent);
    left.SetPercent(percent);
    front.SetPercent(-2*percent);

    Sleep(time);

    return;
}

//This function allows the robot to move to the front-right
//The nose does not lead for this function
void frightmove(float percent, float time){
    right.SetPercent(percent);
    front.SetPercent(-percent);

    Sleep(time);

    return;
}

//This function allows the robot to move to the front-left
//The nose does not lead for this function
void fleftmove(float percent, float time){
    left.SetPercent(-percent);
    front.SetPercent(percent);

    Sleep(time);

    return;
}

//This function allows the robot to move to the front-left
//The right nose will lead for this function
void brightmove(float percent, float time){
    left.SetPercent(percent);
    front.SetPercent(-percent);

    Sleep(time);

    return;
}

//This function allows the robot to move to the front-left
//The right nose will lead for this function
void bleftmove(float percent, float time){
    right.SetPercent(-percent);
    front.SetPercent(percent);

    Sleep(time);

    return;
}

//This function allows the robot to move backwards
//The back face will lead for this function
void backmove(float percent, float time){
    right.SetPercent(-percent);
    left.SetPercent(percent);

    Sleep(time);

    return;
}
//This function stops all motors
void stop(){
    right.Stop();
    left.Stop();
    front.Stop();
    
    return;
}

void ERCMain()
{
    //Wait for Touch to start Milestone Part 1
    int x, y;
    while(!LCD.Touch(&x, &y)){};

    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    right.SetPercent(25.0);
    left.SetPercent(-25.0);
    front.SetPercent(25.0);

    while(right_encoder.Counts() < 100){
        LCD.Clear();
        LCD.WriteLine(right_encoder.Counts());
        LCD.WriteLine(left_encoder.Counts());
        LCD.WriteLine(front_encoder.Counts());
        Sleep(0.1);
    };
    
    stop();

    LCD.WriteLine(right_encoder.Counts());
    LCD.WriteLine(left_encoder.Counts());

}