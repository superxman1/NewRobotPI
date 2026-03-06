#include <FEH.h>
#include <Arduino.h>


// Declare things like Motors, Servos, etc. here
// For example:
// FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
// FEHServo servo(FEHServo::Servo0);
FEHMotor front(FEHMotor::Motor0, 9.0);
FEHMotor right(FEHMotor::Motor1, 9.0);
FEHMotor left(FEHMotor::Motor2, 9.0);

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

    //Move from one side to another (Milestone Part 1)
    frontmove(50.0, 3.0);
    stop();


    //Wait for Touch to start Milestone Part 2
    while(!LCD.Touch(&x, &y)){};

    //Move up and down the ramp (Milestone Part 2 and Bonus)
    frontmove(50.0, 2.0);
    stop();
    Sleep(2.0);
    backmove(50.0, 2.0);
    stop();
}