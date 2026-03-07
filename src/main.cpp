#include <FEH.h> 

#include <Arduino.h> 
#include <math.h>

 

//Hello 

// Declare things like Motors, Servos, etc. here 

// For example: 

// FEHMotor leftMotor(FEHMotor::Motor0, 6.0); 

// FEHServo servo(FEHServo::Servo0); 

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
 

//after testing we will change these values to their correct encodings per inch, but just placeholders for now 

#define R_ENCODE_P_IN 1 

#define L_ENCODE_P_IN 1 

#define F_ENCODE_P_IN 1 

 

//8 movement directions (cardinal + diagonals)

enum Direction{ 

    FORWARD, 

    REVERSE, 

    LEFT,

    RIGHT,

    LEFT_F, 

    LEFT_R, 

    RIGHT_F, 

    RIGHT_R 

}; 

 

//declare motor variables 

FEHMotor rightdrive(FEHMotor::Motor1,9.0); 

FEHMotor leftdrive(FEHMotor::Motor0,9.0); 

FEHMotor frontdrive(FEHMotor::Motor0,9.0); 

FEHMotor compost(FEHMotor::Motor2,5.0);

FEHServo arm(FEHServo::Servo0);


//Declaring Digital Encoders 

DigitalEncoder left_encoder(FEHIO::Pin9); 

DigitalEncoder right_encoder(FEHIO::Pin8); 

DigitalEncoder front_encoder(FEHIO::Pin8); 

 

void Drive(Direction dir, int8_t speed, float distance); //takes input direction (see diagram), speed (in percent), and distance (inches) 

void StopAll(); //stops the motion of all motors 

void Stop(FEHMotor motor); //stops the motion of a specific motor 

 

void Turn_Right(); 

void Turn_Left(); 

//Pivot funtions
void Pivot_Set_Angle(int degree);
//Compost mechanism functions
void Compost_Set_Speed(float percent);

//void Course(); 

 
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

void Drive(Direction dir, int8_t speed, float distance){ 

    // Robot motion commands
    float Vx = 0.0;    // forward (+forward, -back)
    float Vy = 0.0;    // left (+left, -right)
    float omega = 0.0; // rotation (+CCW, -CW)

    switch (dir)
    {
    case FORWARD:
        Vx = (float)speed;
        break;

    case REVERSE:
        Vx = (float)(-speed);
        break;

    case LEFT:
        Vy = (float)speed;
        break;

    case RIGHT:
        Vy = (float)(-speed);
        break;

    case LEFT_F:
        Vx = (float)speed;
        Vy = (float)speed;
        break;

    case LEFT_R:
        Vx = (float)(-speed);
        Vy = (float)speed;
        break;

    case RIGHT_F:
        Vx = (float)speed;
        Vy = (float)(-speed);
        break;

    case RIGHT_R:
        Vx = (float)(-speed);
        Vy = (float)(-speed);
        break;

    default:
        LCD.WriteLine("Direction not specified during drive function");
        return;
    }

    // Kiwi drive wheel speeds
    float wheel1 = Vx + omega;
    float wheel2 = -0.5f * Vx + 0.8660254f * Vy + omega;
    float wheel3 = -0.5f * Vx - 0.8660254f * Vy + omega;

    // Keep wheel output in [-100, 100] while preserving direction ratios.
    float maxMag = fmaxf(fabsf(wheel1), fmaxf(fabsf(wheel2), fabsf(wheel3)));
    if(maxMag > 100.0f){
        float scale = 100.0f / maxMag;
        wheel1 *= scale;
        wheel2 *= scale;
        wheel3 *= scale;
    }

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    front_encoder.ResetCounts();

    rightdrive.SetPercent(wheel1);
    leftdrive.SetPercent(wheel2);
    frontdrive.SetPercent(wheel3);

    float avg_target_counts = distance * ((L_ENCODE_P_IN + R_ENCODE_P_IN + F_ENCODE_P_IN) / 3.0f);
    while(((left_encoder.Counts() + right_encoder.Counts() + front_encoder.Counts()) / 3.0f) < avg_target_counts);

    StopAll();
} 

void Turn_Left(){ 

    rightdrive.SetPercent(-15.); 

    leftdrive.SetPercent(15.); 

    Sleep(2.0); 

    Stop(leftdrive); 
    Stop(rightdrive); 

    return; 

} 

 

void Turn_Right(){ 

    rightdrive.SetPercent(15.); 

    leftdrive.SetPercent(-15.); 

    Sleep(2.0); 

    Stop(leftdrive);
    Stop(rightdrive); 

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

 

/*void Course(){ 

//567 

    //Reseting encoder counts 

    left_encoder.ResetCounts(); 

    right_encoder.ResetCounts(); 

 

    //Sets motors to 25% power 

    leftdrive.SetPercent(25.0); 

    rightdrive.SetPercent(25.0); 

 

    //Wait until encoder reaches 567 counts 

    while(left_encoder.Counts() < 567); 

 

    Stop(); 

 

    Sleep(1.0); 

 

    Turn_Left(); 

 

    Sleep(1.0); 

 

    //Reseting encoder counts 

    left_encoder.ResetCounts(); 

    right_encoder.ResetCounts(); 

 

    //Sets motors to 25% power 

    leftdrive.SetPercent(25.0); 

    rightdrive.SetPercent(25.0); 

 

    //Wait until encoder reaches 405 counts 

    while(left_encoder.Counts() < 405); 

 

    Stop(); 

 

    Sleep(1.0); 

 

    Turn_Right(); 

 

    Sleep(1.0); 

 

    //Reseting encoder counts 

    left_encoder.ResetCounts(); 

    right_encoder.ResetCounts(); 

 

    //Sets motors to 25% power 

    leftdrive.SetPercent(25.0); 

    rightdrive.SetPercent(25.0); 

 

    //Wait until encoder reaches 162 counts 

    while(left_encoder.Counts() < 162); 

 

    Stop(); 

 

    return; 

}*/ 

 

void ERCMain() 

{ 

    int x, y; 

    while(!LCD.Touch(&x, &y)); 

} 

