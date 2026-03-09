#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
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

//Define Motors, Servos, and Sensors here
FEHMotor frontdrive(FEHMotor::Motor0,9.0); 
FEHMotor rightdrive(FEHMotor::Motor1,9.0);
FEHMotor leftdrive(FEHMotor::Motor2,9.0);

DigitalEncoder left_encoder(FEHIO::Pin8); 
DigitalEncoder right_encoder(FEHIO::Pin9); 
DigitalEncoder front_encoder(FEHIO::Pin10);

FEHMotor compost(FEHMotor::Motor3,5.0);
FEHServo arm(FEHServo::Servo0);

AnalogInputPin CdS_cell(FEHIO::Pin3);

void StopAll(); //stops the motion of all motors 
void Turn_Right(); 
void Turn_Left(); 
void startButton();
void simpleDrive(int speed, float time);
void simpleReverse(int speed, float time);
void startButton();
void humidifier();

/* void Drive_Forward();
void Drive_Back();
void Turn_Right();
void Turn_Left();
void Stop(); */

//after testing we will change these values to their correct encodings per inch, but just placeholders for now
#define R_ENCODE_P_IN 1
#define L_ENCODE_P_IN 1
#define F_ENCODE_P_IN 1

// Encoder counts needed per degree of robot rotation.
// Tune this value on your robot so angle turns are accurate.
#define TURN_COUNTS_PER_DEG 1.0

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






void Drive(Direction dir, double speed, double distance); //takes input direction (see diagram), speed (in percent), and distance (inches)

void StopAll(); //stops the motion of all motors

void Stop(FEHMotor &motor); //stops the motion of a specific motor

void Turn_Right(double angle_deg, double speed);
void Turn_Left(double angle_deg, double speed);
void Turn_Right();
void Turn_Left();

//Pivot funtions
void Pivot_Set_Angle(int degree);
//Compost mechanism functions
void Compost_Set_Speed(double percent);

//void Course();

//Pivot functions
void Pivot_Set_Angle(int degree){
    arm.SetDegree(degree);
    return;
}

//Compost mechanism functions
void Compost_Set_Speed(double percent){
    compost.SetPercent(percent);
    return;
}

 


#define START_LIGHT 1
/* void startButton() {
    float startTime = TimeNow();
    float currentTime = 0;
    float startCondition = 0;
    float lightReading();

    while(currentTime < 10) {
        float lightReading = CdS_cell.Value();

        if(lightReading < START_LIGHT) {
            Drive(REVERSE, 0.25, 5);
            startCondition = 1;
        }
        currentTime = startTime - TimeNow();
    }

    if(startCondition == 0) {
        Drive(REVERSE, 0.25, 5);
        startCondition = 1;
    }
} */

void Drive(Direction dir, double speed, double distance){

    // Robot motion commands
    double Vx = 0.0;    // forward (+forward, -back)
    double Vy = 0.0;    // left (+left, -right)
    double omega = 0.0; // rotation (+CCW, -CW)

    switch (dir)
    {
    case FORWARD:
        Vx = speed;
        break;

    case REVERSE:
        Vx = -speed;
        break;

    case LEFT:
        Vy = speed;
        break;

    case RIGHT:
        Vy = -speed;
        break;

    case LEFT_F:
        Vx = speed;
        Vy = speed;
        break;

    case LEFT_R:
        Vx = -speed;
        Vy = speed;
        break;

    case RIGHT_F:
        Vx = speed;
        Vy = -speed;
        break;

    case RIGHT_R:
        Vx = -speed;
        Vy = -speed;
        break;

    default:
        LCD.WriteLine("Direction not specified during drive function");
        return;
    }

    // Kiwi drive wheel speeds
    double wheel1 = Vx + omega;
    double wheel2 = -0.5 * Vx + 0.8660254 * Vy + omega;
    double wheel3 = -0.5 * Vx - 0.8660254 * Vy + omega;

    // Keep wheel output in [-100, 100] while preserving direction ratios.
    double maxMag = fmax(fabs(wheel1), fmax(fabs(wheel2), fabs(wheel3)));
    if(maxMag > 100.0){
        double scale = 100.0 / maxMag;
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

    double target_counts = distance * ((fabs(L_ENCODE_P_IN) + fabs(R_ENCODE_P_IN) + fabs(F_ENCODE_P_IN)));
    while(((left_encoder.Counts() + right_encoder.Counts() + front_encoder.Counts())) < target_counts){
        Sleep(0.005);
    }

    StopAll();
}

static void Turn_By_Encoder(bool turn_left, double angle_deg, double speed){
    if(angle_deg <= 0.0 || speed == 0){
        Stop(leftdrive);
        Stop(rightdrive);
        Stop(frontdrive);
        return;
    }

    double turn_speed = abs(speed);

    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    front_encoder.ResetCounts();

    if(turn_left){
        rightdrive.SetPercent(-turn_speed);
        leftdrive.SetPercent(turn_speed);
        frontdrive.SetPercent(turn_speed);
    } else {
        rightdrive.SetPercent(turn_speed);
        leftdrive.SetPercent(-turn_speed);
        frontdrive.SetPercent(-turn_speed);
    }

    double target_counts = angle_deg * TURN_COUNTS_PER_DEG;
    while((((double)left_encoder.Counts() + (double)right_encoder.Counts() + (double)front_encoder.Counts())) < target_counts){
        Sleep(0.005);
    }

    Stop(leftdrive);
    Stop(rightdrive);
    Stop(frontdrive);
}

void Turn_Left(double angle_deg, double speed){
    Turn_By_Encoder(true, angle_deg, speed);
    return;
}

void Turn_Right(double angle_deg, double speed){
    Turn_By_Encoder(false, angle_deg, speed);
    return;
}

void Turn_Left(){
    Turn_Left(90.0, 15);
    return;
}

void Turn_Right(){
    Turn_Right(90.0, 15);
    return;
}

void Stop(FEHMotor &motor){

    motor.SetPercent(0.0);

    return;

}

void StopAll(){

    rightdrive.SetPercent(0.0);

    leftdrive.SetPercent(0.0);

    frontdrive.SetPercent(0.0);

    return;

}


/* void humidifier() {
    float startTime = TimeNow();
    float condition = 0
    float lightReading;
    while(startTime < 3 & condition == 0) {
        lightReading = CdS_Cell.Value();

        if(lightReading < RED_LIGHT) {
            Drive(RIGHT_R, 0.1, 2);
            Drive(FORWARD, 0.1, 3);
            condition = 1;
        }

        else if (lightReading < RED_LIGHT && lightReading > BLUE_LIGHT && condition == 0) {
            Drive(LEFT_F, 0.1, 2);
            Drive(Forward, 0.1, 3);
            condition = 1; 
        }
        startTime = TimeNow();
    }

    if(condition == 0) {
        Drive(RIGHT_R, 0.1, 2);
        Drive(FORWARD, 0.1, 3);
        condition = 1;
    }
} */


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


    enum LineStates {
        MIDDLEOfLine,
        RIGHTOfLine,
        LEFTOfLine
    };


void ERCMain()

{

    int x, y;

    while(!LCD.Touch(&x, &y));

    while(1){
        LCD.WriteLine(CdS_cell.Value());
        Sleep(0.05);
    }
}
