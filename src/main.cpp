#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEH.h>
#include <FEHRCS.h>
#include <Arduino.h>
#include <math.h>

//Hello
#define SQRT3 1.73205081
#define SIN60 0.8660254
#define COS60 0.5
#define INV_SQRT2 0.70710678
#define Radian_Conversion (PI/180)
#define SQRT32 0.86602540378
#define BASECOUNT 39
#define COUNTS_PER_INCH 40.3860807722

// Declare things like Motors, Servos, etc. here
// For example:
// FEHMotor leftMotor(FEHMotor::Motor0, 6.0);
// FEHServo servo(FEHServo::Servo0);

#define RADIUS 3.91

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

//Declaring DC Motors
FEHMotor LEFTMOTOR(FEHMotor::Motor0,9.0); 
FEHMotor RIGHTMOTOR(FEHMotor::Motor1,9.0);
FEHMotor BACKMOTOR(FEHMotor::Motor2,9.0);

//Declaring Servo Motors
FEHServo BIG_SERVO(FEHServo::Servo0);
FEHServo CONTINUOUS_SERVO(FEHServo::Servo1);

//Declaring Encoders
DigitalEncoder LEFTENCODER(FEHIO::Pin9); 
DigitalEncoder RIGHTENCODER(FEHIO::Pin10); 
DigitalEncoder BACKENCODER(FEHIO::Pin8);

//Declaring CDS Cell
AnalogInputPin CdS_cell(FEHIO::Pin0);

void StopAll(); //stops the motion of all motors 
void Turn_Right(); 
void Turn_Left(); 
void startButton();
void simpleDrive(int speed, float time);
void simpleReverse(int speed, float time);
void startButton();
void humidifier();
void Milestone_3();
void lever();

/* void Drive_Forward();
void Drive_Back();
void Turn_Right();
void Turn_Left();
void Stop(); */

//after testing we will change these values to their correct encodings per inch, but just placeholders for now
#define R_ENCODE_P_IN ((318.0/7.874))
#define L_ENCODE_P_IN ((318.0/7.874))
#define F_ENCODE_P_IN ((318.0/7.874))


// Encoder counts needed per degree of robot rotation.
// Tune this value on your robot so angle turns are accurate.
#define TURN_COUNTS_PER_DEG 1.0


#define START_LIGHT 1.5
#define RED_LIGHT 2.0
#define BLUE_LIGHT_MIN 2.0
#define BLUE_LIGHT_MAX 2.6

void STOP(){
    LEFTMOTOR.SetPercent(0);
    RIGHTMOTOR.SetPercent(0);
    BACKMOTOR.SetPercent(0);
}

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

//Defining Useful Trig Values
#define RAD60 (PI/3)

//UNUSED FUNCTION
float TRIG_CALULATIONS(float x, float y){
    float drive_angle;
    float left_angle, left_x_multiplier, left_y_multiplier;
    float right_angle, right_x_multiplier, right_y_multiplier;
    float back_angle, back_x_multiplier, back_y_multiplier;
    float unit_direction_x, unit_direction_y;
    
    //Calculates the angle clockwise from the positive y-axis
    drive_angle = atan2(x,y);
    
    //calculates a unit vector for intended direction
    unit_direction_x = sin(drive_angle);
    unit_direction_y = cos(drive_angle);

    //Calculates angle used to determine x and y components of wheel forces
    left_angle = drive_angle + RAD60;
    right_angle = drive_angle + RAD60;
    back_angle = drive_angle;

    //Calculating Trig values for wheel power calculations
    left_x_multiplier = cos(left_angle);
    left_y_multiplier = sin(left_angle);

    right_x_multiplier = cos(right_angle);
    right_y_multiplier = sin(right_angle);

    back_x_multiplier = cos(back_angle);
    back_y_multiplier = sin(back_angle);
}

//Defining unit vector components for each wheel
#define LEFT_X -0.5
#define LEFT_Y (sin(-(2*(PI/3))))
#define RIGHT_X -0.5
#define RIGHT_Y (sin(2*(PI/3)))
#define BACK_X 1
#define BACK_Y 0

void SERVO_CALIBRATION(){
    //Sets Min Value
    BIG_SERVO.SetMin(500);

    //Sets Max Value
    BIG_SERVO.SetMax(1800);
}

//Calculates the x and y components of a unit direction vector in the intended direction of travel
float drive_angle, unit_direction_x, unit_direction_y;
void UNIT_DIRECTION_VECTOR(float x, float y){
    //Calculates the angle clockwise from the positive y-axis
    drive_angle = atan2(x,y);

    //Calculates a unit vector for intended direction
    unit_direction_x = sin(drive_angle);
    unit_direction_y = cos(drive_angle);
}

//Calculates motor power based on dot product.
float LEFT_POWER, RIGHT_POWER, BACK_POWER;
void DOT_PRODUCT(int POWER){
    LEFT_POWER = POWER * ((LEFT_X * unit_direction_x) + (LEFT_Y * unit_direction_y));
    RIGHT_POWER = POWER * ((RIGHT_X * unit_direction_x) + (RIGHT_Y * unit_direction_y));
    BACK_POWER = POWER * ((BACK_X * unit_direction_x) + (BACK_Y * unit_direction_y));
}

//Starts motors with specified percents
void START_MOTORS(){
    LEFTMOTOR.SetPercent(LEFT_POWER);
    RIGHTMOTOR.SetPercent(RIGHT_POWER);
    BACKMOTOR.SetPercent(BACK_POWER);
}

//Resets Encoder Counts to Zero
void ENCODER_RESET(){
    LEFTENCODER.ResetCounts();
    RIGHTENCODER.ResetCounts();
    BACKENCODER.ResetCounts();
}

//Prints Encoder Values to the ERC Screen
void ENCODER_PRINT(){
    LCD.Write("LEFT: "); LCD.Write(LEFTENCODER.Counts());
    LCD.Write("\nRIGHT:"); LCD.Write(RIGHTENCODER.Counts());
    LCD.Write("\nBACK: "); LCD.Write(BACKENCODER.Counts());
}

//Keeps track of encoder values using directional logic
int LEFT_COUNTS, RIGHT_COUNTS, BACK_COUNTS;
void ENCODER_DIRECTIONAL_UPDATE(){
    int LEFT_CYCLE_COUNTS, RIGHT_CYCLE_COUNTS, BACK_CYCLE_COUNTS;
    
    //Read encoder values from current cycle
    LEFT_CYCLE_COUNTS = LEFTENCODER.Counts();
    RIGHT_CYCLE_COUNTS = RIGHTENCODER.Counts();
    BACK_CYCLE_COUNTS = BACKENCODER.Counts();

    //Determines sign for encoder values and adjusts manual count respectively
    if(LEFT_POWER < 0){
        LEFT_COUNTS -= LEFT_CYCLE_COUNTS;
    }

    if(LEFT_POWER > 0){
        LEFT_COUNTS += LEFT_CYCLE_COUNTS;
    }

    if(RIGHT_POWER < 0){
        RIGHT_COUNTS -= RIGHT_CYCLE_COUNTS;
    }

    if(RIGHT_POWER > 0){
        RIGHT_COUNTS += RIGHT_CYCLE_COUNTS;
    }

    if(BACK_POWER < 0){
        BACK_COUNTS -= BACK_CYCLE_COUNTS;
    }

    if(BACK_POWER > 0){
        BACK_COUNTS += BACK_CYCLE_COUNTS;
    }

    ENCODER_RESET();
}

//Resets the manual encoder count
void ENCODER_RESET_MANUAL(){
    LEFT_COUNTS = 0;
    RIGHT_COUNTS = 0;
    BACK_COUNTS = 0;
}

//Calculates encoder coutns for each wheel
int RIGHT_INTENDED_COUNTS, LEFT_INTENDED_COUNTS, BACK_INTENDED_COUNTS;
void ENCODER_CALCULATE_COUNTS(float x, float y){
    LEFT_INTENDED_COUNTS = ((LEFT_X * (COUNTS_PER_INCH * x)) + (LEFT_Y * ((COUNTS_PER_INCH) * y)));
    RIGHT_INTENDED_COUNTS = ((RIGHT_X * (COUNTS_PER_INCH * x)) + (RIGHT_Y * ((COUNTS_PER_INCH) * y)));
    BACK_INTENDED_COUNTS = ((BACK_X * (COUNTS_PER_INCH * x)) + (BACK_Y * ((COUNTS_PER_INCH) * y)));
}

//Determines if end condition has been met
int DRIVE_CONDITION(){
    //Updates directional encoder values
    ENCODER_DIRECTIONAL_UPDATE();

    if(LEFT_INTENDED_COUNTS == 0){
        LEFT_COUNTS = 0;
    }

    if(RIGHT_INTENDED_COUNTS == 0){
        RIGHT_COUNTS = 0;
    }

    if(BACK_INTENDED_COUNTS == 0){
        BACK_COUNTS = 0;
    }

    if(fabs(LEFT_COUNTS) <= fabs(LEFT_INTENDED_COUNTS) && fabs(RIGHT_COUNTS) <= fabs(RIGHT_INTENDED_COUNTS) && fabs(BACK_COUNTS) <= fabs(BACK_INTENDED_COUNTS)){
        return 1;
    }
    else{
        return 0;
    }
}

//Corrects wheel power based on heading error
void POWER_CORRECT(){
    float LEFT_RATIO = 0, RIGHT_RATIO = 0, BACK_RATIO = 0;
    float MAX_PERCENT;
    float LEFT_CORRECTION_FACTOR, RIGHT_CORRECTION_FACTOR, BACK_CORRECTION_FACTOR;

    //Calculates ratio of counts completed vs total intended counts
    if(LEFT_INTENDED_COUNTS != 0){
        LEFT_RATIO = fabs((float) LEFT_COUNTS/LEFT_INTENDED_COUNTS);
    }
    
    if(RIGHT_INTENDED_COUNTS != 0){
        RIGHT_RATIO = fabs((float) RIGHT_COUNTS/RIGHT_INTENDED_COUNTS);
    }

    if(BACK_INTENDED_COUNTS != 0){
        BACK_RATIO = fabs((float) BACK_COUNTS/BACK_INTENDED_COUNTS);
    }

    //Detemines which encoder has compelted the highest percent of intended counts
    MAX_PERCENT = max(LEFT_RATIO, max(RIGHT_RATIO, BACK_RATIO));

    //Return if MAX_PERCENT is zero to avoid division by zero
    if(MAX_PERCENT != 0){
        //Calculates motor correction factors for all motors
        if(LEFT_RATIO > 0){
            LEFT_CORRECTION_FACTOR = fabs(MAX_PERCENT / LEFT_RATIO);
        }
        else{
            LEFT_CORRECTION_FACTOR = 1;
        }

        if(RIGHT_RATIO > 0){
            RIGHT_CORRECTION_FACTOR = fabs(MAX_PERCENT / RIGHT_RATIO);
        }
        else{
            RIGHT_CORRECTION_FACTOR = 1;
        }

        if(BACK_RATIO > 0){
            BACK_CORRECTION_FACTOR = fabs(MAX_PERCENT / BACK_RATIO);
        }
        else{
            BACK_CORRECTION_FACTOR = 1;
        }

        //UPDATES MOTOR POWERS
        LEFT_POWER *= LEFT_CORRECTION_FACTOR;
        RIGHT_POWER *= RIGHT_CORRECTION_FACTOR;
        BACK_POWER *= BACK_CORRECTION_FACTOR;
    }
}

//Prints directional encoder values to ERC
void ENCODER_PRINT_MANUAL(){
    LCD.Clear();
    LCD.Write("LEFT: "); LCD.Write(LEFT_COUNTS); LCD.Write("    INT: "); LCD.Write(LEFT_INTENDED_COUNTS);
    LCD.Write("\nRIGHT:"); LCD.Write(RIGHT_COUNTS); LCD.Write("    INT: "); LCD.Write(RIGHT_INTENDED_COUNTS);
    LCD.Write("\nBACK: "); LCD.Write(BACK_COUNTS); LCD.Write("    INT: "); LCD.Write(BACK_INTENDED_COUNTS);
}

//Compiles many functions to drive
void DRIVE(float x, float y, int POWER){
    //Calculate direction and respective motor powers
    UNIT_DIRECTION_VECTOR(x, y);
    DOT_PRODUCT(POWER);
    
    //Resets Encoder Values
    ENCODER_RESET_MANUAL();
    ENCODER_RESET();

    //Calulates encoder counts
    ENCODER_CALCULATE_COUNTS(x, y);

    //Start Motors
    START_MOTORS();

    //Correcting and adjusting path until end condition is met
    while(DRIVE_CONDITION() == 1){
        //Update Directional Encoder Values
        //ENCODER_DIRECTIONAL_UPDATE();

        //Corrects heading based on directional encoding
        POWER_CORRECT();

        //Applies updated motor powers
        START_MOTORS();

        //Sleep between loops
        Sleep(0.1);
    }

    //Get rid of this once encoders work
    STOP();

    ENCODER_PRINT_MANUAL();
}

//Pivot funtions
//void Pivot_Set_Angle(int degree);

//Compost mechanism functions
//void Compost_Set_Speed(double percent);

//Pivot fnctions
/*void Pivot_Set_Angle(int degree){
    arm.SetDegree(degree);
    return;
}*/

//Compost mechanism functions
/*void Compost_Set_Speed(double percent){
    compost.SetPercent(percent);
    return;
}*/

/*void startButton() {
    float startTime = TimeNow();
    float currentTime = 0;
    float startCondition = 0;
    float lightReading();

    while(CdS_cell.Value() > 1.5){}
    return;
}*/

void RotateDegrees(float angleDeg, float speed){
    const float ROBOT_RADIUS = 3.91;          // distance from center to wheel, adjust if needed
    const float MIN_SPEED = 12.0;              // adjust if needed
    const float SLOWDOWN_COUNTS = 40.0;        // start slowing near end

    // Convert degrees to radians
    float theta = angleDeg * PI / 180.0;

    // Wheel travel distance required
    float wheelDistance = ROBOT_RADIUS * fabs(theta);

    // Convert to encoder counts
    float targetCounts = wheelDistance * R_ENCODE_P_IN;

    // Reset encoders
    RIGHTENCODER.ResetCounts();
    BACKENCODER.ResetCounts();
    LEFTENCODER.ResetCounts();

    // Determine direction
    float direction = (theta > 0) ? 1.0f : -1.0f;

    while (true)
    {
        float c1 = fabs(RIGHTENCODER.Counts());
        float c2 = fabs(BACKENCODER.Counts());
        float c3 = fabs(LEFTENCODER.Counts());

        float avgCounts = (c1 + c2 + c3) / 3.0f;
        float remaining = targetCounts - avgCounts;

        if (remaining <= 0)
            break;

        // Slow down near the target to reduce overshoot
        float currentSpeed = speed;
        if (remaining < SLOWDOWN_COUNTS)
        {
            float scale = remaining / SLOWDOWN_COUNTS;
            if (scale < (MIN_SPEED / speed))
                scale = MIN_SPEED / speed;

            currentSpeed = speed * scale;
        }

        RIGHTMOTOR.SetPercent(direction * -currentSpeed);
        BACKMOTOR.SetPercent(direction * -currentSpeed);
        LEFTMOTOR.SetPercent(direction * -currentSpeed);

        Sleep(0.005);
    }

    STOP();
}

/*void StopAll(){

    rightdrive.SetPercent(0.0);

    leftdrive.SetPercent(0.0);

    frontdrive.SetPercent(0.0);

    return;

}*/

//Adam's attempt at allowing movement in ANY DIRECTION (VERY ROUGH TEST)
/*void DriveTEST(float Angle, float Speed, float Distance){
    Angle = Angle * Radian_Conversion;

    //Sets a multiplier based on the angle of each wheel
    float frontmult, rightmult, leftmult;
    frontmult = sin(Angle);
    rightmult = sin(Angle - (2*(PI/3)));
    leftmult = sin(Angle - (4*(PI/3)));

    //Determines the number of counts each wheel must travel
    int frontcount, rightcount, leftcount;
    frontcount = Distance * BASECOUNT * frontmult;
    rightcount = Distance * BASECOUNT * rightmult;
    leftcount = Distance * BASECOUNT * leftmult;

    //Prepping for actual moving loop to start
    if(frontcount == 0){
        frontcount = 1000;
    }
    if(rightcount == 0){
        rightcount = 1000;
    }
    if(leftcount == 0){
        leftcount = 1000;
    }

    //Resets the encoders before moving
    front_encoder.ResetCounts();
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    //Starting the motors (INCLUDING CORRECTION FACTORS)
    frontdrive.SetPercent(-(Speed * frontmult));
    rightdrive.SetPercent(-(Speed * rightmult));
    leftdrive.SetPercent(-((Speed * leftmult)));

    while(abs(front_encoder.Counts()) < abs(frontcount) && abs(right_encoder.Counts()) < abs(rightcount) && abs(left_encoder.Counts()) < abs(leftcount)){
        //Keep moving until counts reached
    }

    //Writing encoder counts to screen
    LCD.Clear();
    LCD.WriteLine(front_encoder.Counts());
    LCD.WriteLine(right_encoder.Counts());
    LCD.WriteLine(left_encoder.Counts());

    //Stops the motors
    StopAll();

    return;
}*/

int CDS_CHECK(){
        LCD.Clear();
        LCD.WriteLine(CdS_cell.Value());
        Sleep(0.3);

    if(CdS_cell.Value() < RED_LIGHT){
        return 0; //Red
    }
        else if(CdS_cell.Value() > BLUE_LIGHT_MIN && CdS_cell.Value() < BLUE_LIGHT_MAX){
        return 1; //Blue
    }
    else{
        return 2; //Red
    }

}

/*bool DriveTEST_Light(float Angle, float Speed, float Distance){
    int Light = 2;
    Angle = Angle * Radian_Conversion;

    // Sets a multiplier based on the angle of each wheel
    float frontmult, rightmult, leftmult;
    frontmult = sin(Angle);
    rightmult = sin(Angle - (2*(PI/3)));
    leftmult = sin(Angle - (4*(PI/3)));

    // Determines the number of counts each wheel must travel
    int frontcount, rightcount, leftcount;
    frontcount = Distance * BASECOUNT * frontmult;
    rightcount = Distance * BASECOUNT * rightmult;
    leftcount = Distance * BASECOUNT * leftmult;

    // Prepping for actual moving loop to start
    if(frontcount == 0){
        frontcount = 1000;
    }
    if(rightcount == 0){
        rightcount = 1000;
    }
    if(leftcount == 0){
        leftcount = 1000;
    }

    // Resets the encoders before moving
    front_encoder.ResetCounts();
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();

    // Starting the motors
    frontdrive.SetPercent(-(Speed * frontmult));
    rightdrive.SetPercent(-(Speed * rightmult));
    leftdrive.SetPercent(-(Speed * leftmult));

    while(abs(front_encoder.Counts()) < abs(frontcount) &&
          abs(right_encoder.Counts()) < abs(rightcount) &&
          abs(left_encoder.Counts()) < abs(leftcount))
    {
        // Check CDS during movement
        Light = CDS_CHECK();
        if(CDS_CHECK() != 2){
            StopAll();
            return true; // light found
        }
        Sleep(0.05);
    }

    // Writing encoder counts to screen
    LCD.Clear();
    LCD.WriteLine(front_encoder.Counts());
    LCD.WriteLine(right_encoder.Counts());
    LCD.WriteLine(left_encoder.Counts());

    // Stops the motors
    StopAll();

    return false; // move finished normally
}*/

/*void Milestone_2(){
    DriveTEST(180, 20.0, 1.0);

    DriveTEST(0, 20.0, 1.0);

    RotateDegrees(44.06, 25);

    DriveTEST(0, 25.0, 3.0);

    DriveTEST(90, 25.0, 1.0);

    DriveTEST(0, 50, 30.5);

    RotateDegrees((float) -87.5, 25);

    DriveTEST(0, 50, 13.2);

    
    double legLength = 0.1;
    double legStep = 0.1;
    double maxLegLength = 8.0;
    double maxLRLegLength = .3;
    double LRLen;
    int Light = CDS_CHECK();

    while (Light == 2 && legLength <= maxLegLength)
    {
        if(legLength > maxLRLegLength){
            LRLen = maxLRLegLength;
        } else {
            LRLen = legLength;
        }
        DriveTEST(180, 20.0, legLength);
        Light = CDS_CHECK();
        if (Light != 2) break;

        DriveTEST(90, 20.0, LRLen);
        Light = CDS_CHECK();
        if (Light != 2) break;

        legLength += legStep;

        DriveTEST(0, 20.0, legLength);
        Light = CDS_CHECK();
        if (Light != 2) break;

        DriveTEST(270, 20.0, LRLen);
        Light = CDS_CHECK();
        if (Light != 2) break;

        legLength += legStep;
    }
   
    
    if(Light == 0){ //Red
        LCD.Clear();
        LCD.WriteLine("Red Detected");
        Sleep(1.0);
        DriveTEST(90, 20.0, .8);
        DriveTEST(0, 20.0, 5.0);
        /*
        rightdrive.SetPercent(30);
        leftdrive.SetPercent(30);
        Sleep(2.0);
        StopAll(); 
        Sleep(1.0); 
        DriveTEST(180, 20.0, 5.0);
        DriveTEST(-90, 20.0, .8);
    }

    if(Light == 1){ //Blue
        LCD.Clear();
        LCD.WriteLine("Blue Detected");
        Sleep(1.0);
        DriveTEST(-90, 20.0, .8);
        DriveTEST(0, 20.0, 5.0);
        /*
        rightdrive.SetPercent(30);
        leftdrive.SetPercent(30);
        Sleep(2.0);
        StopAll(); 
        Sleep(1.0);  
        DriveTEST(180, 20.0, 5.0);
        DriveTEST(90, 20.0, .8);
    }

    DriveTEST(180, 20.0, 17.0);
    
    RotateDegrees(-95, 25);

    DriveTEST(0, 50.0, 45.0);

    DriveTEST(180, 20.0, 2.0);

    DriveTEST(-90, 50.0, 10.0);
}*/

/*void DriveRightTime(float speed, float time)
{
    float Angle = 90.0 * Radian_Conversion;

    // Kiwi wheel contributions (all 3 wheels active)
    float frontmult = sin(Angle);
    float rightmult = sin(Angle - (2 * PI / 3));
    float leftmult  = sin(Angle - (4 * PI / 3));

    // Set motor speeds
    frontdrive.SetPercent((speed * frontmult));
    rightdrive.SetPercent((speed * rightmult));
    leftdrive.SetPercent(-(speed * leftmult));

    Sleep(time);

    StopAll();
    
    return;
}*/

void WaitForTouch()
{
    int x, y;

    while (!LCD.Touch(&x, &y)) {}    // wait for press
}

/*void BacktoWall(float time){
    leftdrive.SetPercent(-50);
    frontdrive.SetPercent(50);

    Sleep(time);

    StopAll();
    return;
}*/

/*void FronttoWall(float time){
    leftdrive.SetPercent(50);
    frontdrive.SetPercent(-50);

    Sleep(time);

    StopAll();
    return;
}*/

/*void Ramp(float time){
    rightdrive.SetPercent(75);
    frontdrive.SetPercent(-75);

    Sleep(time);

    StopAll();

    return;
}*/

/*void Milestone_3(){
    WaitForTouch();
    
    //Start Button
    startButton();
    DriveTEST(180, 20.0, 1.0);
    DriveTEST(0, 20.0, 1.0);

    RotateDegrees(-15, 25);

    DriveXY(6, 0, 75);
    DriveXY(0, 5, 75);

    RotateDegrees(10, 75);

    //DriveXY(40, 0, 75);

    Ramp(2.0);

    RotateDegrees(-120, 75);

    BacktoWall(1.5);

    DriveTEST(210, 50, 5);

    RotateDegrees(-15, 50);

    BacktoWall(0.2);

    Sleep(1.0);

    FronttoWall(1.15);

    Sleep(1.0);

    RotateDegrees(-30, 50);

    Sleep(1.0);

    Ramp(0.4);

    Sleep(1.0);

    DriveTEST(135, 25, 2);

    Sleep(1.0);

    RotateDegrees(80, 65);

    Ramp(0.1);

    RotateDegrees(-150, 50);
}*/

/*void lever() {
    int correctLever = RCS.GetLever();
    Pivot_Set_Angle(Lever_Up_ANGLE);

    if(correctLever == 0) { // left lever
        LCD.WriteLine("Left Lever");
        //drive left to lever
        Pivot_Set_Angle(Lever_Down_ANGLE);
        Sleep(5.0);
        //drive backwards slightly
        Pivot_Set_Angle(Lever_Down_ANGLE);
        //drive fowards slightly
        Pivot_Set_Angle(Lever_Up_ANGLE);
        // drive backwards and to the right to initial position (for consistency)
    }
    else if(correctLever == 1) { // middle lever
        LCD.WriteLine("Middle Lever");
        Pivot_Set_Angle(Lever_Down_ANGLE);
        Sleep(5.0);
        // drive backwards slightly
        Pivot_Set_Angle(Lever_Down_ANGLE); // should be lower than previous one
        //drive forwards slightly
        Pivot_Set_Angle(Lever_Up_ANGLE);
        //drive backward to initial position
    }
    else if(correctLever == 2) { //right lever
        LCD.WriteLine("Right Lever");
        //drive right to lever
        Pivot_Set_Angle(Lever_Down_ANGLE);
        Sleep(5.0);
        //drive backwards slightly
        Pivot_Set_Angle(Lever_Down_ANGLE); // lower than previous angle
        // drive fowards slightly
        Pivot_Set_Angle(Lever_Up_ANGLE);
        //drive backward and then left to initial position (for consistency)
    }

    // correct heading?

}*/

/*void Reset_Counts(){
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    front_encoder.ResetCounts();
    return;
}*/

/*void Encoder_test(){
    //Reset encoder counts
    Reset_Counts();
    
    //Setting all motors to default forward
    rightdrive.SetPercent(-25);
    leftdrive.SetPercent(-25);
    frontdrive.SetPercent(-25);

    //Printing encoder counts every 0.1 seconds
    while(true){
        LCD.Clear();
        LCD.Write("Front: "); LCD.Write(front_encoder.Counts());
        LCD.Write("\nRight: "); LCD.Write(right_encoder.Counts());
        LCD.Write("\nLeft: "); LCD.Write(left_encoder.Counts());

        Sleep(0.1);
    }
}*/

void START_BUTTON(){
    while(CdS_cell.Value() > RED_LIGHT);

    LCD.WriteLine("START LIGHT DETECTED");
    
    DRIVE(0, -.5, 25);

    DRIVE(0, 1.5, 25);
}

//Drives from start to Apple Basket
void DRIVE_TO_APPLE_BASKET(){
    DRIVE(0, 18, 50);
    
    //Rotate Robot to correct orientation
    RotateDegrees(-45, 25);
}

//Completes the Apple Basket Task
void APPLE_BASKET(){
    //Drives from start to Apple Basket
    DRIVE_TO_APPLE_BASKET();

    //Set Big Servo to initial angle
    BIG_SERVO.SetDegree(80);
    
    Sleep(1.0);

    //Drive hook under basket handle
    DRIVE(0, 3.0, 15);

    //Pick up basket
    BIG_SERVO.SetDegree(70);

    Sleep(0.5);

    BIG_SERVO.SetDegree(60);

    Sleep(0.5);

    BIG_SERVO.SetDegree(50);

    Sleep(0.5);

    BIG_SERVO.SetDegree(40);

    Sleep(0.5);

    BIG_SERVO.SetDegree(30);

    Sleep(1.0);

    DRIVE(-5, -21, 50);

    Sleep(0.1);

    RotateDegrees(90, 25);

    Sleep(0.5);

    //Drive up Ramp
    DRIVE(0, 35, 50);

    DRIVE(3.5, 0, 25);

    LEFTMOTOR.SetPercent(-25);
    RIGHTMOTOR.SetPercent(25);

    Sleep(1.0);

    STOP();

    //Set Apple Basket Down
    BIG_SERVO.Off();

    Sleep(1.0);

    //Drive away
    DRIVE(0, -5, 15);
}

void ROBOT_CALIBRATION(){
    //Initializes the RCS system
    RCS.InitializeTouchMenu("1130D6KKR");

    //Set servo 90 degrees
    BIG_SERVO.SetDegree(90);
}

//Flips lever down and up
void FLIP_LEVER(){
    //Flips lever down
    BIG_SERVO.SetDegree(135);

    //Move lever arm below fertilizer lever
    RotateDegrees(-5, 10);
    BIG_SERVO.SetDegree(105);  
    RotateDegrees(5, 10);

    //Waits 5 seconds
    Sleep(5.0);

    //Flips lever up
    BIG_SERVO.SetDegree(90); //ADJUST ANGLE
}

//Completes the Fertilizer Lever Task
void LEVER(){
    //NEED CODE TO DRIVE TO LEVERS
    DRIVE(-5, 0, 50);

    RotateDegrees(-45, 25);

    //Set servo arm to  prepare for flipping lever down
    BIG_SERVO.SetDegree(60);

    DRIVE(0, 13, 50);

    //Reads lever information
    int CORRECT_LEVER = RCS.GetLever();

    //Process for left lever
    if(CORRECT_LEVER == 0){
        //Prints correct lever to screen
        LCD.Clear();
        LCD.WriteLine("LEFT LEVER READ");

        //Sleep for testing
        //DELETE LATER
        Sleep(1.0);

        //Align with left lever
        DRIVE(-2, 0, 25);

        FLIP_LEVER();
    }

    //Process for middle lever
    if(CORRECT_LEVER == 1){
        //Prints correct lever to screen
        LCD.Clear();
        LCD.WriteLine("MIDDLE LEVER READ");

        //Sleep for testing
        //DELETE LATER
        Sleep(1.0);

        FLIP_LEVER();
    }

    //Process for right lever
    if(CORRECT_LEVER == 2){
        //Prints correct lever to screen
        LCD.Clear();
        LCD.WriteLine("RIGHT LEVER READ");

        //Sleep for testing
        //DELETE LATER
        Sleep(1.0);

        //Align with right lever
        DRIVE(2, 0, 25);

        FLIP_LEVER();
    }
}

//Completes the compost mechanism task
void COMPOST(){
    //Rotate mechanism towards bin
    RotateDegrees(70, 25);

    //Drive to Compost Bin
    DRIVE(-SQRT32 * 6, -6/2, 50);
    Sleep(0.5);
    DRIVE(4.5/2, -SQRT32 * 4.5, 50);
    Sleep(0.5);
    DRIVE(-SQRT32 * 3.5, -3.5/2, 50);
    Sleep(0.5);
    RotateDegrees(-5, 25);
 
    CONTINUOUS_SERVO.SetDegree(95);
    Sleep(1.5);
    CONTINUOUS_SERVO.SetDegree(70);
    Sleep(1.5);
    CONTINUOUS_SERVO.Off();

    DRIVE(SQRT32 * 11, 11/2, 50);



}

void ERCMain()
{   
    START_BUTTON();

    COMPOST();
}