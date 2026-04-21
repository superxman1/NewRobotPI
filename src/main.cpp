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
#define Servo_Max_Angle 135
#define Servo_Min_Angle 70

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
void DRIVE(float x, float y, int POWER);
void DriveFieldRelative(float headingDeg, float fieldX, float fieldY, int power);
void RotateDegrees(float angleDeg, float speed);
void startButton();
void humidifier();
void Milestone_3();
void lever();
int RCSData();
void Distance_Calc();
void RCSFunction();
void FLIP_LEVER();

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
#define BLUE_LIGHT_MIN 1.6
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
    BIG_SERVO.SetMin(1315);

    //Sets Max Value
    BIG_SERVO.SetMax(2025);
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

//Check distance and initiates PID
float LEFT_TRAVELLED, RIGHT_TRAVELLED, BACK_TRAVELLED, MAX_TRAVELLED;
void DISTANCE_UPDATE(){
    LEFT_TRAVELLED = sqrt(pow(((LEFT_X * (LEFT_COUNTS/COUNTS_PER_INCH)) + (LEFT_Y * (LEFT_COUNTS/COUNTS_PER_INCH))), 2));
    RIGHT_TRAVELLED = sqrt(pow(((RIGHT_X * (RIGHT_COUNTS/COUNTS_PER_INCH)) + (RIGHT_Y * (RIGHT_COUNTS/COUNTS_PER_INCH))), 2));
    BACK_TRAVELLED = sqrt(pow(((BACK_X * (RIGHT_COUNTS/COUNTS_PER_INCH)) + (BACK_Y * (RIGHT_COUNTS/COUNTS_PER_INCH))), 2));

    //Calculating the average of all distances travelled
    MAX_TRAVELLED = max(LEFT_TRAVELLED, max(RIGHT_TRAVELLED, BACK_TRAVELLED));
}

float SLOW_MULTIPLIER;
void SLOWDOWN_CHECK(){
    //Decreases speed based on distance from target
    if(MAX_TRAVELLED <= 3.5){
        //Calculates a slowdown multiplier based on distance from target
        SLOW_MULTIPLIER = MAX_TRAVELLED / 3.5;
        
        //If the multiplier is below 0.5, then don't slow down the motors more
        if(SLOW_MULTIPLIER <= 0.5){
            SLOW_MULTIPLIER = 1;
        }

        //Adjusts motor powers
        LEFT_POWER *= SLOW_MULTIPLIER;
        RIGHT_POWER *= SLOW_MULTIPLIER;
        BACK_POWER *= SLOW_MULTIPLIER;
    }
}

//Corrects wheel power based on heading error
void POWER_CORRECT(){
    float LEFT_RATIO = 0, RIGHT_RATIO = 0, BACK_RATIO = 0;
    float MAX_PERCENT;
    float LEFT_CORRECTION_FACTOR, RIGHT_CORRECTION_FACTOR, BACK_CORRECTION_FACTOR;
    float SLOW_MULTIPLIER;

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
        //Updates distance remaining
        DISTANCE_UPDATE();

        //Corrects heading based on directional encoding
        //POWER_CORRECT();

        //Slows down the robot before reaching target
        //SLOWDOWN_CHECK();

        //Applies updated motor powers
        START_MOTORS();

        //Sleep between loops
        Sleep(0.1);
    }

    //Get rid of this once encoders work
    STOP();

    ENCODER_PRINT_MANUAL();
}

//Define a global variable to keep track of heading (in degrees, CCW positive)
float Robot_Heading = 0;
float X_POS = 0;
float Y_POS = 0;
int increment = 0;

struct CourseCoordinates {
    float CourseHeading;
    float CourseX;
    float CourseY;
};

struct CourseCoordinates p1 = {300, 21.64, 49.25}; //Pre-determined course values
// Create struct for each desired position

struct CourseCoordinates HUMIDIFIER_LOCATION = {0, 13.95, 49.87};
struct CourseCoordinates APPLE_LOCATION = {57, 10.69, 19.97};
struct CourseCoordinates BOTTOM_RAMP_LOCATION = {330, 32.10, 16.18};
struct CourseCoordinates TOP_RAMP_LOCATION = {321, 33.26, 54.87};
struct CourseCoordinates LEVER_LOCATION = {10, 17.15, 58.64};
struct CourseCoordinates LEFT_LEVER_LOCATION = {16, 8.89, 57.17};
struct CourseCoordinates MIDDLE_LEVER_LOCATION = {12, 13.10, 60.08};
struct CourseCoordinates RIGHT_LEVER_LOCATION = {12, 15.87, 64.45};
struct CourseCoordinates RED_LIGHT_LOCATION = {0, 9.18, 51.45};
struct CourseCoordinates BLUE_LIGHT_LOCATION = {0, 9.22, 47.73};
struct CourseCoordinates WINDOW_LOCATION = {145, 17.65, 44.98};
struct CourseCoordinates FINAL_Top_Ramp_LOCATION = {330, 31.12, 49.78};
struct CourseCoordinates Window_PRE_Close = {150, 10.21, 45.42};
struct CourseCoordinates Window_POST_Close = {150, 11.3, 45.4};
struct CourseCoordinates FINAL_Button_LOCATION = {105, 29.50, 7.30};

int RCSData() {
    Sleep(.5);
    RCSPose* pose = RCS.RequestPosition();
    if (pose == nullptr || (pose->x < 0 && pose->y < 0 && pose->heading < 0)) {
        LCD.WriteLine("No RCS data received.");
        return 0;
    }
    Robot_Heading = pose->heading;
    X_POS = pose->x;
    Y_POS = pose->y;
    return 1;
}

float DesiredHeading = 0;
float DesiredX = 0;
float DesiredY = 0;

void Distance_Calc(CourseCoordinates *target) {
    DesiredHeading = target->CourseHeading - Robot_Heading;

    if(DesiredHeading > 180) {
        DesiredHeading -= 360;
    }
    if(DesiredHeading < -180) {
        DesiredHeading += 360;
    }

    DesiredX = target->CourseX - X_POS;
    DesiredY = target->CourseY - Y_POS;
}

void RCSFunction(CourseCoordinates *target) {
    if (RCSData() == 1) {
        Distance_Calc(target);
    }
}

void RCSFunctionRotate(CourseCoordinates *target, int speed) {
    if (RCSData() == 1) {
        Distance_Calc(target);
        RotateDegrees(DesiredHeading, speed);
    }
}

void RCSFunctionDrive(CourseCoordinates *target, int speed, float distMultiplier) {
    if (RCSData() == 1) {
        Distance_Calc(target);
        DriveFieldRelative(Robot_Heading, DesiredX * distMultiplier, DesiredY * distMultiplier, speed);
    }
}



void DriveFieldRelative(float headingDeg, float fieldX, float fieldY, int power)
{
    // Convert heading to radians and adjust for field-relative control
    float headingOffsetDeg = 325;
    float HEADING = (headingDeg - headingOffsetDeg) * PI / 180.0f;

    float robotX =  fieldX * cos(HEADING) + fieldY * sin(HEADING);
    float robotY = -fieldX * sin(HEADING) + fieldY * cos(HEADING);

    DRIVE(robotX, robotY, power);
}

//Rotates the robot to a specific angle
void RotateDegrees(float angleDeg, float speed){
    //CCW is positive
    const float ROBOT_RADIUS = 3.92;          // distance from center to wheel, adjust if needed
    const float MIN_SPEED = 12.0;              // adjust if needed
    const float SLOWDOWN_COUNTS = 40.0;        // start slowing near end

    // Update robot heading
    Robot_Heading = Robot_Heading + angleDeg;

    // Convert degrees to radians
    float theta = -angleDeg * PI / 180.0;

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
        float remaining1 = targetCounts - c1;
        float remaining2 = targetCounts - c2;
        float remaining3 = targetCounts - c3;

        if (remaining1 <= 0 || remaining2 <= 0 || remaining3 <= 0)
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

        Sleep(0.01);
    }

    STOP();
}

int HUMIDIFIER_LIGHT(){
        LCD.Clear();
        LCD.WriteLine(CdS_cell.Value());
        Sleep(0.3);

        LCD.Clear();
    if(CdS_cell.Value() < BLUE_LIGHT_MIN){
        //Writes information to screen
        LCD.WriteLine("RED LIGHT DETECTED");
        LCD.WriteLine(CdS_cell.Value());

        RCSFunctionDrive(&RED_LIGHT_LOCATION, 50, 1.0);

        return 0; //Red
    }
        else if(CdS_cell.Value() > BLUE_LIGHT_MIN && CdS_cell.Value() < BLUE_LIGHT_MAX){
        //Writes information to screen
        LCD.WriteLine("BLUE LIGHT DETECTED");
        LCD.WriteLine(CdS_cell.Value());
        
        RCSFunctionDrive(&BLUE_LIGHT_LOCATION, 50, 1.0);

        return 1; //Blue
    }
    else{
        //Writes information to screen
        LCD.WriteLine("ERROR; NOT IN RANGE");
        LCD.WriteLine(CdS_cell.Value());

        return 2; //NULL
    }

}

void BIG_SERVO_ROTATE(float angle){
    if(angle > Servo_Max_Angle){
        angle = Servo_Max_Angle;
    }
    else if(angle < Servo_Min_Angle){
        angle = Servo_Min_Angle;
    }

    BIG_SERVO.SetDegree(angle);
}


void ROBOT_CALIBRATION(){
    //Initializes the RCS system
    RCS.InitializeTouchMenu("1130D6KKR");

    //Set servo up degrees
    BIG_SERVO_ROTATE(75);
}

//Flips lever down and up
void FLIP_LEVER(){
    //Flip lever down
    BIG_SERVO_ROTATE(132);

    float startTime = TimeNow();

    Sleep(1.0);

    BIG_SERVO_ROTATE(121);

    Sleep(0.5);

    RotateDegrees(50, 50);

    BIG_SERVO.SetDegree(137);

    Sleep(.5);


    RotateDegrees(-50, 50);

    DriveFieldRelative(Robot_Heading, -.5, .5, 50);

    while(TimeNow() - startTime < 5.25){
        Sleep(0.01);
    }

    BIG_SERVO_ROTATE(120);

    Sleep(0.1);

    DriveFieldRelative(Robot_Heading, 2, -4, 50);
}

void WaitForTouch()
{
    int x, y;

    while (!LCD.Touch(&x, &y)) {}    // wait for press
}

//Completes start button sensing
void START_BUTTON(){
    while(CdS_cell.Value() > RED_LIGHT);

    LCD.WriteLine("START LIGHT DETECTED");
    Robot_Heading = 45;
    DRIVE(0, -.4, 50);

    DRIVE(0, 1.5, 50);

    RotateDegrees(-72, 50);
}

//Completes compost task
void COMPOST(){

    //Drive to Compost Bin
    DRIVE(-SQRT32 * 6, -6/2, 50);
    DRIVE(5/2, -SQRT32 * 5, 50);
    DRIVE(-SQRT32 * 3.5, -3.5/2, 50);
    RotateDegrees(5, 25);
 
    //Rotate continuous servo both directions
    CONTINUOUS_SERVO.SetDegree(105);
    Sleep(1.3);
    CONTINUOUS_SERVO.SetDegree(65);
    Sleep(1.3);
    CONTINUOUS_SERVO.Off();

    //Back away
    Robot_Heading = 300;
    DRIVE(SQRT32 * 2, 1, 50);
}


//Completes the Apple Basket Task
void APPLE_BASKET(){
    //Drives from humidifier to apple basket
    RotateDegrees(117, 50);
    
    DriveFieldRelative(Robot_Heading, -6.7, 15.25, 75);

    RCSFunctionRotate(&APPLE_LOCATION, 50);

    BIG_SERVO_ROTATE(112);

    RCSFunctionDrive(&APPLE_LOCATION, 50, 0.9);

    //Correct error
    RCSFunctionDrive(&APPLE_LOCATION, 25, 1.0);

    DriveFieldRelative(Robot_Heading, -1, 0, 50);

    BIG_SERVO_ROTATE(70);

    Sleep(0.25);

    DriveFieldRelative(Robot_Heading, 17.41, -6.79, 75);

    RCSFunctionRotate(&BOTTOM_RAMP_LOCATION, 50);

    RCSFunctionDrive(&BOTTOM_RAMP_LOCATION, 50, 1.0);

    //Drive up ramp
    DriveFieldRelative(Robot_Heading, -4, 32, 65);

    RCSFunctionRotate(&TOP_RAMP_LOCATION, 50);

    RCSFunctionDrive(&TOP_RAMP_LOCATION, 75, 1.0);

    DriveFieldRelative(Robot_Heading, 0, 3, 75);

    BIG_SERVO_ROTATE(80);

    Sleep(0.1);

    BIG_SERVO.Off();

    DriveFieldRelative(Robot_Heading, 0, -5, 75);
}

//Completes the Fertilizer Lever Task
void LEVER(){
    //Sleep(0.5);
    
    DriveFieldRelative(Robot_Heading, -14.11, 6.67, 75);

    //Corrects location and heading
    RCSFunctionRotate(&LEVER_LOCATION, 50);
    RCSFunctionDrive(&LEVER_LOCATION, 50, .9);


    BIG_SERVO_ROTATE(100);

    //Reads lever information
    int CORRECT_LEVER = RCS.GetLever();

    //Process for left lever
    if(CORRECT_LEVER == 0){
        //Prints correct lever to screen
        LCD.Clear();
        LCD.WriteLine("LEFT LEVER READ");

        //Moves to correct lever
        RCSFunctionRotate(&LEFT_LEVER_LOCATION, 50);
        RCSFunctionDrive(&LEFT_LEVER_LOCATION, 25, 0.82);
    }

    //Process for middle lever
    if(CORRECT_LEVER == 1){
        //Prints correct lever to screen
        LCD.Clear();
        LCD.WriteLine("MIDDLE LEVER READ");

        //Moves to correct lever
        RCSFunctionRotate(&MIDDLE_LEVER_LOCATION, 50);
        RCSFunctionDrive(&MIDDLE_LEVER_LOCATION, 25, 0.85);
    }

    //Process for right lever
    if(CORRECT_LEVER == 2){
        //Prints correct lever to screen
        LCD.Clear();
        LCD.WriteLine("RIGHT LEVER READ");

        //Moves to correct lever
        RCSFunctionRotate(&RIGHT_LEVER_LOCATION, 50);
        RCSFunctionDrive(&RIGHT_LEVER_LOCATION, 25, 0.92);
    }

    FLIP_LEVER();

    
}


void HUMIDIFIER(){
    BIG_SERVO_ROTATE(70);
    
    //Drive to humidifier location from any lever
    RCSFunctionDrive(&HUMIDIFIER_LOCATION, 50, 1.0);

    //Correct heading
    RCSFunctionRotate(&HUMIDIFIER_LOCATION, 50);

    //Correct error
    RCSFunctionDrive(&HUMIDIFIER_LOCATION, 25,1.0);

    //Correct error
    RCSFunctionDrive(&HUMIDIFIER_LOCATION, 25, .85);


    //Reads CDS value for humidifer light
    if(HUMIDIFIER_LIGHT() == 2){
        RCSFunctionDrive(&HUMIDIFIER_LOCATION, 25, .85);
        HUMIDIFIER_LIGHT();
    }
}

void WINDOW(){
     
    //Back away from the humidifier
    DriveFieldRelative(Robot_Heading, 4, 0, 50);

    //Correct heading
    RCSFunctionRotate(&WINDOW_LOCATION, 50);

    BIG_SERVO_ROTATE(110);

    RCSFunctionDrive(&WINDOW_LOCATION, 50, 1.0);

    //Correct error
    RCSFunctionDrive(&WINDOW_LOCATION, 25,.80);

    //Correct error
    //RCSFunctionDrive(&WINDOW_LOCATION, 25, .85);

    RotateDegrees(-50, 65);

    RCSFunctionRotate(&Window_PRE_Close, 50);

    RCSFunctionDrive(&Window_PRE_Close, 75, 1.0);

    RCSFunctionDrive(&Window_POST_Close, 75, 1.0);

    //RotateDegrees(40, 65);

    RCSFunctionRotate(&Window_POST_Close, 50);

    DriveFieldRelative(Robot_Heading, 6, 0, 50);
}


void FinalButton(){

    BIG_SERVO_ROTATE(75);

    RCSFunctionRotate(&FINAL_Top_Ramp_LOCATION, 50);

    RCSFunctionDrive(&FINAL_Top_Ramp_LOCATION, 40, .9);

    DriveFieldRelative(Robot_Heading, -2, 0, 25);

    RCSFunctionDrive(&FINAL_Button_LOCATION, 75, 1);

    RotateDegrees(60, 75);

    DriveFieldRelative(Robot_Heading, 4, -4, 50);

    RCSFunctionRotate(&FINAL_Button_LOCATION, 50);

    RCSFunctionDrive(&FINAL_Button_LOCATION, 50, 1.1);
}

//Completes the compost mechanism task


void ERCMain()
{
    int i = 135;

    
    
    //RCS AND ROBOT CALIBRATION
    ROBOT_CALIBRATION();
    WaitForFinalAction();
    Sleep(0.5);

    //Reads start button value
    START_BUTTON();

    //Completes compost task
    COMPOST();

    //Completes apple basket task
    APPLE_BASKET();
    
    //Completes lever task
    LEVER();

    //Completes humidifier task
    HUMIDIFIER();

    WINDOW();

    FinalButton();
    
}