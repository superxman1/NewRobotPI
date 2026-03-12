#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHSD.h>
#include <FEH.h>

#include <Arduino.h>
#include <math.h>

//Hello

#define Radian_Conversion (PI/180)
#define BASECOUNT 39
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

DigitalEncoder front_encoder(FEHIO::Pin10); 
DigitalEncoder right_encoder(FEHIO::Pin9); 
DigitalEncoder left_encoder(FEHIO::Pin8);

FEHMotor compost(FEHMotor::Motor3,5.0);
FEHServo arm(FEHServo::Servo0);

AnalogInputPin CdS_cell(FEHIO::Pin0);

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
#define R_ENCODE_P_IN ((318.0/7.874))
#define L_ENCODE_P_IN ((318.0/7.874))
#define F_ENCODE_P_IN ((318.0/7.874))

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

 


#define START_LIGHT 1.5
#define RED_LIGHT 1.75
#define BLUE_LIGHT_MIN 1.75
#define BLUE_LIGHT_MAX 2.5

void startButton() {
    float startTime = TimeNow();
    float currentTime = 0;
    float startCondition = 0;
    float lightReading();

    while(CdS_cell.Value() > 1.5){}
    return;
} 

void Drive(Direction dir, double speed, double distance)
{
    // Commanded robot-frame direction unit vector
    double ux = 0.0;
    double uy = 0.0;

    // Motor command components
    double Vx = 0.0;
    double Vy = 0.0;
    double omega = 0.0;

    const double INV_SQRT2 = 0.70710678;
    const double SQRT3 = 1.73205081;

    switch (dir)
    {
    case FORWARD:
        ux = 1.0;  uy = 0.0;
        Vx = speed;
        break;

    case REVERSE:
        ux = -1.0; uy = 0.0;
        Vx = -speed;
        break;

    case LEFT:
        ux = 0.0;  uy = -1.0;
        Vy = -speed;
        break;

    case RIGHT:
        ux = 0.0;  uy = 1.0;
        Vy = speed;
        break;

    case LEFT_F:
        ux = INV_SQRT2;  uy = -INV_SQRT2;
        Vx = speed * INV_SQRT2;
        Vy = -speed * INV_SQRT2;
        break;

    case LEFT_R:
        ux = -INV_SQRT2; uy = -INV_SQRT2;
        Vx = -speed * INV_SQRT2;
        Vy =  -speed * INV_SQRT2;
        break;

    case RIGHT_F:
        ux = INV_SQRT2;  uy = INV_SQRT2;
        Vx = speed * INV_SQRT2;
        Vy = speed * INV_SQRT2;
        break;

    case RIGHT_R:
        ux = -INV_SQRT2; uy = INV_SQRT2;
        Vx = -speed * INV_SQRT2;
        Vy = speed * INV_SQRT2;
        break;

    default:
        LCD.WriteLine("Direction not specified during drive function");
        return;
    }

    // Kiwi drive forward kinematics: body command -> wheel commands
    double wheel1 = (-1.0 * Vy + omega) * 100.0;               // bottom wheel
    double wheel2 = ( 0.8660254 * Vx + 0.5 * Vy + omega) * 100.0; // right wheel
    double wheel3 = (-0.8660254 * Vx + 0.5 * Vy + omega) * 100.0; // left wheel
    
    LCD.WriteLine(wheel1);
    LCD.WriteLine(wheel2);      
    LCD.WriteLine(wheel3);

    // Reset encoders
    right_encoder.ResetCounts(); // wheel1
    left_encoder.ResetCounts();  // wheel2
    front_encoder.ResetCounts(); // wheel3

    

    // Start motors
    rightdrive.SetPercent(-wheel3);
    leftdrive.SetPercent(-wheel1);
    frontdrive.SetPercent(-wheel2);

    while (true)
    {
        // Convert signed encoder counts -> signed wheel travel in inches
        double s1 = -left_encoder.Counts() / R_ENCODE_P_IN; // wheel1
        double s2 = front_encoder.Counts()  / L_ENCODE_P_IN; // wheel2
        double s3 = -right_encoder.Counts() / F_ENCODE_P_IN; // wheel3


        
        // Inverse kiwi kinematics: wheel travel -> robot displacement
        double dx = (s2 - s3) / SQRT3;
        double dy = ((-2.0 * s1) + s2 + s3) / 3.0;

        // Progress along commanded direction
        double progress = dx * ux + dy * uy;

        
        if (progress >= distance)
        {
            LCD.Clear();
        LCD.Write("s1: "); LCD.WriteLine(s1);
        LCD.Write("s2: "); LCD.WriteLine(s2);
        LCD.Write("s3: "); LCD.WriteLine(s3);
        LCD.Write("dx: "); LCD.WriteLine(dx);
        LCD.Write("dy: "); LCD.WriteLine(dy);
        LCD.Write("p: "); LCD.WriteLine(progress);
        LCD.Write("rencoder: "); LCD.WriteLine(right_encoder.Counts());
        LCD.Write("lencoder: "); LCD.WriteLine(left_encoder.Counts());
        LCD.Write("fencoder: "); LCD.WriteLine(front_encoder.Counts());
        //Sleep(.2);

            break;
        }

        Sleep(0.005);
    }

    StopAll();
}

/*
 * DriveXY()
 *
 * Drives the robot to a target displacement in the robot's local coordinate frame.
 *
 * Coordinate system:
 *  +X = forward
 *  -X = backward
 *  +Y = left
 *  -Y = right
 *
 * Inputs:
 *  xTarget  -> desired forward/backward travel (inches)
 *  yTarget  -> desired left/right travel (inches)
 *  speed    -> commanded drive speed (motor percent scale)
 */

void DriveXY(double xTarget, double yTarget, double speed)
{
    const double SQRT3 = 1.73205081;      // √3 used in kiwi kinematics equations
    const double POSITION_TOLERANCE = 0.15; // acceptable error in inches before stopping

    // Compute total straight-line distance to the target point
    double distance = sqrt(xTarget * xTarget + yTarget * yTarget);

    /*
      Normalize the desired motion vector.
      (ux, uy) is the unit direction the robot should move in.
     */
    double ux = xTarget / distance;
    double uy = yTarget / distance;

    /*
     * Convert the unit direction into commanded robot-frame velocity.
     * Vx = forward velocity component
     * Vy = sideways velocity component
     * omega = rotational velocity (not used here)
     */
    double Vx = speed * ux;
    double Vy = speed * uy;
    double omega = 0.0; // no rotation during this movement

    /*
     * Kiwi forward kinematics:
     * Convert robot motion (Vx, Vy, omega) into individual wheel commands.
     *
     * wheel1 -> right wheel
     * wheel2 -> left wheel
     * wheel3 -> front wheel
     */
    double wheel1 = (-1.0 * Vy + omega) * 100.0;               // bottom wheel
    double wheel2 = ( 0.8660254 * Vx + 0.5 * Vy + omega) * 100.0; // right wheel
    double wheel3 = (-0.8660254 * Vx + 0.5 * Vy + omega) * 100.0; // left wheel
    
    LCD.WriteLine(wheel1);
    LCD.WriteLine(wheel2);      
    LCD.WriteLine(wheel3);

    // Reset encoders
    right_encoder.ResetCounts(); // wheel1
    left_encoder.ResetCounts();  // wheel2
    front_encoder.ResetCounts(); // wheel3

    

    // Start motors
    rightdrive.SetPercent(-wheel3);
    leftdrive.SetPercent(-wheel1);
    frontdrive.SetPercent(-wheel2);

    /*
     * Main control loop:
     * Continuously estimate robot displacement from encoder readings
     * and stop when the robot reaches the target point.
     */
    while (true)
    {
        /*
         * Convert encoder counts into wheel travel distance (inches).
         * Each encoder is divided by its counts-per-inch calibration value.
         */
        double s1 = right_encoder.Counts() / R_ENCODE_P_IN; // right wheel travel
        double s2 = left_encoder.Counts()  / L_ENCODE_P_IN; // left wheel travel
        double s3 = front_encoder.Counts() / F_ENCODE_P_IN; // front wheel travel

        /*
         * Kiwi inverse kinematics:
         * Convert wheel travel distances into robot displacement.
         *
         * dx = forward displacement
         * dy = sideways displacement
         */
        double dx = (2.0 * s1 - s2 - s3) / 3.0;
        double dy = (s2 - s3) / SQRT3;

        /*
         * Compute the remaining error between the robot's current position
         * and the requested target position.
         */
        double ex = xTarget - dx;
        double ey = yTarget - dy;

        // Euclidean distance from the target point
        double error = sqrt(ex * ex + ey * ey);

        /*
         * Stop once the robot is close enough to the target.
         */
        if (error <= POSITION_TOLERANCE)
        {
            break;
        }

        // Small delay to prevent the loop from running excessively fast
        Sleep(0.005);
    }

    // Stop all motors once the target position has been reached
    StopAll();
}

void RotateDegrees(float angleDeg, float speed)
{
    // distance from robot center to wheel (inches)
    const double ROBOT_RADIUS = 4.05234;
    // Convert degrees to radians
    float theta = angleDeg * PI / 180.0;

    // Wheel travel distance required
    float wheelDistance = ROBOT_RADIUS * fabs(theta);

    // Convert to encoder counts
    float targetCounts = wheelDistance * R_ENCODE_P_IN;

    // Reset encoders
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    front_encoder.ResetCounts();

    // Determine direction
    float direction = (theta > 0) ? 1.0 : -1.0;

    // All wheels spin the same direction for pure rotation
    rightdrive.SetPercent(direction * -speed);
    leftdrive.SetPercent(direction * -speed);
    frontdrive.SetPercent(direction * -speed);

    while(abs(front_encoder.Counts()) < abs(targetCounts) && abs(right_encoder.Counts()) < abs(targetCounts) && abs(left_encoder.Counts()) < abs(targetCounts)){
        //Keep moving until counts reached
    }

    // Wait until wheels reach required rotation distance
    /*while (true)
    {
        double c1 = fabs(right_encoder.Counts());
        double c2 = fabs(left_encoder.Counts());
        double c3 = fabs(front_encoder.Counts());

        double avg = (c1 + c2 + c3) / 3.0;

        if (avg >= targetCounts)
            break;

        Sleep(0.005);
    }*/

    StopAll();
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

//Adam's attempt at allowing movement in ANY DIRECTION (VERY ROUGH TEST)
void DriveTEST(float Angle, float Speed, float Distance){
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
}

int CDS_CHECK(){
        LCD.Clear();
        LCD.WriteLine(CdS_cell.Value());
        Sleep(0.5);

    if(CdS_cell.Value() < 1.75){
        return 0; //Red
    }
        else if(CdS_cell.Value() > BLUE_LIGHT_MIN && CdS_cell.Value() < BLUE_LIGHT_MAX){
        return 1; //Blue
    }
    else{
        return 2; //Red
    }

}

void Milestone_2(){
    DriveTEST(180, 20.0, 1.0);

    DriveTEST(0, 20.0, 1.0);

    RotateDegrees(44.06, 25);

    DriveTEST(0, 25.0, 3.0);

    DriveTEST(90, 25.0, 1.0);

    DriveTEST(0, 50, 30.75);

    RotateDegrees((float) -80, 25);

    DriveTEST(0, 50, 12.65);

    //Detect Red v Blue
    int Light = 2;

    while(Light == 2){
        Light = CDS_CHECK();
    }

    if(Light == 0){ //Red
        DriveTEST(90, 20.0, 1.75);
        DriveTEST(0, 20.0, 3.0);
        Sleep(1.0);
        DriveTEST(180, 20.0, 3.0);

        return;
    }

    if(Light == 1){ //Blue
        DriveTEST(-90, 20.0, 1.75);
        DriveTEST(0, 20.0, 3.0);
        Sleep(1.0);
        DriveTEST(180, 20.0, 3.0);

        return;
    }
}

void ERCMain()
{
    while(CdS_cell.Value() > START_LIGHT){
        LCD.Clear();
        LCD.WriteLine(CdS_cell.Value());
    }


    Milestone_2();

    while(1){
        LCD.Clear();
        LCD.WriteLine(CdS_cell.Value());
        Sleep(0.5);
    }
    /*Drive(FORWARD, 0.30, 3);

    LCD.WriteLine("Done!");

    LCD.WriteLine(right_encoder.Counts());
    LCD.WriteLine(left_encoder.Counts());
    LCD.WriteLine(front_encoder.Counts());


    while(!LCD.Touch(&x, &y));

    LCD.Clear();

    Drive(RIGHT, 0.30, 3);
    */
 
}