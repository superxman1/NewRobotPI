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
#define RED_LIGHT 2.0
#define BLUE_LIGHT_MIN 2.0
#define BLUE_LIGHT_MAX 2.6

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
        Vx = -speed;
        break;

    case REVERSE:
        ux = -1.0; uy = 0.0;
        Vx = speed;
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
        Vx = -speed * INV_SQRT2;
        Vy = -speed * INV_SQRT2;
        break;

    case LEFT_R:
        ux = -INV_SQRT2; uy = -INV_SQRT2;
        Vx = speed * INV_SQRT2;
        Vy = -speed * INV_SQRT2;
        break;

    case RIGHT_F:
        ux = INV_SQRT2;  uy = INV_SQRT2;
        Vx = -speed * INV_SQRT2;
        Vy = speed * INV_SQRT2;
        break;

    case RIGHT_R:
        ux = -INV_SQRT2; uy = INV_SQRT2;
        Vx = speed * INV_SQRT2;
        Vy = speed * INV_SQRT2;
        break;

    default:
        LCD.WriteLine("Direction not specified during drive function");
        return;
    }

    // Kiwi drive forward kinematics
    // wheel1 = bottom
    // wheel2 = right
    // wheel3 = left
    double wheel1 = (-1.0 * Vy + omega) * 100.0;
    double wheel2 = ( 0.8660254 * Vx + 0.5 * Vy + omega) * 100.0;
    double wheel3 = (-0.8660254 * Vx + 0.5 * Vy + omega) * 100.0;

    // Reset encoders
    left_encoder.ResetCounts();    // wheel1 = bottom
    right_encoder.ResetCounts();   // wheel2 = right
    front_encoder.ResetCounts();   // wheel3 = left

    // Start motors using REAL physical mapping
    leftdrive.SetPercent(-wheel1);    // bottom wheel
    rightdrive.SetPercent(-wheel2);   // right wheel
    frontdrive.SetPercent(-wheel3);   // left wheel

    while (true)
    {
        // Encoder sign calibration constants
        const double S1_SIGN = -1.0; // bottom wheel
        const double S2_SIGN =  1.0; // right wheel
        const double S3_SIGN =  1.0; // left wheel

        double s1 = S1_SIGN * left_encoder.Counts()  / R_ENCODE_P_IN;
        double s2 = S2_SIGN * right_encoder.Counts() / F_ENCODE_P_IN;
        double s3 = S3_SIGN * front_encoder.Counts() / L_ENCODE_P_IN;

        // True inverse kiwi kinematics
        double dx = (s2 + s3) / SQRT3;
        double dy = ((-2.0 * s1) + s2 + s3) / 3.0;

        // Progress along commanded direction
        double px = dx * ux;
        double py = dy * uy;
        double progress = px + py;
        if (fabs(progress) >= distance)
        {
            StopAll();
            LCD.Clear();
            LCD.Write("s1: "); LCD.WriteLine(s1);
            LCD.Write("s2: "); LCD.WriteLine(s2);
            LCD.Write("s3: "); LCD.WriteLine(s3);
            LCD.Write("dx: "); LCD.WriteLine(dx);
            LCD.Write("dy: "); LCD.WriteLine(dy);
            LCD.Write("px: "); LCD.WriteLine(px);
            LCD.Write("py: "); LCD.WriteLine(py);
            LCD.Write("p: ");  LCD.WriteLine(progress);
            LCD.Write("rencoder: "); LCD.WriteLine(right_encoder.Counts());
            LCD.Write("lencoder: "); LCD.WriteLine(left_encoder.Counts());
            LCD.Write("fencoder: "); LCD.WriteLine(front_encoder.Counts());
            break;
        }

        Sleep(0.005);
    }
}


void DriveXY(double xTarget, double yTarget, double speed)
{
    const double SQRT3 = 1.73205081;

    const double X_TOLERANCE = 0.10;
    const double Y_TOLERANCE = 0.10;

    const double SLOWDOWN_RADIUS = 3.0;
    const double MIN_SLOW_SPEED_SCALE = 0.35;

    const double RAMP_UP_TIME = 0.20;
    const double START_SPEED_SCALE = 0.5;

    const double MAX_DRIVE_TIME = 5.0;

    double distance = sqrt(xTarget * xTarget + yTarget * yTarget);

    if (distance < 0.001)
    {
        StopAll();
        return;
    }

    left_encoder.ResetCounts();    // wheel1 = bottom
    right_encoder.ResetCounts();   // wheel2 = right
    front_encoder.ResetCounts();   // wheel3 = left

    double startTime = TimeNow();
    double omega = 0.0;

    while (true)
    {
        // Signed wheel travel in inches
        double s1 = (fabs(left_encoder.Counts())  / R_ENCODE_P_IN);
        double s2 = (fabs(right_encoder.Counts()) / F_ENCODE_P_IN);
        double s3 = (fabs(front_encoder.Counts()) / L_ENCODE_P_IN);

        // Recover signs from actual commanded wheel directions would be better,
        // but keeping your current convention structure:
        // If your current sign convention already works for pure-axis tests,
        // keep using the signed version from your working code instead.

        // Use your original signed wheel travel version:
        double baseDistance = sqrt(xTarget * xTarget + yTarget * yTarget);
        double ux0 = xTarget / baseDistance;
        double uy0 = yTarget / baseDistance;

        double baseVx = -ux0;
        double baseVy =  uy0;

        double baseWheel1 = (-1.0 * baseVy + omega);
        double baseWheel2 = ( 0.8660254 * baseVx + 0.5 * baseVy + omega);
        double baseWheel3 = (-0.8660254 * baseVx + 0.5 * baseVy + omega);

        double W1_SIGN = (baseWheel1 >= 0.0) ? 1.0 : -1.0;
        double W2_SIGN = (baseWheel2 >= 0.0) ? 1.0 : -1.0;
        double W3_SIGN = (baseWheel3 >= 0.0) ? 1.0 : -1.0;

        s1 *= W1_SIGN;
        s2 *= W2_SIGN;
        s3 *= W3_SIGN;

        // Position estimate
        double dx = -(s2 - s3) / SQRT3;
        double dy = -(2.0 * s1 - s2 - s3) / 3.0;

        // Current error
        double ex = xTarget - dx;
        double ey = yTarget - dy;
        double error = sqrt(ex * ex + ey * ey);

        double elapsed = TimeNow() - startTime;

        // Stop only when actual x/y target is reached, or timeout
        if ((fabs(ex) <= X_TOLERANCE && fabs(ey) <= Y_TOLERANCE) ||
            (elapsed > MAX_DRIVE_TIME))
        {
            StopAll();
            LCD.Clear();
            LCD.WriteLine("DriveXY done");
            LCD.Write("dx: "); LCD.WriteLine(dx);
            LCD.Write("dy: "); LCD.WriteLine(dy);
            LCD.Write("ex: "); LCD.WriteLine(ex);
            LCD.Write("ey: "); LCD.WriteLine(ey);
            LCD.Write("err: "); LCD.WriteLine(error);
            break;
        }

        // Ramp-up
        double rampScale = 1.0;
        if (elapsed < RAMP_UP_TIME)
        {
            rampScale = START_SPEED_SCALE +
                        (1.0 - START_SPEED_SCALE) * (elapsed / RAMP_UP_TIME);
        }

        // Slowdown near target
        double slowScale = 1.0;
        if (error < SLOWDOWN_RADIUS)
        {
            slowScale = error / SLOWDOWN_RADIUS;
            if (slowScale < MIN_SLOW_SPEED_SCALE)
            {
                slowScale = MIN_SLOW_SPEED_SCALE;
            }
        }

        double speedScale = (rampScale < slowScale) ? rampScale : slowScale;
        double currentSpeed = speed * speedScale;

        // RE-AIM toward the remaining error, not the original path
        double ux_cmd = 0.0;
        double uy_cmd = 0.0;

        if (error > 0.0001)
        {
            ux_cmd = ex / error;
            uy_cmd = ey / error;
        }

        double Vx = -currentSpeed * ux_cmd;
        double Vy =  currentSpeed * uy_cmd;

        double wheel1 = (-1.0 * Vy + omega);
        double wheel2 = ( 0.8660254 * Vx + 0.5 * Vy + omega);
        double wheel3 = (-0.8660254 * Vx + 0.5 * Vy + omega);

        leftdrive.SetPercent(-wheel1);
        rightdrive.SetPercent(-wheel2);
        frontdrive.SetPercent(-wheel3);

        Sleep(0.005);
    }
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

// Adam's attempt at allowing movement in ANY DIRECTION (VERY ROUGH TEST)
int CDS_CHECK();

bool DriveTEST_Light(float Angle, float Speed, float Distance){
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
}



void Milestone_2(){
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
        StopAll(); */
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
        StopAll(); */
        Sleep(1.0);  
        DriveTEST(180, 20.0, 5.0);
        DriveTEST(90, 20.0, .8);
    }

    DriveTEST(180, 20.0, 17.0);
    
    RotateDegrees(-95, 25);

    DriveTEST(0, 50.0, 45.0);

    DriveTEST(180, 20.0, 2.0);

    DriveTEST(-90, 50.0, 10.0);
}

 void WaitForTouch()
{
    int x, y;

    while (LCD.Touch(&x, &y)) {}     // wait for release
    while (!LCD.Touch(&x, &y)) {}    // wait for press
    while (LCD.Touch(&x, &y)) {}     // wait for release
}

void ERCMain()
{
    int x, y;
    
    /*
    while(!LCD.Touch(&x, &y));
    RotateDegrees(90, 25);
    while(!LCD.Touch(&x, &y));
    RotateDegrees(-90, 25);
    while(!LCD.Touch(&x, &y));
    RotateDegrees(45, 25);
    while(!LCD.Touch(&x, &y));
    RotateDegrees(90, 25);
    while(!LCD.Touch(&x, &y));
    RotateDegrees(180, 50);
    while(!LCD.Touch(&x, &y));
    RotateDegrees(45, 25);
    */

 
    while(!LCD.Touch(&x, &y));
    DriveXY(7.5,0,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(0,7.5,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(-7.5,0,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(0,-7.5,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(10,5,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(-3,8,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(-1,8,75);
    while(!LCD.Touch(&x, &y));
    DriveXY(8,-2,75);

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