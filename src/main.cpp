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
#define BASECOUNT 39
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

//Global Heading Variable
double g_heading_deg = 90.0;

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

//Funciton prototypes

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

 
//Heading normilzation function 
double NormalizeAngleDeg(double angle){
    while (angle >= 360.0) angle -= 360.0;
    while (angle < 0.0)    angle += 360.0;
    return angle;
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


void DriveXY(double xTarget, double yTarget, int speed)
{
    //Sets an error tolerance for final position of robot after movement
    const double X_TOLERANCE = 0.20;
    const double Y_TOLERANCE = 0.20;

    //Sets distance from target before starting to slowdown
    //Sets a minimum percent of inputted speed
    const double SLOWDOWN_RADIUS = 3.0;
    const double MIN_SLOW_SPEED_SCALE = 0.35;


    //Sets an initial speed (50% of input)
    //Sets time to reach inputted speed
    const double RAMP_UP_TIME = 0.20;
    const double START_SPEED_SCALE = 0.5;

    //Sets a time before timing out
    const double MAX_DRIVE_TIME = 5.0;

    //Calculates distance to target given x and y coordinates
    double distance = sqrt(pow(xTarget, 2) + pow(yTarget, 2));

    //If the input distance is less than 0.05 inches, don't move and return to main();
    if (distance < 0.05)
    {
        StopAll();
        return;
    }

    //Reset encoder counts before the movement begins.
    left_encoder.ResetCounts();    // Wheel1 = back
    right_encoder.ResetCounts();   // Wheel2 = right
    front_encoder.ResetCounts();   // Wheel3 = left

    double omega = 0.0;

    //Setting xRemaining and yRemaining for initial condition check
    double xRemaining = xTarget;
    double yRemaining = yTarget;
    double elapsed = 0;
    
    double dx, dy;
    double error;

    double dtheta, current_heading, heading_error;
    double desired_heading = atan2(yTarget, xTarget) * (180.0 / PI);

    //Starting timer for timeout
    double startTime = TimeNow();

    //While ex and ey are not within tolerance ranges, and max drive time has not been reached, the loop will continue
    while (!(fabs(xRemaining) <= X_TOLERANCE && fabs(yRemaining) <= Y_TOLERANCE) && !(elapsed > MAX_DRIVE_TIME))
    {
        //Signed wheel travel in inches
        double s1 = (fabs(left_encoder.Counts())  / R_ENCODE_P_IN);
        double s2 = (fabs(right_encoder.Counts()) / F_ENCODE_P_IN);
        double s3 = (fabs(front_encoder.Counts()) / L_ENCODE_P_IN);

        //Creates a unit normal vector for desired direction
        double baseDistance = sqrt(xTarget * xTarget + yTarget * yTarget);
        double baseVx = -(xTarget / baseDistance);
        double baseVy = (yTarget / baseDistance);

        //Sets base power for each wheel using unit vector
        double baseWheel1 = -(baseVy + omega);
        double baseWheel2 = ((SIN60 * baseVx) + (COS60 * baseVy) + omega);
        double baseWheel3 = ((-SIN60 * baseVx) + (COS60 * baseVy) + omega);

        /*Defaulting signs to be positive, then checking baseWheel. If baseWheel is negative,
        it will change the respective sign to be negative, if not it will remian positive. Additionally
        changed W*_SIGN to be integers to save memory.*/
        int W1_SIGN = 1;
        int W2_SIGN = 1;
        int W3_SIGN = 1;

        if (baseWheel1 <= 0.0){
            W1_SIGN = -1;
        }
        if (baseWheel2 <= 0.0){
            W2_SIGN = -1;
        }
        if (baseWheel3 <= 0.0){
            W3_SIGN = -1;
        }

        //Gives correct sign to each wheel
        //May add this into the if statements above to save on lines and memory
        s1 *= W1_SIGN;
        s2 *= W2_SIGN;
        s3 *= W3_SIGN;

        //Calculating an estimate of current x and y positions
        dx = -(s2 - s3) / SQRT3;
        dy = -(2.0 * s1 - s2 - s3) / 3.0;

        //Correcting for changes in orientation
        dtheta = (s1 + s2 + s3) / (3.0 * RADIUS);
        static double current_heading = 0.0;
        current_heading += dtheta * (180.0 / PI);
        heading_error = desired_heading - current_heading;

        if (heading_error > 180){
            heading_error -= 360;
        }
        if (heading_error < -180){
            heading_error += 360;
        }
        const double kP_heading = 0.5; //NEEDS ADJUSTED
        omega = kP_heading * heading_error;

        if (omega > 20) omega = 20;
        if (omega < -20) omega = -20;

        //Calculating distance remaining in x and y directions
        xRemaining = xTarget - dx;
        yRemaining = yTarget - dy;
        error = sqrt(pow(xRemaining, 2) + pow(yRemaining, 2));

        //Calculating time elapsed to check for slow down and ramp up conditions
        elapsed = TimeNow() - startTime;

        //Ramp up conditions
        double rampScale = 1.0;
        if (elapsed < RAMP_UP_TIME)
        {
            rampScale = START_SPEED_SCALE + (1.0 - START_SPEED_SCALE) * (elapsed / RAMP_UP_TIME);
        }

        //Slowdown conditions
        double slowScale = 1.0;
        if (error < SLOWDOWN_RADIUS)
        {
            slowScale = error / SLOWDOWN_RADIUS;
            if (slowScale < MIN_SLOW_SPEED_SCALE)
            {
                slowScale = MIN_SLOW_SPEED_SCALE;
            }
        }

        //Determines whether the robot needs to slow down or speed up
        //Default to slowScale, if condition is met, set to rampScale
        double speedScale = slowScale;
        if(rampScale < slowScale){
            speedScale = rampScale;
        }

        //Sets current speed based on above statement
        double currentSpeed = speed * speedScale;
        
        //Creates a new unit vector to update the direction based on error
        double xCorrection;
        double yCorrection;
        if (error != 0)
        {
            xCorrection = xRemaining / error;
            yCorrection = yRemaining / error;
        }

        //Calculates updated x and y speeds based on new unit vectors
        double Vx = -currentSpeed * xCorrection;
        double Vy =  currentSpeed * yCorrection;

        //Calculates wheel speeds based on updated updated x and y speeds
        double wheel1 = (-1.0 * Vy + omega);
        double wheel2 = ((SIN60 * Vx) + (COS60 * Vy) + omega);
        double wheel3 = ((-SIN60 * Vx) + (COS60 * Vy) + omega);

        //Sets wheel speeds
        leftdrive.SetPercent(-wheel1);
        rightdrive.SetPercent(-wheel2);
        frontdrive.SetPercent(-wheel3);

        //Sleeps between loops
        Sleep(0.005);
        
        //Calculating time elapsed to check for slow down and ramp up conditions
        elapsed = TimeNow() - startTime;
    }

    //Stops all motors
    StopAll();

    //Prints all data from DriveXY to scree
    LCD.Clear();
    LCD.WriteLine("DriveXY Data");
    LCD.Write("dx: "); LCD.WriteLine(dx);
    LCD.Write("dy: "); LCD.WriteLine(dy);
    LCD.Write("xRemaining: "); LCD.WriteLine(xRemaining);
    LCD.Write("yRemaining: "); LCD.WriteLine(yRemaining);
    LCD.Write("err: "); LCD.WriteLine(error);
    LCD.Write("rencoder: "); LCD.WriteLine(right_encoder.Counts());
    LCD.Write("lencoder: "); LCD.WriteLine(left_encoder.Counts());
    LCD.Write("fencoder: "); LCD.WriteLine(front_encoder.Counts());
}

void DriveFieldXY(double fieldDx, double fieldDy, double speed)
{
    double h = g_heading_deg * PI / 180.0;

    // Convert desired field movement into robot-frame movement
    double forwardTarget = fieldDx * cos(h) + fieldDy * sin(h);
    double rightTarget   = fieldDx * sin(h) - fieldDy * cos(h);

    DriveXY(forwardTarget, rightTarget, speed);
}

void RotateDegrees(float angleDeg, float speed)
{
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
    right_encoder.ResetCounts();
    left_encoder.ResetCounts();
    front_encoder.ResetCounts();

    // Determine direction
    float direction = (theta > 0) ? 1.0f : -1.0f;

    while (true)
    {
        float c1 = fabs(right_encoder.Counts());
        float c2 = fabs(left_encoder.Counts());
        float c3 = fabs(front_encoder.Counts());

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

        rightdrive.SetPercent(direction * -currentSpeed);
        leftdrive.SetPercent(direction * -currentSpeed);
        frontdrive.SetPercent(direction * -currentSpeed);

        Sleep(0.005);
    }

    StopAll();

    // Assuming positive RotateDegrees = CCW in field frame
    g_heading_deg = NormalizeAngleDeg(g_heading_deg + angleDeg);
    NormalizeAngleDeg(g_heading_deg); // Ensure heading stays within 0-360 range
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

void DriveRightTime(float speed, float time)
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
}

void WaitForTouch()
{
    int x, y;

    while (!LCD.Touch(&x, &y)) {}    // wait for press

    return;
}

void BacktoWall(float time){
    leftdrive.SetPercent(-50);
    frontdrive.SetPercent(50);

    Sleep(time);

    StopAll();
    return;
}

void FronttoWall(float time){
    leftdrive.SetPercent(50);
    frontdrive.SetPercent(-50);

    Sleep(time);

    StopAll();
    return;
}

void Ramp(float time){
    rightdrive.SetPercent(75);
    frontdrive.SetPercent(-75);

    Sleep(time);

    StopAll();

    return;
}

void Milestone_3(){
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
}
 
void Milestone_4(){

}

void lever() {
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

}
void ERCMain()
{
    RCS.InitializeTouchMenu("1130D6KKR");
    Milestone_3();
    
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
