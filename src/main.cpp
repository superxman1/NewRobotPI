#include <FEH.h> 

#include <Arduino.h> 

 

//Hello 

// Declare things like Motors, Servos, etc. here 

// For example: 

// FEHMotor leftMotor(FEHMotor::Motor0, 6.0); 

// FEHServo servo(FEHServo::Servo0); 

 

//after testing we will change these values to their correct encodings per inch, but just placeholders for now 

#define R_ENCODE_P_IN 1 

#define L_ENCODE_P_IN 1 

#define F_ENCODE_P_IN 1 

 

//each of the 6 directions possible from any one point (see diagram for greater detail) 

enum Direction{ 

    FORWARD, 

    REVERSE, 

    LEFT_F, 

    LEFT_R, 

    RIGHT_F, 

    RIGHT_R 

}; 

 

//declare motor variables 

FEHMotor rightdrive(FEHMotor::Motor1,9.0); 

FEHMotor leftdrive(FEHMotor::Motor0,9.0); 

FEHMotor frontdrive(FEHMotor::Motor0,9.0); 

 

//Declaring Digital Encoders 

DigitalEncoder left_encoder(FEHIO::Pin9); 

DigitalEncoder right_encoder(FEHIO::Pin8); 

DigitalEncoder front_encoder(FEHIO::Pin8); 

 

void Drive(Direction dir, int8_t speed, float distance); //takes input direction (see diagram), speed (in percent), and distance (inches) 

void StopAll(); //stops the motion of all motors 

void Stop(FEHMotor motor); //stops the motion of a specific motor 

 

void Turn_Right(); 

void Turn_Left(); 

//void Course(); 

 

void Drive(Direction dir, int8_t speed, float distance){ 

    //will eventually need a correction factor for momentum 

    switch (dir) 

    { 

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