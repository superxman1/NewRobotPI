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

// Drivetrain PID tuning constants.
// Inner loop: wheel velocity PID.
// Outer loop: robot-level position gain -> desired speed magnitude.
#define DRIVE_PID_KP 0.75
#define DRIVE_PID_KI 0.05
#define DRIVE_PID_KD 0.25
#define DRIVE_PID_ERROR_SUM_LIMIT 50.0
#define DRIVE_PID_MIN_PERCENT 10.0
#define DRIVE_PID_LOOP_DT 0.1
#define DRIVE_MAX_COUNTS_PER_SEC_100 120.0
#define DRIVE_POS_KP 0.35
#define DRIVE_POSITION_TOL_COUNTS 2.0
#define DRIVE_POSITION_MIN_PERCENT 12.0

// Rotation command defaults for Turn_PID().
#define TURN_PID_SPEED_PERCENT 20.0
#define TURN_PID_TARGET_COUNTS 140.0

 

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

 

void Drive(Direction dir, int8_t speed, double distance); //takes input direction (see diagram), speed (in percent), and distance (inches) 

void StopAll(); //stops the motion of all motors 

void Stop(FEHMotor motor); //stops the motion of a specific motor 

 

void Turn_Right(); 

void Turn_Left(); 
void Turn_PID(bool ccw_turn);

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

struct VelocityPIDState {
    // Running integral of error (I term memory).
    double error_sum;
    // Previous error and sample data for derivative and velocity estimates.
    double previous_error;
    double previous_counts;
    double previous_time;
    // Last commanded power; PID returns oldPower + P + I + D.
    double motor_power;
};

double ClampDouble(double value, double min_value, double max_value){
    if(value < min_value){
        return min_value;
    }
    if(value > max_value){
        return max_value;
    }
    return value;
}

void ResetVelocityPIDState(VelocityPIDState &state, double initial_power, double current_counts, double current_time){
    // Reset all state each time a new motion begins.
    state.error_sum = 0.0;
    state.previous_error = 0.0;
    state.previous_counts = current_counts;
    state.previous_time = current_time;
    state.motor_power = initial_power;
}

double VelocityPIDAdjustment(
    double expected_percent,
    double current_counts,
    double current_time,
    double speed_limit,
    VelocityPIDState &state
){
    // 1) Measure dt and encoder delta since last PID update.
    double delta_time = current_time - state.previous_time;
    if(delta_time < 0.001){
        delta_time = DRIVE_PID_LOOP_DT;
    }

    double delta_counts = current_counts - state.previous_counts;

    // 2) Convert measured wheel speed to a percent-equivalent speed estimate.
    double actual_counts_per_sec = delta_counts / delta_time;
    double actual_percent = 0.0;
    if(DRIVE_MAX_COUNTS_PER_SEC_100 > 0.0){
        actual_percent = (actual_counts_per_sec / DRIVE_MAX_COUNTS_PER_SEC_100) * 100.0;
    }

    // 3) PID error terms.
    double error = expected_percent - actual_percent;
    state.error_sum += error * delta_time;
    state.error_sum = ClampDouble(
        state.error_sum,
        -DRIVE_PID_ERROR_SUM_LIMIT,
        DRIVE_PID_ERROR_SUM_LIMIT
    );

    double p_term = error * DRIVE_PID_KP;
    double i_term = state.error_sum * DRIVE_PID_KI;
    double d_term = ((error - state.previous_error) / delta_time) * DRIVE_PID_KD;

    // 4) Instructor-style update: newPower = oldPower + P + I + D.
    state.motor_power += p_term + i_term + d_term;
    state.motor_power = ClampDouble(state.motor_power, 0.0, speed_limit);

    // Prevent tiny commands that may stall a wheel while still "moving".
    if(expected_percent <= 0.001){
        state.motor_power = 0.0;
    } else if(state.motor_power < DRIVE_PID_MIN_PERCENT){
        state.motor_power = fmin(DRIVE_PID_MIN_PERCENT, speed_limit);
    }

    // 5) Save historical values for the next loop update.
    state.previous_error = error;
    state.previous_counts = current_counts;
    state.previous_time = current_time;

    return state.motor_power;
}

void ResetDrivePIDVariables(
    VelocityPIDState &pid1,
    VelocityPIDState &pid2,
    VelocityPIDState &pid3,
    double initial_power1,
    double initial_power2,
    double initial_power3
){
    // Pseudocode "ResetPIDVariables": clear encoders + PID memory + timestamp.
    left_encoder.ResetCounts();
    right_encoder.ResetCounts();
    front_encoder.ResetCounts();

    double now = TimeNow();
    ResetVelocityPIDState(pid1, initial_power1, 0.0, now);
    ResetVelocityPIDState(pid2, initial_power2, 0.0, now);
    ResetVelocityPIDState(pid3, initial_power3, 0.0, now);

    // Avoid tiny first dt values which destabilize velocity estimates.
    Sleep(DRIVE_PID_LOOP_DT);
}

void Drive(Direction dir, int8_t speed, double distance){ 

    // Robot motion commands
    double Vx = 0.0;    // forward (+forward, -back)
    double Vy = 0.0;    // left (+left, -right)
    double omega = 0.0; // rotation (+CCW, -CW)

    switch (dir)
    {
    case FORWARD:
        Vx = (double)speed;
        break;
    case REVERSE:
        Vx = (double)(-speed);
        break;
    case LEFT:
        Vy = (double)speed;
        break;
    case RIGHT:
        Vy = (double)(-speed);
        break;
    case LEFT_F:
        Vx = (double)speed;
        Vy = (double)speed;
        break;
    case LEFT_R:
        Vx = (double)(-speed);
        Vy = (double)speed;
        break;
    case RIGHT_F:
        Vx = (double)speed;
        Vy = (double)(-speed);
        break;
    case RIGHT_R:
        Vx = (double)(-speed);
        Vy = (double)(-speed);
        break;
    default:
        LCD.WriteLine("Direction not specified during drive function");
        return;
    }

    double speed_limit = fmin(100.0, fabs((double)speed));
    if(speed_limit <= 0.0 || distance <= 0.0){
        StopAll();
        return;
    }

    // Kiwi drive wheel speeds
    double wheel1 = Vx + omega;
    double wheel2 = -0.5 * Vx + 0.8660254 * Vy + omega;
    double wheel3 = -0.5 * Vx - 0.8660254 * Vy + omega;

    // Keep wheel output within requested speed while preserving direction ratios.
    double maxMag = fmax(fabs(wheel1), fmax(fabs(wheel2), fabs(wheel3)));
    if(maxMag > speed_limit){
        double scale = speed_limit / maxMag;
        wheel1 *= scale;
        wheel2 *= scale;
        wheel3 *= scale;
    }

    double wheel1_dir = (fabs(wheel1) > 0.001) ? ((wheel1 > 0.0) ? 1.0 : -1.0) : 0.0;
    double wheel2_dir = (fabs(wheel2) > 0.001) ? ((wheel2 > 0.0) ? 1.0 : -1.0) : 0.0;
    double wheel3_dir = (fabs(wheel3) > 0.001) ? ((wheel3 > 0.0) ? 1.0 : -1.0) : 0.0;

    double wheel1_mag = fabs(wheel1);
    double wheel2_mag = fabs(wheel2);
    double wheel3_mag = fabs(wheel3);
    double commanded_max_mag = fmax(wheel1_mag, fmax(wheel2_mag, wheel3_mag));
    if(commanded_max_mag <= 0.001){
        StopAll();
        return;
    }

    // Preserve kiwi-drive wheel magnitude ratios for distributing desired speed.
    double ratio1 = wheel1_mag / commanded_max_mag;
    double ratio2 = wheel2_mag / commanded_max_mag;
    double ratio3 = wheel3_mag / commanded_max_mag;

    // Single robot-level position target in encoder counts.
    double target_avg_counts = distance * ((L_ENCODE_P_IN + R_ENCODE_P_IN + F_ENCODE_P_IN) / 3.0);

    VelocityPIDState pid1;
    VelocityPIDState pid2;
    VelocityPIDState pid3;
    ResetDrivePIDVariables(
        pid1, pid2, pid3,
        speed_limit * ratio1, speed_limit * ratio2, speed_limit * ratio3
    );

    // Outer position controller + inner wheel velocity PID.
    while(true){
        double count1 = (double)right_encoder.Counts();
        double count2 = (double)left_encoder.Counts();
        double count3 = (double)front_encoder.Counts();

        // Outer loop: use robot-level average position error only.
        double avg_counts = (count1 + count2 + count3) / 3.0;
        double remaining_avg_counts = fmax(0.0, target_avg_counts - avg_counts);
        if(remaining_avg_counts <= DRIVE_POSITION_TOL_COUNTS){
            break;
        }

        // True outer position-gain controller: remaining position -> desired speed.
        double desired_speed = DRIVE_POS_KP * remaining_avg_counts;
        desired_speed = ClampDouble(desired_speed, 0.0, speed_limit);
        if(desired_speed < DRIVE_POSITION_MIN_PERCENT){
            desired_speed = fmin(DRIVE_POSITION_MIN_PERCENT, speed_limit);
        }

        // Preserve kiwi ratio distribution; signs are applied separately below.
        double expected1_percent = desired_speed * ratio1;
        double expected2_percent = desired_speed * ratio2;
        double expected3_percent = desired_speed * ratio3;

        // Inner loop: per-wheel velocity PID tracking expected wheel speeds.
        double now = TimeNow();
        double out1 = VelocityPIDAdjustment(expected1_percent, count1, now, speed_limit, pid1);
        double out2 = VelocityPIDAdjustment(expected2_percent, count2, now, speed_limit, pid2);
        double out3 = VelocityPIDAdjustment(expected3_percent, count3, now, speed_limit, pid3);

        rightdrive.SetPercent(wheel1_dir * out1);
        leftdrive.SetPercent(wheel2_dir * out2);
        frontdrive.SetPercent(wheel3_dir * out3);

        Sleep(DRIVE_PID_LOOP_DT);
    }

    StopAll();
} 

void Turn_PID(bool ccw_turn){
    // Pure kiwi rotation: all 3 wheels run the same magnitude, same sign.
    double omega_percent = ccw_turn ? TURN_PID_SPEED_PERCENT : -TURN_PID_SPEED_PERCENT;
    double speed_limit = fabs(omega_percent);
    if(speed_limit <= 0.0){
        StopAll();
        return;
    }

    double wheel1 = omega_percent;
    double wheel2 = omega_percent;
    double wheel3 = omega_percent;

    double wheel1_dir = (wheel1 >= 0.0) ? 1.0 : -1.0;
    double wheel2_dir = (wheel2 >= 0.0) ? 1.0 : -1.0;
    double wheel3_dir = (wheel3 >= 0.0) ? 1.0 : -1.0;

    double expected1_percent = fabs(wheel1);
    double expected2_percent = fabs(wheel2);
    double expected3_percent = fabs(wheel3);

    VelocityPIDState pid1;
    VelocityPIDState pid2;
    VelocityPIDState pid3;

    ResetDrivePIDVariables(pid1, pid2, pid3, expected1_percent, expected2_percent, expected3_percent);

    while(((left_encoder.Counts() + right_encoder.Counts() + front_encoder.Counts()) / 3.0) < TURN_PID_TARGET_COUNTS){
        double now = TimeNow();
        double count1 = (double)right_encoder.Counts();
        double count2 = (double)left_encoder.Counts();
        double count3 = (double)front_encoder.Counts();

        double out1 = VelocityPIDAdjustment(expected1_percent, count1, now, speed_limit, pid1);
        double out2 = VelocityPIDAdjustment(expected2_percent, count2, now, speed_limit, pid2);
        double out3 = VelocityPIDAdjustment(expected3_percent, count3, now, speed_limit, pid3);

        rightdrive.SetPercent(wheel1_dir * out1);
        leftdrive.SetPercent(wheel2_dir * out2);
        frontdrive.SetPercent(wheel3_dir * out3);

        Sleep(DRIVE_PID_LOOP_DT);
    }

    StopAll();
}

void Turn_Left(){ 

    Turn_PID(true);
    return;

} 

 

void Turn_Right(){ 

    Turn_PID(false);
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

