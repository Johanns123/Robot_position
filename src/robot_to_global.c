#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <windows.h> 
// #include <unistd.h> //for linux is  unistd

#define DEG_TO_RAD M_PI/180
#define RAD_TO_DEG 180/M_PI
#define LENGTH 0.146 //meters
#define RADIUS 0.040 //meters
#define LIMIT_LINEAR_PRECISION_RATE 0.001
#define LIMIT_ANGULAR_PRECISION_RATE 0.001
#define LIMIT_ANGULAR_ROBOT_SPEED 18.367123
#define LIMIT_LINEAR_ROBOT_SPEED 1.34
#define LIMIT_ANGULAR_MOTOR_SPEED 33.52
#define LIMIT_OUTPUT_VOLTAGE 12

typedef struct
{
    float x;
    float y;
    float theta;
    float theta_1;
} coordinate_global;

typedef struct
{
    float x;
    float y;
    float theta;
    float theta_1;
    float delta_theta;
} coordinate_local;

typedef struct
{
    float uk_ang;
    float uk_ang1;
    float uk_disp;
    float uk_disp1;
} external_controller;


void calculate_coordinates (coordinate_global *global, coordinate_local *local, float distance);
external_controller control_system(uint8_t mode, float ang_setpoint, float PV_ang_setpoint, float lin_setpoint, float PV_lin_setpoint);
void control_motors(float w_refR, float w_refL, float *yk_MR, float *yk_ML);

float T = 50e-3;

int main(void)
{
    
    static uint16_t i = 0;
    static float v = 0, w_robot =  0, DS; //meters
    static float WR_ref = 0, WL_ref = 0, VR = 0, VL = 0, yk_MR, yk_ML;
    // static float voltsR = 0, voltsL = 0; //for opne loop
    float theta_desired = 0, desired_displacement = 0;
    coordinate_global global;
    coordinate_local robot;
    external_controller ext_ctrl;
    global.theta_1 = 0, global.theta = 0;
    robot.theta_1 = 0, robot.theta = 0;
    // unsigned int v = 0;
    float X_desired = 0, Y_desired = 0;

    FILE *fp0 = fopen("angular.txt", "w" );
    if(fp0 == NULL)
    {
        printf("Error opening file!\n");
        return 0;
    }
    FILE *fp1 = fopen("WrefR.txt", "w" );
    if(fp1 == NULL)
    {
        printf("Error opening file!\n");
        return 0;
    }
    FILE *fp2 = fopen("WrefL.txt", "w" );
    if(fp2 == NULL)
    {
        printf("Error opening file!\n");
        return 0;
    }
    FILE *fp3 = fopen("displacement.txt", "w" );
    if(fp3 == NULL)
    {
        printf("Error opening file!\n");
        return 0;
    }

    printf("Enter the X desired coordinate >> ");
    scanf("%f", &X_desired);
    printf("Enter the y desired coordinate >> ");
    scanf("%f", &Y_desired);

    desired_displacement = sqrt((X_desired*X_desired)+(Y_desired*Y_desired));
    theta_desired = atan2(Y_desired, X_desired);
    // theta_desired = 3.14;
    printf("%f\n", theta_desired);
    // usleep(1000000);
    Sleep(1000);

    while(1)
    {
        //calculate the actual global theta
        calculate_coordinates(&global, &robot, DS);

        printf("(%3.2f) Xl:%7.4f Xg:%7.4f Yl:%7.4f Yg:%7.4f ang:%7.4f dist:%7.4f\n",float(i*T), robot.x, global.x, robot.y, global.y, global.theta, DS);
        if(i == 200) //close the file with 200 samples
        {
            fclose(fp0);
            fclose(fp1);
            fclose(fp2);
            fclose(fp3);
            // break;
        }

        else
        {
            fprintf(fp0, "%7.4f\n", global.theta);
            fprintf(fp1, "%7.4f\n", yk_MR);
            fprintf(fp2, "%7.4f\n", yk_ML);
            fprintf(fp3, "%7.4f\n", DS);
        }
        i++;

        //calculating displacement and robot theta
        v       = (RADIUS * ((yk_MR) + (yk_ML)) / 2) * T;
        DS     += v;
        w_robot = (RADIUS * ((yk_MR) - (yk_ML)) / LENGTH) * T;
        robot.theta  += w_robot;

        //system input

        if(abs((desired_displacement - DS)) >= LIMIT_LINEAR_PRECISION_RATE && abs((theta_desired - global.theta)) >= LIMIT_ANGULAR_PRECISION_RATE)
        {            
            ext_ctrl = control_system(0, theta_desired, global.theta, desired_displacement, DS);
        }

        else if(abs(desired_displacement - DS) >= LIMIT_LINEAR_PRECISION_RATE && abs(theta_desired - global.theta) <= LIMIT_ANGULAR_PRECISION_RATE)
        {
            ext_ctrl = control_system(1, theta_desired, global.theta, desired_displacement, DS);
        }

        else if(abs(desired_displacement - DS) <= LIMIT_LINEAR_PRECISION_RATE && abs(theta_desired - global.theta) >= LIMIT_ANGULAR_PRECISION_RATE)
        {
            ext_ctrl = control_system(2, theta_desired, global.theta, desired_displacement, DS);
        }

        else
        {
            printf("(%4.2f) displacement (m) : %7.4f | desired disp (m) : %7.4f | global theta (o) : %7.4f\n", float(i*T), DS, desired_displacement, global.theta);
            break;
        }
              
        VR  = ((2*ext_ctrl.uk_disp) + (ext_ctrl.uk_ang*LENGTH))/(2);
        VL  = ((2*ext_ctrl.uk_disp) - (ext_ctrl.uk_ang*LENGTH))/(2);
        
        //converting linear motor velocity to angular motor velocity
        WR_ref = VR/RADIUS;
        WL_ref = VL/RADIUS;

        control_motors(WR_ref, WL_ref, &yk_MR, &yk_ML);

        // usleep(50000);
        Sleep(50);
    }
    return 0;
}


//Ke = V/w = 0.357995
//v = 0.357995*w
void calculate_coordinates (coordinate_global *global, coordinate_local *local, float distance)
{
    global->x = distance*(cos(global->theta_1 + (local->delta_theta/2)));
    global->y = distance*(sin(global->theta_1 + (local->delta_theta/2)));
    local->x = distance*(cos(local->theta));
    local->y = distance*(sin(local->theta));

    local->delta_theta = local->theta - local->theta_1;
    global->theta = global->theta_1 + local->delta_theta;

    global->theta_1 = global->theta; 
    local->theta_1 = local->theta; 
}

external_controller control_system(uint8_t mode, float ang_setpoint, float PV_ang_setpoint, float lin_setpoint, float PV_lin_setpoint)
{
    static float error_ang_k = 0, error_ang_k1 = 0,
    error_disp_k, error_disp_k1;
    external_controller ang_disp;

    if(mode == 0)
    {
        error_ang_k = ang_setpoint - PV_ang_setpoint;
        ang_disp.uk_ang = 1.4939*error_ang_k - 1.442808*error_ang_k1 + ang_disp.uk_ang1;
        if(ang_disp.uk_ang > LIMIT_ANGULAR_ROBOT_SPEED)
            ang_disp.uk_ang  = LIMIT_ANGULAR_ROBOT_SPEED;
        else if(ang_disp.uk_ang < -LIMIT_ANGULAR_ROBOT_SPEED)
            ang_disp.uk_ang = -LIMIT_ANGULAR_ROBOT_SPEED;
        // uk = 0.7845; //w //open loop

        error_ang_k1 = error_ang_k;
        ang_disp.uk_ang1 = ang_disp.uk_ang;

        error_disp_k = lin_setpoint - PV_lin_setpoint;
        ang_disp.uk_disp = 1.4939*error_disp_k - 1.442808*error_disp_k1 + ang_disp.uk_disp1; //linear speed
        if(ang_disp.uk_disp > LIMIT_LINEAR_ROBOT_SPEED)
            ang_disp.uk_disp  = LIMIT_LINEAR_ROBOT_SPEED;
        else if(ang_disp.uk_disp < -LIMIT_LINEAR_ROBOT_SPEED)
            ang_disp.uk_disp = -LIMIT_LINEAR_ROBOT_SPEED;
        error_disp_k1 = error_disp_k;
        ang_disp.uk_disp1 = ang_disp.uk_disp;
    }

    else if(mode == 1)
    {
        error_disp_k = lin_setpoint - PV_lin_setpoint;
        ang_disp.uk_disp = 1.4939*error_disp_k - 1.442808*error_disp_k1 + ang_disp.uk_disp1; //linear speed
        if(ang_disp.uk_disp > LIMIT_LINEAR_ROBOT_SPEED)
            ang_disp.uk_disp  = LIMIT_LINEAR_ROBOT_SPEED;
        else if(ang_disp.uk_disp < -LIMIT_LINEAR_ROBOT_SPEED)
            ang_disp.uk_disp = -LIMIT_LINEAR_ROBOT_SPEED;
        error_disp_k1 = error_disp_k;
        ang_disp.uk_disp1 = ang_disp.uk_disp;
    }

    else
    {
        error_ang_k = ang_setpoint - PV_ang_setpoint;
        ang_disp.uk_ang = 1.4939*error_ang_k - 1.442808*error_ang_k1 + ang_disp.uk_ang1;
        if(ang_disp.uk_ang > LIMIT_ANGULAR_ROBOT_SPEED)
            ang_disp.uk_ang  = LIMIT_ANGULAR_ROBOT_SPEED;
        else if(ang_disp.uk_ang < -LIMIT_ANGULAR_ROBOT_SPEED)
            ang_disp.uk_ang = -LIMIT_ANGULAR_ROBOT_SPEED;
        // uk = 0.7845; //w //open loop

        error_ang_k1 = error_ang_k;
        ang_disp.uk_ang1 = ang_disp.uk_ang;

        ang_disp.uk_disp = 0; //zero linear speed
    }

    // V_robot = 0.5*T; // m/s - definido        
    // uk_disp = V_robot/T;

    return ang_disp;
}

void control_motors(float w_ref_MR, float w_ref_ML, float *yk_MR, float *yk_ML)
{
    static float yk_1_MR = 0, uk_1_MR = 0;  //u is voltage 
    static float yk_1_ML = 0, uk_1_ML = 0; 
    static float uk_MR = 0, ek_MR = 0, ek_1_MR = 0;
    static float uk_ML = 0, ek_ML = 0, ek_1_ML = 0;


    if(w_ref_MR > LIMIT_ANGULAR_MOTOR_SPEED)
            w_ref_MR = LIMIT_ANGULAR_MOTOR_SPEED;
    else if (w_ref_MR < -LIMIT_ANGULAR_MOTOR_SPEED)
        w_ref_MR = -LIMIT_ANGULAR_MOTOR_SPEED;

    if(w_ref_ML > LIMIT_ANGULAR_MOTOR_SPEED)
        w_ref_ML = LIMIT_ANGULAR_MOTOR_SPEED;
    else if (w_ref_ML < -LIMIT_ANGULAR_MOTOR_SPEED)
        w_ref_ML = -LIMIT_ANGULAR_MOTOR_SPEED;

    // voltsR = 0.347247*w_ref_MR; //open loop
    // voltsL = 0.347247*w_ref_ML;

    // if(voltsR > 12)
    //     voltsR = 12;
    // else if (voltsR < -12)
    //     voltsR = -12;
    
    // if(voltsL > 12)
    //     voltsL = 12;
    // else if (voltsL < -12)
    //     voltsL = -12;

    //motors control
    ek_MR = w_ref_MR - *yk_MR;
    ek_ML = w_ref_ML - *yk_ML;

    // uk_MR = voltsR; //open loop
    // uk_ML = voltsL;
    uk_MR = 0.31167*ek_MR-0.109832*ek_1_MR+uk_1_MR;
    uk_ML = 0.31167*ek_ML-0.109832*ek_1_ML+uk_1_ML;

    if(uk_MR > LIMIT_OUTPUT_VOLTAGE)
        uk_MR = LIMIT_OUTPUT_VOLTAGE;
    else if (uk_MR < -LIMIT_OUTPUT_VOLTAGE)
        uk_MR = -LIMIT_OUTPUT_VOLTAGE;
    
    if(uk_ML > LIMIT_OUTPUT_VOLTAGE)
        uk_ML = LIMIT_OUTPUT_VOLTAGE;
    else if (uk_ML < -LIMIT_OUTPUT_VOLTAGE)
        uk_ML = -LIMIT_OUTPUT_VOLTAGE;

    //motors plant
    *yk_MR = 0.5647*yk_1_MR + 1.387*uk_1_MR;
    *yk_ML = 0.5647*yk_1_ML + 1.387*uk_1_ML;

    if(*yk_MR > LIMIT_ANGULAR_MOTOR_SPEED)
        *yk_MR = LIMIT_ANGULAR_MOTOR_SPEED;
    else if (*yk_MR < -LIMIT_ANGULAR_MOTOR_SPEED)
        *yk_MR = -LIMIT_ANGULAR_MOTOR_SPEED;

    if(*yk_ML > LIMIT_ANGULAR_MOTOR_SPEED)
        *yk_ML = LIMIT_ANGULAR_MOTOR_SPEED;
    else if (*yk_ML < -LIMIT_ANGULAR_MOTOR_SPEED)
        *yk_ML = -LIMIT_ANGULAR_MOTOR_SPEED;
    //updating past samples
    yk_1_MR = *yk_MR;
    yk_1_ML = *yk_ML; //rad/s
    ek_1_MR = ek_MR;
    ek_1_ML = ek_ML;
    uk_1_MR = uk_MR;
    uk_1_ML = uk_ML;
}