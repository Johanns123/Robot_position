#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <windows.h> 
// #include <unistd.h> //for linux is  unistd

#define DEG_TO_RAD M_PI/180
#define RAD_TO_DEG 180/M_PI
#define LENGTH 0.146 //meters
#define RADIUS 0.040 //meters
#define LIMIT_ANGULAR_ROBOT_SPEED 18.367123
#define LIMIT_LINEAR_ROBOT_SPEED 1.34
#define LIMIT_ANGULAR_MOTOR_SPEED 33.52
#define LIMIT_OUTPUT_VOLTAGE 12
#define NUM_POINTS 4 


typedef struct
{
    float x;
    float y;
    float x0;
    float y0;
    float theta;
    float theta0;
} coordinate_global;


typedef struct
{
    float uk_ang;
    float uk_ang1;
    float uk_disp;
    float uk_disp1;
} external_controller;


void calculate_coordinates (coordinate_global *global);
external_controller control_system(float error_ang, float error_lin);
void control_motors(float w_refR, float w_refL, float *yk_MR, float *yk_ML);
void calc_pose(void);


float T = 50e-3;
// int iterador = 0;
// float x_goal [NUM_POINTS] = {1.0, 1.0, 0.0, 0.0};
// float y_goal [NUM_POINTS] = {0.0, 1.0, 1.0, 0.0};
float x_goal = 0;
float x_goal_global = 0;
float y_goal = 0;
float y_goal_global = 0;
float phi = 0, phi0 = 0, phi_desired = 0, delta_phi = 0;
float delta_distance = 0, distance0 = 0;
float x = 0, y = 0;
float error_x = 0, error_y = 0;
float delta_s_desired = 0;
float DS;

int main(void)
{
    // FILE *fp0 = fopen("angular.txt", "w" );
    // if(fp0 == NULL)
    // {
    //     printf("Error opening file!\n");
    //     return 0;
    // }
    // FILE *fp1 = fopen("WrefR.txt", "w" );
    // if(fp1 == NULL)
    // {
    //     printf("Error opening file!\n");
    //     return 0;
    // }
    // FILE *fp2 = fopen("WrefL.txt", "w" );
    // if(fp2 == NULL)
    // {
    //     printf("Error opening file!\n");
    //     return 0;
    // }
    // FILE *fp3 = fopen("displacement.txt", "w" );
    // if(fp3 == NULL)
    // {
    //     printf("Error opening file!\n");
    //     return 0;
    // }

    static uint16_t i = 0;
    static float v = 0, w_robot =  0; //meters
    static float WR_ref = 0, WL_ref = 0, VR = 0, VL = 0, yk_MR, yk_ML;
    static float error_disp = 0, error_angular = 0;
    // static float voltsR = 0, voltsL = 0; //for opne loop
    coordinate_global global;
    external_controller ext_ctrl;
    global.x = 0;
    global.y = 0;
    global.theta0 = 0;
    

    printf("Where is your Robot?\n");
    printf("Insert X global position>> ");
    scanf("%f", &global.x);
    printf("Insert Y global position>> ");
    scanf("%f", &global.y);
    printf("Insert the Robot to Global angle>> ");
    scanf("%f", &phi);

    printf("Insert the desired coordinates\n");
    printf("Insert X goal>> ");
    scanf("%f", &x_goal_global);
    printf("Insert Y goal>> ");
    scanf("%f", &y_goal_global);

    x_goal = x_goal_global - global.x;
    y_goal = y_goal_global - global.y;
    phi = phi * M_PI/180;
    
    delta_s_desired = sqrtf(powf((float)(x_goal),2)+ powf((float)(y_goal),2));
    phi_desired     = atan2f(y_goal,x_goal);
    // usleep(1000000);
    Sleep(1000);

    while(1)
    {
        // printf("(%3.2f) %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n",float(i*T), x_goal[iterador], y_goal[iterador], x, y, global.x, global.y);        // if(i == 200) //close the file with 200 samples
        printf("(%3.2f) X desired:%7.4f Y desired:%7.4f X local:%7.4f Y local:%7.4f X global:%7.4f Y global:%7.4f\n",float(i*T), delta_distance, delta_phi, x, y, global.x, global.y);        // if(i == 200) //close the file with 200 samples
        // {
        //     fclose(fp0);
        //     fclose(fp1);
        //     fclose(fp2);
        //     fclose(fp3);
        //     // break;
        // }

        // else
        // {
        //     fprintf(fp0, "%7.4f\n", global.theta);
        //     fprintf(fp1, "%7.4f\n", yk_MR);
        //     fprintf(fp2, "%7.4f\n", yk_ML);
        //     fprintf(fp3, "%7.4f\n", DS);
        // }
        i++;

        //calculating displacement and robot theta
        calc_pose();
        calculate_coordinates(&global);
        v       = (RADIUS * ((yk_MR) + (yk_ML)) / 2) * T;
        DS     += v;
        w_robot = (RADIUS * ((yk_MR) - (yk_ML)) / LENGTH) * T;
        phi  += w_robot;
        delta_distance = DS - distance0;
        distance0  = DS;
        delta_phi = phi - phi0;
        phi0 = phi;

        // phi_desired = atan2f(y_goal-y, x_goal-x);
        // error_x = x_goal - x;
        // error_y = y_goal - y;
        // error_disp = sqrt((error_x*error_x)+(error_y*error_y));
        // error_angular = atan2f(sin(phi_desired-phi), cos(phi_desired-phi));
        error_disp = delta_s_desired - DS;
        error_angular = phi_desired-phi;

        //system input

        // if(abs(error_x) <= 0.1 && abs(error_y) <= 0.1)
        // {
        //     // if(iterador <= (NUM_POINTS-1))
        //     //     iterador++;

        //     // else
        //     // {
        //     //     // i = 0;
        //     //     break;    
        //     // }
        //     break;
        // }

        // if(abs(error_disp) <= 0.01)
        // {
        //     // if(iterador <= (NUM_POINTS-1))
        //     //     iterador++;

        //     // else
        //     // {
        //     //     // i = 0;
        //     //     break;    
        //     // }
        //     break;
        // }

        
        // else
        // {
            ext_ctrl = control_system(error_angular, error_disp);
        // }
              
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

void calc_pose(void)
{
    x = DS*cosf(phi);
    y = DS*sinf(phi);
}

//Ke = V/w = 0.357995
//v = 0.357995*w
void calculate_coordinates (coordinate_global *global)
{  
    global->x = DS*cosf(global->theta0 + delta_phi/2);
    global->y = DS*sinf(global->theta0 + delta_phi/2);
    global->theta = global->theta0 + delta_phi;
    global->theta0 = global->theta;

    // printf("%4.2f %4.2f %4.2f %4.2f %4.2f\n", global->x, delta_distance, global->theta0, delta_phi, delta_distance*cosf(global->theta0 + delta_phi/2));
}

external_controller control_system(float error_ang, float error_lin)
{
    static float error_ang_k1 = 0, error_disp_k1;
    external_controller ang_disp;

    ang_disp.uk_ang = 1.4939*error_ang - 1.442808*error_ang_k1 + ang_disp.uk_ang1;
    if(ang_disp.uk_ang > LIMIT_ANGULAR_ROBOT_SPEED)
        ang_disp.uk_ang  = LIMIT_ANGULAR_ROBOT_SPEED;
    else if(ang_disp.uk_ang < -LIMIT_ANGULAR_ROBOT_SPEED)
        ang_disp.uk_ang = -LIMIT_ANGULAR_ROBOT_SPEED;
    // uk = 0.7845; //w //open loop

    error_ang_k1 = error_ang;
    ang_disp.uk_ang1 = ang_disp.uk_ang;

    ang_disp.uk_disp = 1.4939*error_lin - 1.442808*error_disp_k1 + ang_disp.uk_disp1; //linear speed
    if(ang_disp.uk_disp > LIMIT_LINEAR_ROBOT_SPEED)
        ang_disp.uk_disp  = LIMIT_LINEAR_ROBOT_SPEED;
    else if(ang_disp.uk_disp < -LIMIT_LINEAR_ROBOT_SPEED)
        ang_disp.uk_disp = -LIMIT_LINEAR_ROBOT_SPEED;
    error_disp_k1 = error_lin;
    ang_disp.uk_disp1 = ang_disp.uk_disp;

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