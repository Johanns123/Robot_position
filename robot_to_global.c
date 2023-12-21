#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <windows.h> //for linus is  unistd

#define DEG_TO_RAD M_PI/180
#define RAD_TO_DEG 180/M_PI

typedef struct
{
    float x;
    float x_1;
    float y;
    float y_1;
    float theta;
    float theta_1;
} coordinate;

void calculate_coordinates (coordinate *global, coordinate *local, float distance);
void calc_distance(float *d, float d0, uint8_t v, float T);

float theta_desired = 0;

int main(void)
{
    coordinate global;
    coordinate robot;
    
    global.x_1 = 0, global.y_1 = 0, global.theta_1 = 45*DEG_TO_RAD, global.theta = 0;
    robot.theta_1 = 0, robot.theta = 0, robot.x = robot.x_1 = robot.y = robot.y_1 = 0;
    float d = 0, d0 = 0, T = 50e-3;
    unsigned int v = 0;
    float X_desired = 0, Y_desired = 0;

    while(1)
    {

        printf("Insert the desired coordinates\n");
        printf("X: ");  scanf("%f", &X_desired);
        printf("Y: ");  scanf("%f", &Y_desired);
        printf("\n\n");

        theta_desired = atan2(Y_desired, X_desired) * RAD_TO_DEG; //rad

        printf("theta desired: %.2f\n\n", theta_desired);

        printf("Insert initial global coordinates of the robot:\n");
        printf("X: "); scanf("%f", &global.x_1);
        printf("Y: "); scanf("%f", &global.y_1);
        printf("Theta: "); scanf("%f", &robot.theta_1);
        printf("\n\n");

        global.theta_1 = atan2(global.y_1, global.x_1)*RAD_TO_DEG;

        printf("Insert the velocity of the robot:\n");
        printf("Velocity cm/s: "); scanf("%d", &v);
        printf("\n\n");
        
        printf("=============Initial values==========\n");
        printf("x: %.2f  y: %.2f theta: %.2f\n", global.x_1, global.y_1, global.theta_1);

        Sleep(500);

        while((fabs((global.x_1 - X_desired)) > 0.1) && (fabs((global.y_1 - Y_desired)) > 0.1))
        {
            printf("delta x: %.2f delta y: %.2f\n\n", fabs((global.x_1 - X_desired)), fabs((global.y_1 - Y_desired)));
            calc_distance(&d, d0, v, T);
            calculate_coordinates(&global, &robot, d);
            printf("=============Actual values==========\n");
            printf("x: %.2f  y: %.2f theta: %.2f\n", global.x, global.y, global.theta);
            printf("atan: %.2f\n\n\n\n\n", atan2(global.y, global.x)*RAD_TO_DEG);
            Sleep(10);
        }

        printf("===========Desired values==========\n");
        printf("x: %.2f  y: %.2f theta: %.2f\n\n", X_desired, Y_desired, theta_desired);

        printf("===========Obtained values=========\n");
        printf("x: %.2f  y: %.2f theta: %.2f\n\n", global.x, global.y, atan2(global.y, global.x)*RAD_TO_DEG);

        printf("=============Error in %%=============\n");
        printf("x: %.2f%%  y: %.2f%% theta: %.2f%%\n\n\n\n", (X_desired - global.x)/X_desired, (Y_desired - global.y)/Y_desired, (theta_desired - (atan2(global.y, global.x)*RAD_TO_DEG))/theta_desired);

        printf("Want to continue? Press '1' to yes or '0' to no: ");
        uint8_t choice = 0;
        scanf("%d", &choice);
        printf("\n\n\n\n");

        if(!choice)
            break;
    }

    return 0;
}

void calculate_coordinates (coordinate *global, coordinate *local, float distance)
{

    global->theta *= DEG_TO_RAD; 
    global->theta_1 *= DEG_TO_RAD;
    local->theta *= DEG_TO_RAD;
    local->theta_1 *= DEG_TO_RAD;

    global->x = global->x_1 + distance*(cos(global->theta_1 + (local->theta/2)));
    global->y = global->y_1 + distance*(sin(global->theta_1 + (local->theta/2)));
    
    global->theta = global->theta_1 + local->theta_1*2;

    global->theta_1 = global->theta; 
    global->x_1 = global->x;
    global->y_1 = global->y;

    local->theta = (theta_desired*DEG_TO_RAD) - global->theta;
    local->theta_1 = local->theta;

    global->theta *= RAD_TO_DEG; 
    global->theta_1 *= RAD_TO_DEG;
    local->theta *= RAD_TO_DEG;
    local->theta_1 *= RAD_TO_DEG;

    // printf("global: %.2f desired: %.2f\n", global->theta, theta_desired);
}

void calc_distance(float *d, float d0, uint8_t v, float T)
{
    *d = d0 + v*T;
}
    // printf("Insert a value to initial local x and y and a global theta in rad: ");
    // scanf("%f %f %f", &robot.x, &robot.y, &global.theta);
    // puts("\n");

    // global.x = (robot.x * cos(global.theta)) - (robot.y * sin(global.theta));
    // global.y = (robot.x * sin(global.theta)) + (robot.y * cos(global.theta));
    
    // printf("X: %.2f  Y: %.2f\n", global.x, global.y);