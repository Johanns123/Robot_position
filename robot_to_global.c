#include <stdio.h>
#include <math.h>
#include <stdint.h>

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

int main(void)
{
    coordinate global;
    coordinate robot;
    
    global.x_1 = 0, global.y_1 = 0, global.theta_1 = 45*DEG_TO_RAD;
    robot.theta_1 = 0, robot.theta = 0, robot.x = robot.x_1 = robot.y = robot.y_1 = 0;
    float d  = 0;

    uint8_t choice = 1;
    char chr = 0;

    while(choice)
    {

        printf("Insert initial global coordinates of the robot:\n");
        printf("X: "); scanf("%f", &global.x_1);
        printf("Y: "); scanf("%f", &global.y_1);
        printf("Theta: "); scanf("%f", &global.theta_1);
        printf("\n\n");

        printf("Insert the local angle of the robot and the travalled distance:\n");
        printf("Theta: "); scanf("%f", &robot.theta);
        printf("Distance: "); scanf("%f", &d);
        printf("\n\n");

        printf("=============Initial values==========\n");
        printf("x: %.2f  y: %.2f theta: %.2f\n", global.x_1, global.y_1, global.theta_1);
        
        calculate_coordinates(&global, &robot, d);
        
        printf("=============Actual values==========\n");
        printf("x: %.2f  y: %.2f theta: %.2f\n\n\n\n\n", global.x, global.y, global.theta);

        printf("Do u wanna calc another step? Press Y to 'yes' or N to 'no': ");
        scanf("%c", &chr);

        if(chr == 'Y' || chr == 'y')
            choice = 1;

        else if(chr == 'N' || chr == 'n')
            choice = 0;
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
    global->theta = global->theta_1 + local->theta;

    global->theta_1 = global->theta; 
    global->x_1 = global->x;
    global->y_1 = global->y;

    local->theta_1 = local->theta;

    global->theta *= RAD_TO_DEG; 
    global->theta_1 *= RAD_TO_DEG;
    local->theta *= RAD_TO_DEG;
    local->theta_1 *= RAD_TO_DEG;
}


    // printf("Insert a value to initial local x and y and a global theta in rad: ");
    // scanf("%f %f %f", &robot.x, &robot.y, &global.theta);
    // puts("\n");

    // global.x = (robot.x * cos(global.theta)) - (robot.y * sin(global.theta));
    // global.y = (robot.x * sin(global.theta)) + (robot.y * cos(global.theta));
    
    // printf("X: %.2f  Y: %.2f\n", global.x, global.y);