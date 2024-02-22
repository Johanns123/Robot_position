#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <windows.h> 
// #include <unistd.h> //for linux is  unistd

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

float theta_desired = 0, theta0 = 0, desired_displacement = 0;
float d = 0, d0 = 0, T = 50e-3;

int main(void)
{
    coordinate global;
    coordinate robot;
    global.x_1 = 0, global.y_1 = 0, global.theta_1 = 0, global.theta = 0;
    robot.theta_1 = 0, robot.theta = 0, robot.x = robot.x_1 = robot.y = robot.y_1 = 0;
    // unsigned int v = 0;
    float X_desired = 0, Y_desired = 0;

    FILE *fp = fopen("output.txt", "w" );
    if(fp == NULL)
    {
        printf("Error opening file!\n");
        return 0;
    }

    X_desired = 2;
    Y_desired = 2;

    desired_displacement = sqrt((X_desired*X_desired)+(Y_desired*Y_desired));
    theta_desired = atan2(Y_desired, X_desired);
    // theta_desired = 0.02;
    printf("%f\n", theta_desired);
    Sleep(1000);

    while(1)
    {
        static float yk_MR = 0, yk_1_MR = 0, uk_1_MR = 0;  //u is voltage 
        static float yk_ML = 0, yk_1_ML = 0, uk_1_ML = 0; 
        static float uk_MR = 0, ek_MR = 0, ek_1_MR = 0;
        static float uk_ML = 0, ek_ML = 0, ek_1_ML = 0;
        static float w_ref_MR =  5, w_ref_ML = 5; //ang freq reference
        static uint16_t i = 0;
        static float v = 0, w_robot =  0, DS, radius = 0.04, L = 0.146; //meters
        static float V_robot = 0, VR = 0, VL = 0;
        static float uk = 0, uk_1, error_ang_k = 0, error_ang_k1 = 0;
        static float theta_k = 0, wk_1 = 0, theta_k_1 = 0;
        static float w = 0;
        //calculate the actual global theta
        calculate_coordinates(&global, &robot, DS);

        // printf("(%d) Motor Right (rad/s): %.2f | Motor Left (rad/s): %.2f\n",i, yk_MR, yk_ML);
        // printf("Displacement (m): %7.2f | (%3d) Speed %7.2f | Theta (rad): %7.4f | Angular velocity (rad/s): %7.4f\n", DS, i, v, robot.theta, w_robot);
        printf("(%7.2f) displacement (m) : %7.4f | desired disp (m) : %7.4f | linear speed (m/s) : %7.4f\n", float(i*T), DS, desired_displacement, (v*T));
        
        if(i == 200) //close the file with 200 samples
        {
            fclose(fp);
            // break;
        }

        else
        {
            fprintf(fp, "%7.4f\n", DS);
        }
        i++;

        //calculating displacement and robot theta
        v       = (radius * ((yk_MR) + (yk_ML)) / 2) * T;
        DS     += v;
        w_robot = (radius * ((yk_MR) - (yk_ML)) / L) * T;
        robot.theta  += w_robot;

        //system input
        if(fabs(desired_displacement - DS) >= 0.001)
        {            
            
            // V_robot = 0.4*T; // m/s - definido        
            error_ang_k = desired_displacement - DS;
            uk = (2.928*error_ang_k - 2.827862*error_ang_k1)*0.05 + uk_1; //linear speed
            error_ang_k1 = error_ang_k;
            uk_1 = uk;

            VR  = ((2*uk) + (w*L))/(2*radius);
            VL  = ((2*uk) - (w*L))/(2*radius);
        }

        // if(abs((theta_desired - global.theta)) >= 0.001)
        // {            
        //     error_ang_k = theta_desired - global.theta;
        //     // uk = 0.075077*error_ang_k - 0.072509*error_ang_k1 + uk_1; //angular speed
        //     uk = 0.071819*error_ang_k - 0.017143*error_ang_k1 + 0.4893*uk_1; //angular speed
        //     // uk = theta_desired;
        //     V_robot = 0.4*T; // m/s - definido        

        //     VR  = ((2*V_robot) + (uk*L))/(2*radius);
        //     VL  = ((2*V_robot) - (uk*L))/(2*radius);
            
        //     //simulacao do ideal
        //     theta_k = 0.975*wk_1 + theta_k_1;
        //     wk_1 = uk_1;
        //     theta_k_1 = theta_k;

        //     error_ang_k1 = error_ang_k;
        //     uk_1 = uk;
        // }

        else
        {
            printf("(%7.2f) displacement (m) : %7.4f | desired disp (m) : %7.4f | linear speed (m/s) : %7.4f\n", float(i*T), DS, desired_displacement, (v*T));
            V_robot = 0;
            VR  = 0;
            VL  = 0;
            break;
        }

        //converting linear motor velocity to angular motor velocity
        w_ref_MR = VR/radius;
        w_ref_ML = VL/radius;

        //motors control
        ek_MR = w_ref_MR - yk_MR;
        ek_ML = w_ref_ML - yk_ML;

        uk_MR = 0.48435*ek_MR - 0.158721495*ek_1_MR + uk_1_MR;
        uk_ML = 0.48435*ek_ML - 0.158721495*ek_1_ML + uk_1_ML;

        yk_MR = 0.5134*yk_1_MR + 1.359*uk_1_MR;
        yk_ML = 0.5134*yk_1_ML + 1.39*uk_1_ML;

        //updating past samples
        yk_1_MR = yk_MR;
        yk_1_ML = yk_ML; //rad/s
        ek_1_MR = ek_MR;
        ek_1_ML = ek_ML;
        uk_1_MR = uk_MR;
        uk_1_ML = uk_ML;

        Sleep(50);
    }
    

    return 0;
}

void calculate_coordinates (coordinate *global, coordinate *local, float distance)
{
    global->x = distance*(cos(global->theta_1 + (local->theta/2)));
    global->y = distance*(sin(global->theta_1 + (local->theta/2)));
    
    global->theta = global->theta_1 + (local->theta-local->theta_1);

    global->theta_1 = global->theta; 
    // global->x_1 = global->x;
    // global->y_1 = global->y;
    local->theta_1 = local->theta; 

}

