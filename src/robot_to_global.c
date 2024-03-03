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

void calculate_coordinates (coordinate_global *global, coordinate_local *local, float distance);

float theta_desired = 0, theta0 = 0, desired_displacement = 0;
float d = 0, d0 = 0, T = 50e-3;

int main(void)
{
    coordinate_global global;
    coordinate_local robot;
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

    X_desired = 0;
    Y_desired = 0.20;

    desired_displacement = sqrt((X_desired*X_desired)+(Y_desired*Y_desired));
    theta_desired = atan2(Y_desired, X_desired);
    // theta_desired = 3.14;
    printf("%f\n", theta_desired);
    // usleep(1000000);
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
        static float voltsR = 0, voltsL = 0, error_disp_k, error_disp_k1, uk_disp, uk_disp_1;
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
        v       = (radius * ((yk_MR) + (yk_ML)) / 2) * T;
        DS     += v;
        w_robot = (radius * ((yk_MR) - (yk_ML)) / L) * T;
        robot.theta  += w_robot;

        //system input

        if(abs((desired_displacement - DS)) >= 0.001 && abs((theta_desired - global.theta)) >= 0.0001)
        {            
            error_ang_k = theta_desired - global.theta;
            uk = 1.4939*error_ang_k - 1.442808*error_ang_k1 + uk_1;
            if(uk > 18.367123)
                uk  = 18.367123;
            else if(uk < -18.367123)
                uk = -18.367123;
            // uk = theta_desired; //w

            error_ang_k1 = error_ang_k;
            uk_1 = uk;

            error_disp_k = desired_displacement - DS;
            uk_disp = 1.4939*error_disp_k - 1.442808*error_disp_k1 + uk_disp_1; //linear speed
            if(uk_disp > 1.34)
                uk_disp  = 1.34;
            else if(uk_disp < -1.34)
                uk_disp = -1.34;
            error_disp_k1 = error_disp_k;
            uk_disp_1 = uk_disp;
        }

        else if(abs(desired_displacement - DS) >= 0.001 && abs(theta_desired - global.theta) <= 0.0001)
        {
            uk = 0;

            error_disp_k = desired_displacement - DS;
           uk_disp = 1.4939*error_disp_k - 1.442808*error_disp_k1 + uk_disp_1; //linear speed
            if(uk_disp > 1.34)
                uk_disp  = 1.34;
            else if(uk_disp < -1.34)
                uk_disp = -1.34;
            error_disp_k1 = error_disp_k;
            uk_disp_1 = uk_disp;

            // V_robot = 0.5*T; // m/s - definido        
            // uk_disp = V_robot/T;

            VR  = ((2*uk_disp) + (uk*L))/(2);
            VL  = ((2*uk_disp) - (uk*L))/(2);
        }

        else if(abs(desired_displacement - DS) <= 0.001 && abs(theta_desired - global.theta) >= 0.0001)
        {
            error_ang_k = theta_desired - global.theta;
            uk = 1.4939*error_ang_k - 1.442808*error_ang_k1 + uk_1;
            if(uk > 18.367123)
                uk  = 18.367123;
            else if(uk < -18.367123)
                uk = -18.367123;
            // uk = theta_desired; //w

            error_ang_k1 = error_ang_k;
            uk_1 = uk;

            uk_disp = 0;
            VR  = ((2*uk_disp) + (uk*L))/(2);
            VL  = ((2*uk_disp) - (uk*L))/(2);
        }

        else
        {
            printf("(%4.2f) displacement (m) : %7.4f | desired disp (m) : %7.4f | global theta (o) : %7.4f\n", float(i*T), DS, desired_displacement, global.theta);
            break;
        }
        
        // V_robot = 0.5*T; // m/s - definido        
        // uk_disp = V_robot/T;

        VR  = ((2*uk_disp) + (uk*L))/(2);
        VL  = ((2*uk_disp) - (uk*L))/(2);
        

        //converting linear motor velocity to angular motor velocity
        w_ref_MR = VR/radius;
        w_ref_ML = VL/radius;

        if(w_ref_MR > 33.52)
            w_ref_MR = 33.52;
        else if (w_ref_MR < -33.52)
            w_ref_MR = -33.52;

        if(w_ref_ML > 33.52)
            w_ref_ML = 33.52;
        else if (w_ref_ML < -33.52)
            w_ref_ML = -33.52;

        // voltsR = 0.347247*w_ref_MR;
        // voltsL = 0.347247*w_ref_ML;

        if(voltsR > 12)
            voltsR = 12;
        else if (voltsR < -12)
            voltsR = -12;
        
        if(voltsL > 12)
            voltsL = 12;
        else if (voltsL < -12)
            voltsL = -12;
        
        // w_ref_MR = -5;
        // w_ref_ML = -10;

        //motors control
        ek_MR = w_ref_MR - yk_MR;
        ek_ML = w_ref_ML - yk_ML;

        // uk_MR = voltsR;
        // uk_ML = voltsL;
        uk_MR = 0.31167*ek_MR-0.109832*ek_1_MR+uk_1_MR;
        uk_ML = 0.31167*ek_ML-0.109832*ek_1_ML+uk_1_ML;

        if(uk_MR > 12)
            uk_MR = 12;
        else if (uk_MR < -12)
            uk_MR = -12;
        
        if(uk_ML > 12)
            uk_ML = 12;
        else if (uk_ML < -12)
            uk_ML = -12;

        yk_MR = 0.5647*yk_1_MR + 1.387*uk_1_MR;
        yk_ML = 0.5647*yk_1_ML + 1.387*uk_1_ML;

        if(yk_MR > 33.52)
            yk_MR = 33.52;
        else if (yk_MR < -33.52)
            yk_MR = -33.52;

        if(yk_ML > 33.52)
            yk_ML = 33.52;
        else if (yk_ML < -33.52)
            yk_ML = -33.52;
        //updating past samples
        yk_1_MR = yk_MR;
        yk_1_ML = yk_ML; //rad/s
        ek_1_MR = ek_MR;
        ek_1_ML = ek_ML;
        uk_1_MR = uk_MR;
        uk_1_ML = uk_ML;

        // else
        // {
        //     yk_MR = 0;
        //     yk_ML = 0;
        //     printf("(%7.2f) displacement (m) : %7.4f | desired disp (m) : %7.4f | linear speed (m/s) : %7.4f\n", float(i*T), DS, desired_displacement, (v*T));
        //     break;
        // }

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

