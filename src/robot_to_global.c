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

    X_desired = 1;
    Y_desired = 1;

    desired_displacement = sqrt((X_desired*X_desired)+(Y_desired*Y_desired));
    theta_desired = atan2(Y_desired, X_desired);
    // theta_desired = 3.14;
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
        static float w = 0, voltsR = 0, voltsL = 0, error_disp_k, error_disp_k1, uk_disp, uk_disp_1;
        //calculate the actual global theta
        calculate_coordinates(&global, &robot, DS);

        // printf("(%d) Motor Right (rad/s): %.2f | Motor Left (rad/s): %.2f\n",i, yk_MR, yk_ML);
        // printf("Displacement (m): %7.2f | (%3d) Speed %7.2f | Theta (rad): %7.4f | Angular velocity (rad/s): %7.4f\n", DS, i, v, robot.theta, w_robot);
        printf("(%7.2f) global theta : %7.4f | WrefR : %7.4f | WrefL : %7.4f | v: %7.4f | DS: %7.4f | vR: %7.4f | vL: %7.4f\n", float(i*T),global.theta,  yk_MR, yk_ML, (v/T), DS, uk_MR, uk_ML);

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

        if(abs((theta_desired - global.theta)) >= 0.001)
        {            
            error_ang_k = theta_desired - global.theta;
            uk = 0.33244*error_ang_k - 0.306110*error_ang_k1 + uk_1; //angular speed
            if(uk > 18.367123)
                uk  = 18.367123;
            else if(uk < -18.367123)
                uk = -18.367123;
            // uk = 0; //w

            error_disp_k = desired_displacement - DS;
            uk_disp = 0.97147*error_disp_k - 0.789610*error_disp_k1 + uk_disp_1; //linear speed
            if(uk_disp > 1.34)
                uk_disp  = 1.34;
            else if(uk_disp < -1.34)
                uk_disp = -1.34;
            error_disp_k1 = error_disp_k;
            uk_disp_1 = uk_disp;
            V_robot = 1.0*T; // m/s - definido        

            VR  = ((2*uk_disp) + (uk*L))/(2*radius);
            VL  = ((2*uk_disp) - (uk*L))/(2*radius);
            
            //simulacao do ideal
            // theta_k = 0.975*wk_1 + theta_k_1;
            // wk_1 = uk_1;
            // theta_k_1 = theta_k;

            error_ang_k1 = error_ang_k;
            uk_1 = uk;

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

            voltsR = 0.357995*w_ref_MR;
            voltsL = 0.357995*w_ref_ML;

            if(voltsR > 6)
                voltsR = 6;
            else if (voltsR < -6)
                voltsR = -6;
            
            if(voltsL > 6)
                voltsL = 6;
            else if (voltsL < -6)
                voltsL = -6;
            

            //motors control
            ek_MR = w_ref_MR - yk_MR;
            ek_ML = w_ref_ML - yk_ML;

            // uk_MR = voltsR;
            // uk_ML = voltsL;
            uk_MR = 0.18602*ek_MR - 0.077867*ek_1_MR + uk_1_MR;
            uk_ML = 0.18602*ek_ML - 0.077867*ek_1_ML + uk_1_ML;

            if(uk_MR > 6)
                uk_MR = 6;
            else if (uk_MR < -6)
                uk_MR = -6;
            
            if(uk_ML > 6)
                uk_ML = 6;
            else if (uk_ML < -6)
                uk_ML = -6;

            yk_MR = 0.5647*yk_1_MR + 3.658*uk_1_MR;
            yk_ML = 0.5647*yk_1_ML + 3.658*uk_1_ML;

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
        }

        else
        {
            yk_MR = 0;
            yk_ML = 0;
            printf("(%7.2f) displacement (m) : %7.4f | desired disp (m) : %7.4f | linear speed (m/s) : %7.4f\n", float(i*T), DS, desired_displacement, (v*T));
            break;
        }


        Sleep(50);
    }
    

    return 0;
}


//Ke = V/w = 0.357995
//v = 0.357995*w
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

