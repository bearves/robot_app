#include <aris.h>
#include <cmath>
#include <iostream>

void calculate_coe(double* t, double* x, double* v, double* coe);

int main()
{
    double pi = 3.14159265359;
    double beginfootpos[18]={ 480   ,-283,      0,
                         236.88,-283,-236.88,
                        -236.88,-283,-236.88,
                        -480   ,-283,      0,
                        -236.88,-283, 236.88,
                         236.88,-283, 236.88};
    double beginbodypos[3]={0, 0, 0};

    double h=50; // high of step
    double d=400;// Distance of step
    double n=4;  // the number of step
    int totalcount= 2500;
    int controlfreq = 2000;
    int leg_id_A[3] = {0, 2, 4};
    int leg_id_B[3] = {1, 3, 5};
        
    double t[2]={0, totalcount*1.0/controlfreq};
    double x[2]={0, d/4};
    double v1[2]={0, 0.5*d/(totalcount*1.0/controlfreq)};
    double v2[2]={0.5*d/(totalcount*1.0/controlfreq), 0};
    double cubic_coefs1[4], cubic_coefs2[4];
    calculate_coe(t, x, v1, cubic_coefs1);
    calculate_coe(t, x, v2, cubic_coefs2);

    // Planning begin
    double footpos[18]; 
    double bodypos[3]; 
    std::copy(beginfootpos, beginfootpos+18, footpos);
    std::copy(beginbodypos, beginbodypos+3,  bodypos);

    for (int count = 0; count < 2*n*totalcount; count++)
    {
        int period_count=count % totalcount;
        double s=-pi/2*std::cos(pi*(period_count+1)*1.0/totalcount)+pi/2;
        
        if (period_count == 0)
        {
            std::copy(footpos, footpos+18, beginfootpos);
            std::copy(bodypos, bodypos+3,  beginbodypos);
        }
        
        double current_step_len;
        int* swing_leg_group;
        int* stance_leg_group;
        double t_r = (period_count+1)*1.0/controlfreq;
        double t_r2 = t_r * t_r;
        double t_r3 = t_r2 * t_r;

        if (count/totalcount == 0)
        {
            // the first step
            current_step_len = d/2;
            swing_leg_group = leg_id_A;
            stance_leg_group = leg_id_B;

            // body interp(accelerate stage)
            bodypos[0] = beginbodypos[0] + cubic_coefs1[0] * t_r3 + cubic_coefs1[1] * t_r2 +
                                   cubic_coefs1[2] * t_r + cubic_coefs1[3];
        }
        else if (count/totalcount == 2 * n - 1)
        {
            //the last step
            current_step_len = d/2;
            swing_leg_group = leg_id_B;
            stance_leg_group = leg_id_A;

            //body interp(decelerate stage)
            bodypos[0] = beginbodypos[0] + cubic_coefs2[0] * t_r3 + cubic_coefs2[1] * t_r2 +
                                   cubic_coefs2[2] * t_r + cubic_coefs2[3];
        }
        else if (count/totalcount % 2 == 1) //constant velocity stage
        {
            current_step_len = d;
            swing_leg_group = leg_id_B;
            stance_leg_group = leg_id_A;
            //body interp(constant velocity stage)
            bodypos[0]=beginbodypos[0]+(d/2)*((period_count+1)*1.0/totalcount);
        }
        else
        {
            current_step_len = d;
            swing_leg_group = leg_id_A;
            stance_leg_group = leg_id_B;
            //body interp(constant velocity stage)
            bodypos[0]=beginbodypos[0]+(d/2)*((period_count+1)*1.0/totalcount);
        }
        
        for (int i = 0; i < 3; i++)
        {
            int leg_id = swing_leg_group[i];
            footpos[leg_id*3+0]=beginfootpos[leg_id*3+0]+current_step_len*(1-std::cos(s))/2.0;
            footpos[leg_id*3+1]=beginfootpos[leg_id*3+1]+h*std::sin(s);  
            footpos[leg_id*3+2]=beginfootpos[leg_id*3+2];  
        }
        for (int i = 0; i < 3; i++)
        {
            int leg_id = stance_leg_group[i];
            footpos[leg_id*3+0]=beginfootpos[leg_id*3+0]; 
            footpos[leg_id*3+1]=beginfootpos[leg_id*3+1]; 
            footpos[leg_id*3+2]=beginfootpos[leg_id*3+2]; 
        }

        std::cout << count << " ";
        for (int i = 0; i < 3; i++)
        {
            std::cout << bodypos[i] << " ";
        }
        for (int i = 0; i < 18; i++)
        {
            std::cout << footpos[i] << " ";
        }
        std::cout << std::endl;
    }
}

void calculate_coe(double* t, double* x, double* v, double* coe)
{
    double t1 = t[0];
    double t2 = t[1];
    double x1 = x[0];
    double x2 = x[1];
    double v1 = v[0];
    double v2 = v[1];
    double dt = t1 - t2;
    coe[0] = ((t1 - t2) * (v1 + v2) - 2 * x1 + 2 * x2)/dt/dt/dt;
    coe[1] = (-t1*t1 * (v1 + 2 * v2) + t1 * (-t2 * v1 + t2 * v2 + 3 * x1 - 3 * x2) +  t2 * (2 * t2 * v1 + t2 * v2 + 3 * x1 - 3 * x2))/dt/dt/dt;
    coe[2] = (1/dt/dt/dt) * (-t2*t2*t2 * v1 + t1*t1*t1 * v2 + t1*t1 * t2 * (2 * v1 + v2) -   t1 * t2 * (t2 * v1 + 2 * t2 * v2 + 6 * x1 - 6 * x2));
    coe[3] = (1/dt/dt/dt) * (t2 * (t1 * (-t1 + t2) * (t2 * v1 + t1 * v2) - t2 * (-3 * t1 + t2) * x1) + t1*t1 * (t1 - 3 * t2) * x2);
}