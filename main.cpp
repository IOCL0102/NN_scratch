#include <stdio.h>
#include <stdlib.h>
#include<fstream>
#include<vector>
#include<iostream>
#include "TF.h"
#include "PID.h"
using namespace std;


/* Controller parameters */
#define PID_KP  0.988f
#define PID_KI  0.0175f
#define PID_KD  0.75f

#define PID_TAU 0.02f

#define PID_LIM_MIN -20.0f
#define PID_LIM_MAX  20.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 0.01f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 4.0f

/* Simulated dynamical system (first order) */
float TestSystem_Update(float inp);


int main() {
    double setpoint = 10.0f;

    ofstream fout("output.txt");
    double dt = 0.01;
    double t = 14.0;
    double inputd_init = 0, y_init = 0, yd_init = 0;
    double output = 0 , measurement = 0;


    TF <MyModel> tf(0, inputd_init, y_init, yd_init, dt);
    PID_Controller pid;
    pid.init(PID_KP, PID_KI, PID_KD, PID_LIM_MIN, PID_LIM_MAX,
        PID_LIM_MIN_INT, PID_LIM_MAX_INT, SAMPLE_TIME_S, PID_TAU);



    cout << "Time (s),System Output,ControllerOutput\n";
    fout << "Time (s),System Output,ControllerOutput\n";
    for (double i = 0.0; i < t; i += dt) {
        //if (i>1.0)
        //    output = 5.0;
        // measurement = tf.differntial_approach(output);
        measurement = TestSystem_Update(output);
        output = pid.update(setpoint, measurement);

        //cout << i << "," << output << "," << measurement  << endl;
        fout << i << "," << output  << "," << measurement << endl;
    }
    fout.close();
}


float TestSystem_Update(float inp) {

    static float output = 0.0f;
    static const float alpha = 0.02f;

    output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);

    return output;
}