#pragma once
#include<vector>
#include<iostream>
using namespace std;

class Model {
public:
	virtual void init(double input, double inputd_init, double y_init, double yd_init, double sampleTime)=0;
	virtual void update(double input) = 0;
    virtual double getOutput() = 0;
};

class MyModel : public Model {
public:
    double dt;
    double u_prev, ud;
    double y, yd_curr, yd_prev, ydd;

    MyModel() {
        cout << "MyModel Constructor called " << endl;
        dt = 0.0;
        u_prev = ud = 0.0;
        y = 0.0;
        yd_curr = yd_prev = 0.0;
        ydd = 0.0;
    }

    void init(double input, double inputd_init, double y_init, double yd_init, double sampleTime) {
        // input put as 0 when call this init() function, because commonly the initial state is start at 0
        dt = sampleTime;
        ud = inputd_init;
        yd_prev = yd_init;
        y = y_init;
        ydd = ud + 2 * input - yd_prev - 2 * y;
    }

    void update(double input) {

        ud = (input - u_prev) / dt;
        yd_curr = yd_prev + ydd * dt;
        y = y + yd_prev * dt;
        ydd = ud + 2 * input - yd_curr - 2 * y;

        u_prev = input;
        yd_prev = yd_curr;
    }

    double getOutput(){
        return y;
    }
};


template <class T>
class TF {
private:
    Model* model;
public:
    TF( double input, double inputd_init, double y_init, double yd_init, double sampleTime ) { 
        model = new T;
        model->init(input, inputd_init, y_init, yd_init, sampleTime); 
    }

     double differntial_approach (double input) {
        model->update(input);
        return model->getOutput();
	}

    ~TF(){
        delete model;
    }
};