

class PID_Controller {

public:

    struct pidVar {
        double prev = 0.0;    // previous value
        double curr = 0.0;    // current value
    };

    double Kp, Ki, Kd;
    double P, I, D;    // Propotional, Integrator and diffrentiator
    double outputMinLimit, outputMaxLimit;
    double integratorMinLimit, integratorMaxLimit;
    double sampleTime, tau;

    pidVar measurement;
    pidVar error;

    void init(double Kp, double Ki, double Kd, double outputMinLimit, double outputMaxLimit,
        double integratorMinLimit, double integratorMaxLimit, double sampleTime, double tau) {
        P = I = D = 0.0;

        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        this->outputMinLimit = outputMinLimit;
        this->outputMaxLimit = outputMaxLimit;
        this->integratorMinLimit = integratorMinLimit;
        this->integratorMaxLimit = integratorMaxLimit;

        this->sampleTime = sampleTime;
        this->tau = tau;
    }


    bool saturationCheck(double& input, double minLimit, double maxLimit) {
        if (input > maxLimit) {
            input = maxLimit;
            return true;
        }
        else if (input < minLimit) {
            input = minLimit;
            return true;
        }
        return false;
    }


    double update(double setPoint, double input) {
        error.curr = setPoint - input;
        measurement.curr = input;

        P = Kp * error.curr;
        I = Ki * sampleTime * 0.5 * (error.curr + error.prev) + I;

        D = (2.0 * Kd * (measurement.curr - measurement.prev) + ((2.0 * tau) - sampleTime) * D)
            / (2 * tau + sampleTime);
        saturationCheck(I, integratorMinLimit, integratorMaxLimit);


        double output = P + I + D;
        saturationCheck(output, outputMinLimit, outputMaxLimit);

        error.prev = error.curr;       // previous errorTerm = current errorTerm
        measurement.prev = measurement.curr;

        return output;
    }
};

