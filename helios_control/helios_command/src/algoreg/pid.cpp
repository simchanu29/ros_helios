//
// Created by simon on 20/06/18.
//

#include <numeric>
#include "pid.h"

PID::PID(double p, double i, double d, int filterDepth, int intDepth) {
    this->p = p;
    this->i = i;
    this->d = d;
    this->filterDepth = filterDepth;
    this->intDepth = intDepth;
    integratedVal = 0;
    derivatedVal = 0;
}

PID::PID(double p, double i, double d): PID::PID(p, i, d, 3, 100){};

double PID::run(double target, double value) {
    // Handle memory
    if(mem.size()<filterDepth){
        mem.insert(mem.begin(),target-value);
    }
    else{
        const double smoothVal = std::accumulate(mem.begin(), mem.end(), 0.0)/mem.size();
        mem.insert(mem.begin(), target-smoothVal);
        mem.erase(mem.end());
    }
    const double smoothError = mem[0];

    // compute derivatedVal
    if(mem.size()>=filterDepth){
        const double derivatedValAfter = mem[0];
        const double derivatedValBefore = mem[1];
        derivatedVal = derivatedValAfter - derivatedValBefore;
    }

    // compute integratedVal
    integratedVal += mem[0];

    // compute command
    const double u = p*smoothError + i*integratedVal + d*derivatedVal;

    return u;
}
