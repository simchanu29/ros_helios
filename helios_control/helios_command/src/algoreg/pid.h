//
// Created by simon on 20/06/18.
//

#pragma once

#include <vector>

class PID {
public:
    PID(double p, double i, double d, int filterDepth, int intDepth);
    PID(double p, double i, double d): PID(p, i, d, 3, 100);
    double run(double target, double value);
    double p;
    double i;
    double d;
    double filterDepth;
private:
    std::vector<double> mem;
    double integratedVal;
    double derivatedVal;
};