//
// Created by simon on 19/06/18.
//

#pragma once

#include <map>
#include <vector>

class Algo {
public:
    Algo();

    virtual double run(std::vector<double> wp_a, std::vector<double> wp_b, std::vector<double> state_vec)= 0;

    bool debug;
    std::map<std::string, double> debugMap;
};
