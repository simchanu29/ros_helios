//
// Created by simon on 19/06/18.
//

#pragma once

#include "vector"
#include "algo.h"
#include "geodesy/utility.h"

class LineFollow: public Algo {
public:
    LineFollow();
    double run(std::vector<double> wp_a, std::vector<double> wp_b, std::vector<double> state_vec) override;
};
