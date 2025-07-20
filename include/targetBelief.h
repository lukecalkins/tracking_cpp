//
// Created by Luke Calkins on 7/11/25.
//

#ifndef TARGETBELIEF_H
#define TARGETBELIEF_H
#include "target.h"
#include <armadillo>

class TargetLinear2DBelief : public Target2DLinear {

    arma::mat cov;

public:
    TargetLinear2DBelief(const unsigned int ID, const arma::vec &x_init, const arma::mat &A, const arma::mat &W, const arma::mat &cov):
                                                                Target2DLinear(ID, x_init, A, W), cov(cov){}
};
#endif //TARGETBELIEF_H
