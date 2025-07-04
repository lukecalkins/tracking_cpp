//
// Created by Luke Calkins on 5/29/25.
//


#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include<vector>
#include "armadillo"

class KalmanFilter {

    arma::vec x_t;
    arma::mat Sigma_t;
    arma::mat A;
    arma::mat C;

public:
    KalmanFilter(arma::vec x0,
                 arma::mat Sigma_0,
                 arma::mat A,
                 arma::mat C): x_t(x0),
                                       Sigma_t(Sigma_0),
                                       A(A),
                                       C(C)
                                        {}

    arma::vec get_mean() const {
        return x_t;
    }

    arma::mat get_cov() const {
        return Sigma_t;
    }

};
#endif //KALMANFILTER_H
