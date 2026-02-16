//
// Created by Luke Calkins on 2/11/26.
//
#include "armadillo"

int main() {

    arma::mat A = arma::mat(2,4, arma::fill::zeros);

    arma::mat B = arma::mat(1,4, arma::fill::ones);

    std::cout << A << std::endl << B << std::endl;

    A.submat(0, 0, 0, 3) = B;

    std::cout  << A << std::endl;

    B(0, 0) = 2.0;

    std::cout << B << std::endl;

    std::cout << A << std::endl;

    return 0;
}