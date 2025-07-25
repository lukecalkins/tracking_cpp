//
// Created by Luke Calkins on 7/7/25.
//

#ifndef UTILS_H
#define UTILS_H
#include <armadillo>
#include <fstream>
void print_arma_vec(const arma::vec &v, std::ofstream &fstream) {
    for (arma::uword i = 0; i < v.n_elem; ++i) {
        fstream << v(i);
        if (i == v.n_elem - 1) {
            break; // don't print comma after last element
        }
        fstream << ", ";

    }
    fstream << std::endl;
}

void print_arma_mat(const arma::mat &m, std::ofstream &fstream) {
    
}
#endif //UTILS_H
