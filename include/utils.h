//
// Created by Luke Calkins on 7/7/25.
//

#ifndef UTILS_H
#define UTILS_H
#include <armadillo>
#include <fstream>
void print_arma_vec(const arma::vec &v, std::ofstream &fstream) {
    for (arma::uword i = 0; i < v.n_elem; ++i) {
        fstream << v(i) << ", "; // Print each element followed by a space
    }
    fstream << std::endl;
}
#endif //UTILS_H
