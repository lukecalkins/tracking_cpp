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

    bool exit_loops = false;
    for (arma::uword i = 0; i < m.n_rows; i++) {
        for (arma::uword j = 0; j < m.n_cols; j++) {
            fstream << m(i,j);
            if (i == m.n_rows -1 && j == m.n_cols -1) {
                break;
            }
            fstream << ", ";
        }
    }
    fstream << std::endl;
}
#endif //UTILS_H
