//
// Created by Luke Calkins on 7/7/25.
//

#ifndef UTILS_H
#define UTILS_H


#include <armadillo>
#include <fstream>
#include <gtsam/base/Matrix.h>

inline void print_arma_vec(const arma::vec &v, std::ofstream &fstream) {
    for (arma::uword i = 0; i < v.n_elem; ++i) {
        fstream << v(i);
        if (i == v.n_elem - 1) {
            break; // don't print comma after last element
        }
        fstream << ", ";

    }
    fstream << std::endl;
}

inline void print_multi_arma_vec(const std::vector<arma::vec> & v, std::ofstream &fstream) {
    for (auto item : v) {
        print_arma_vec(item, fstream);
    }
}

inline void print_arma_mat(const arma::mat &m, std::ofstream &fstream) {

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

inline void arma_mat_to_gtsam_mat(arma::mat &arma, gtsam::Matrix &gmat) {
    for (int i = 0; i < arma.n_rows; i++) {
        for (int j = 0; j < arma.n_cols; j++) {
            gmat(i, j) = arma(i, j);
        }
    }
}

inline void arma_vec_to_gtsam_vec(arma::vec &arma, gtsam::Vector &gvec) {
    for (int i = 0; i < arma.n_rows; i++) {
        gvec(i) = arma(i);
    }
}

/*
void arma_to_Vector(arma::vec &arma, gtsam::Vector &vec) {

}
*/

#endif //UTILS_H
