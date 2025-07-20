//
// Created by Luke Calkins on 7/5/25.
//

#ifndef TARGET_H
#define TARGET_H

#include <armadillo>

class Target {
    unsigned int ID;
    unsigned int x_dim;

public:
    Target(const unsigned int ID, const unsigned int x_dim): ID(ID), x_dim(x_dim){}

    virtual void forward_simulate(const int T) = 0;

    virtual arma::vec get_state() const = 0;

    unsigned int get_x_dim() const {
        return x_dim;
    }

    unsigned int get_ID() const {
        return ID;
    }

    virtual ~Target() = default;
};

class Target2DLinear :  public Target {

    arma::vec x_t;
    arma::mat A;
    arma::mat W;
public:
    Target2DLinear(const unsigned int ID, const arma::vec &x_init, const arma::mat &A, const arma::mat &W):
    Target(ID, 2), x_t (x_init), A(A), W(W){}

    void forward_simulate(const int T) override {
        arma::vec new_state = A * x_t;
        x_t = new_state;
    }

    arma::vec get_state() const override{
        return x_t;
    }

    ~Target2DLinear() override = default;
};
#endif //TARGET_H
