//
// Created by Luke Calkins on 6/5/25.
//


#ifndef SENSOR_H
#define SENSOR_H

#include <armadillo>
#include <cmath>
#include <random>
#include <cmath>

class Sensor {

public:
    unsigned int z_dim;

    Sensor(unsigned int z_dim): z_dim((z_dim)){};

    [[nodiscard]] virtual arma::vec observation_model(const arma::vec &x_t, const arma::vec &p_t) const = 0;

    virtual double get_measurement(const arma::vec &x_t, const arma::vec &p_t) = 0;

    virtual void get_jacobian(arma::mat & H, arma::mat & V, const arma::vec &x_t, const arma::vec &p_t) = 0;

    virtual ~Sensor() = default;
};


class BearingSensor final : public Sensor {

    const double b_sigma; // standard deviation of measurement noise
    std::normal_distribution<double> distribution;
    std::default_random_engine generator;
    //std::random_device rd;
    bool logging;

public:
    explicit BearingSensor(const double b_sigma, bool logging = false) : Sensor(1),
                                                   b_sigma(b_sigma),
                                                   distribution(0, b_sigma * M_PI / 180),
                                                   generator(std::random_device{}()),
                                                   logging(logging){}

    [[nodiscard]] arma::vec observation_model(const arma::vec &x_t, const arma::vec &p_t) const override {
        /*
         * x_t = target state [x1, x2, v1, v2]
         * p_t = ownship vector [x1, x2, heading]
         */
        arma::vec bearing = {std::atan2(x_t[1] - p_t[1], x_t[0] - p_t[0]) - p_t[0]};

        return bearing;
    }

    double get_measurement(const arma::vec &x_t, const arma::vec &p_t) override {

        arma::vec bearing = observation_model(x_t, p_t);
        const double noise_sample = distribution(generator);
        double measurement = bearing[0] + noise_sample;

        return measurement;

    };

    void get_jacobian(arma::mat &H, arma::mat & V, const arma::vec &x_t, const arma::vec &p_t) override {

        const double range_2 = std::pow(x_t[1] - p_t[1], 2.0) + std::pow(x_t[0] - p_t[0], 2.0);

        H(0, 0) = (p_t[1] - x_t[1]) / range_2;
        H(0, 1) = (x_t[0] - p_t[0]) / range_2;

        V(0, 0) = std::pow(b_sigma, 2);
    }

    ~BearingSensor() override = default;
};

#endif //SENSOR_H