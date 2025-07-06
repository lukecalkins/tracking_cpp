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
protected:
    std::ofstream logFile;
public:
    explicit Sensor(const std::string &output_log): logFile(output_log){}

    [[nodiscard]] virtual arma::vec observation_model(const arma::vec &x_t, const arma::vec &p_t) const = 0;

    virtual double get_measurement(const arma::vec &x_t, const arma::vec &p_t) = 0;

    virtual ~Sensor() = default;
};


class BearingSensor final : public Sensor {

    const double b_sigma; // standard deviation of measurement noise
    std::normal_distribution<double> distribution;
    std::default_random_engine generator;
    //std::random_device rd;
    bool logging;

public:
    explicit BearingSensor(const double b_sigma, std::string &output_log, bool logging = false) : Sensor(output_log),
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

        if (logging) {
            //std::cout << "Logging measurement: " << measurement << std::endl;
            if (logFile.is_open()){
                logFile << "Measurement: " << measurement << std::endl;
            } else {
                std::cerr << "Error: Unable to open log file." << std::endl;
            }

        }

        return measurement;

    };

    ~BearingSensor() override = default;
};

#endif //SENSOR_H