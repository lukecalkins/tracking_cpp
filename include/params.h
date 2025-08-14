//
// Created by Luke Calkins on 6/28/25.
//

#ifndef PARAMS_H
#define PARAMS_H


#include <fstream>
#include <iostream>
#include <string>
#include "json.hpp"
#include "sensor.h"
#include "target.h"
#include "utils.h"
#include "tracker.h"

using json = nlohmann::json;

class Parameters {

    std::string sim_json;
    std:: string outputFilename;
    std::ofstream outputFile;
    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<Target> target;
    std::shared_ptr<Tracker> tracker;
    bool logging;
    unsigned int num_steps;
    double sampling_period;

public:
    explicit Parameters(std::string &sim_json): sim_json(sim_json) {

        //Load top level file
        std::ifstream f(sim_json);
        json data = json::parse(f);

        //Parse sensor file
        std::string sensor_config_file = data["sensor_file"];
        std::ifstream g(sensor_config_file);
        json sensor_data = json::parse(g);
        std::cout << "Sensor noise = " << sensor_data["noise"] << std::endl;
        std::cout << "Sensor type = " << sensor_data["sensor type"] << std::endl;

        //Parse target file
        std::string target_config_file = data["target_file"];
        std::ifstream h(target_config_file);
        json target_data = json::parse(h);
        std::cout <<  "Target type: " << target_data["target type"] << std::endl;

        //parse tracker file
        std::string tracker_config_file = data["tracker_file"];
        std::ifstream k(tracker_config_file);
        json tracker_data = json::parse(k);
        std::cout <<  "Tracker type: " << tracker_data["tracker type"] << std::endl;

        bool logging_enabled = data["logging"];
        outputFilename = data["output_log"];
        num_steps = data["steps"];
        sampling_period = data["sampling_period"];

        logging = logging_enabled;
        sensor = build_sensor(sensor_data, data);
        target = build_target(target_data, data);
        tracker = build_tracker(tracker_data, target_data, data);

        if (logging) {
            outputFile.open(outputFilename, std::ios::out);
        }
    }

    void write_to_log(const arma::vec &measurement, const arma::vec &target_state, const arma::vec &ownship_state,
        const arma::vec &target_belief_state, const arma::mat &target_belief_cov) {
        if (outputFile.is_open()) {
            outputFile << "Measurement: ";
            print_arma_vec(measurement, outputFile);
            outputFile << "target: ";
            print_arma_vec(target_state, outputFile);
            outputFile << "ownship: ";
            print_arma_vec(ownship_state, outputFile);
            outputFile << "target belief state: ";
            print_arma_vec(target_belief_state, outputFile);
            outputFile << "target belief covariance: ";
            print_arma_mat(target_belief_cov, outputFile);
        } else {
            std::cerr << "Error opening file" << std::endl;
        }
    }

    unsigned int get_num_steps() const {
        return num_steps;
    }

    double get_sampling_period() const {
        return sampling_period;
    }
    static void construct_process_noise_cov(arma::mat &W, const double accel_dist, const double sampling_period) {
        W(0, 0) = std::pow(sampling_period, 4.0) / 4.0;
        W(1, 1) = std::pow(sampling_period, 4.0) / 4.0;
        W(0, 2) = std::pow(sampling_period, 3.0) / 2.0;
        W(1, 3) = std::pow(sampling_period, 3.0) / 2.0;
        W(2, 0) = std::pow(sampling_period, 3.0) / 2.0;
        W(3, 1) = std::pow(sampling_period, 3.0) / 2.0;
        W(2, 2) = std::pow(sampling_period, 2.0);
        W(3, 3) = std::pow(sampling_period, 2.0);

        W = std::pow(accel_dist, 2) * W;
    }

    static void construct_state_transition_mat(arma::mat &A, const double sampling_period) {

        // Assume A initialized outside this function as identity mat
        A(0, 2) = sampling_period;
        A(1, 3) = sampling_period;
    }

    static void construct_initial_target_covariance(arma::mat &Sigma, const double init_cov_pos, const double init_cov_vel) {
        Sigma(0, 0) = init_cov_pos;
        Sigma(1, 1) = init_cov_pos;
        Sigma(2, 2) = init_cov_vel;
        Sigma(3, 3) = init_cov_vel;
    }
    std::shared_ptr<Tracker> build_tracker(const json &tracker_json, const json &target_json, const json &sim_json) {
        std::shared_ptr<Tracker> tracker;
        if (tracker_json["use_target_initial_state"]) {
            const double sampling_period = sim_json["sampling_period"];

            int target_dim = target_json["target_dim"];
            arma::mat A(target_dim, target_dim, arma::fill::eye);
            construct_state_transition_mat(A, sampling_period);
            std::cout << "Transition mat A in tracker: " << std::endl << A << std::endl;

            double accel_dist = target_json["sigma_a"];
            arma::mat W(target_dim, target_dim, arma::fill::zeros);
            construct_process_noise_cov(W, accel_dist, sampling_period);
            std::cout << "Noise mat W: " << std::endl << W << std::endl;

            arma::mat init_cov(target_dim, target_dim, arma::fill::zeros);
            double init_cov_pos = target_json["initial_cov_pos"];
            double init_cov_vel = target_json["initial_cov_vel"];
            construct_initial_target_covariance(init_cov, init_cov_pos, init_cov_vel);
            std::cout << "Initial target covariance: " << std::endl << init_cov << std::endl;

            arma::vec x_init(target_dim, 1);
            std::cout << "Initial target state in tracker: " << target_json["initial_state"] << std::endl;
            int i = 0;
            for (const auto& element: target_json["initial_state"]) {
                x_init(i) = static_cast<double>(element);
                i++;
            }

            //create target belief
            TargetLinear2DBelief init_info_target(1, x_init, A, W, init_cov);

            if (tracker_json["tracker_type"] == "EKF") {
                tracker = std::make_shared<KalmanFilter>(init_info_target, sensor);
            }
            else if (tracker_json["tracker_type"] == "UKF") {
                double alpha = tracker_json["alpha"];
                double beta = tracker_json["beta"];
                double kappa = tracker_json["kappa"];

                tracker = std::make_shared<UnscentedKalmanFilter>(init_info_target, sensor, alpha, beta, kappa);
            }

        }
        return tracker;
    }

    std::shared_ptr<Tracker> get_tracker() {
        return tracker;
    }

    static std::shared_ptr<Sensor> build_sensor(const json &sensor_json, const json &sim_json) {

        std::shared_ptr<Sensor> sensor;
        if (sensor_json["sensor type"] == "bearing") {
            double b_sigma = sensor_json["noise"];
            std::string output_log = sim_json["output_log"];
            bool logging = sim_json["logging"];
            sensor = std::make_shared<BearingSensor>(b_sigma);
        }

        return sensor;
    }

    std::shared_ptr<Sensor> get_sensor() {
        return sensor;
    }

    static std::shared_ptr<Target> build_target(const json &target_json, const json &sim_json) {

        std::shared_ptr<Target> target;
        if (target_json["target type"] == "linear_2d") {
            const double sampling_period = sim_json["sampling_period"];

            int target_dim = target_json["target_dim"];
            arma::mat A(target_dim, target_dim, arma::fill::eye);
            construct_state_transition_mat(A, sampling_period);
            std::cout << "Transition mat A in true target: " << std::endl << A << std::endl;


            //TODO add noise to state transition mat
            arma::mat W(target_dim, target_dim, arma::fill::eye);

            arma::vec x_init(target_dim, 1);
            std::cout << "Initial true target state: " << target_json["initial_state"] << std::endl;
            int i = 0;
            for (const auto& element: target_json["initial_state"]) {
                x_init(i) = static_cast<double>(element);
                i++;
            }
            target = std::make_shared<Target2DLinear>(0, x_init, A, W);
        }

        return target;
    }

    std::shared_ptr<Target> get_target() {
        return target;
    }

    bool is_logging_enabled() const {
        return logging;
    }


};

#endif //PARAMS_H
