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

using json = nlohmann::json;

class Parameters {

    std::string sim_json;
    std:: string outputFilename;
    std::ofstream outputFile;
    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<Target> target;
    bool logging;
    unsigned int num_steps;
    double sampling_period;

public:
    explicit Parameters(std::string &sim_json): sim_json(sim_json) {

        std::ifstream f(sim_json);
        json data = json::parse(f);

        //Parse top level file for sensor data

        std::cout << data["sensor_file"] << std::endl;
        std::string sensor_config_file = data["sensor_file"];
        std::ifstream g(sensor_config_file);
        json sensor_data = json::parse(g);

        //Parse sensor file
        std::cout<< sensor_data << std::endl;
        std::cout << "Sensor Noise = " << sensor_data["noise"] << std::endl;
        std::cout << "Sensor Type = " << sensor_data["sensor type"] << std::endl;

        //Parse target file
        std::string target_config_file = data["target_file"];
        std::ifstream h(target_config_file);
        json target_data = json::parse(h);
        std::cout << target_data << std::endl;

        bool logging_enabled = data["logging"];
        outputFilename = data["output_log"];
        num_steps = data["steps"];
        sampling_period = data["sampling_period"];

        logging = logging_enabled;
        sensor = build_sensor(sensor_data, data);
        target = build_target(target_data, data);

        if (logging) {
            outputFile.open(outputFilename, std::ios::out);
        }
    }

    void write_to_log(const double &measurement, const arma::vec &target_state, const arma::vec &ownship_state) {
        if (outputFile.is_open()) {
            outputFile << "Measurement: " << measurement << std::endl;
            outputFile << "target: ";
            print_arma_vec(target_state, outputFile);
            outputFile << "ownship: ";
            print_arma_vec(ownship_state, outputFile);
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
            arma::mat A(4, 4, arma::fill::eye);
            A(0, 2) = sampling_period;
            A(1, 3) = sampling_period;

            arma::mat W(4, 4, arma::fill::eye);

            arma::vec x_init(4);

            std::cout << target_json["initial_state"] << std::endl;
            std::cout << typeid(target_json["initial_state"]).name() << std::endl;
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
