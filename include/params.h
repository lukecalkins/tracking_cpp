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

using json = nlohmann::json;

class Parameters {

    std::string sim_json;
    std:: string output_path;
    std::shared_ptr<Sensor> sensor;
    std::shared_ptr<Target> target;

public:
    Parameters(std::string &sim_json, std::string &output_path): sim_json(sim_json), output_path(output_path) {

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



        sensor = build_sensor(sensor_data, data);
        target = build_target(target_data, data);
    }

    static std::shared_ptr<Sensor> build_sensor(const json &sensor_json, const json &sim_json) {

        std::shared_ptr<Sensor> sensor;
        if (sensor_json["sensor type"] == "bearing") {
            double b_sigma = sensor_json["noise"];
            std::string output_log = sim_json["output_log"];
            bool logging = sim_json["logging"];
            sensor = std::make_shared<BearingSensor>(b_sigma, output_log, true);
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
            std::cout << x_init << std::endl;
        }

        return target;
    }

    std::shared_ptr<Target> get_target() {
        return target;
    }


};

#endif //PARAMS_H
