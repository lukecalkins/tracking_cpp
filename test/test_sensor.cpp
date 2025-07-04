//
// Created by Luke Calkins on 6/21/25.
//

#include <iostream>
#include <fstream>
#include <typeinfo>
#include <armadillo>
#include "sensor.h"
#include "json.hpp"

using json = nlohmann::json;

int main() {

    std::string sim_json = "/Users/lukecalkins/CLionProjects/Tracking/config/sim.json";

    std::ifstream f(sim_json);
    json data = json::parse(f);
    std::cout << data << std::endl;

    std::string sensor_config_file = data["sensor_file"];
    std::ifstream g(sensor_config_file);
    json sensor_data = json::parse(g);

    std::cout << sensor_data["noise"] << std::endl;
    std::cout << typeid(sensor_data["noise"]).name() << std::endl;
    double b_sigma = sensor_data["noise"];

    // parse output og
    std::string output_log = data["output_log"];
    std::cout << output_log << std::endl;
    BearingSensor sensor(b_sigma, output_log, true);

    arma::vec target = {0, 1, 0, 0};
    arma::vec ownship = {0, 0, 0};

    int num_steps = data["steps"];
    for (int i = 0; i < num_steps; i++) {
        sensor.get_measurement(target, ownship);
    }

    return EXIT_SUCCESS;
}