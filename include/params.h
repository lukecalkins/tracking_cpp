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

using json = nlohmann::json;

class Parameters {

    std::string sim_json;
    std:: string output_path;
    std::shared_ptr<Sensor> sensor;

public:
    Parameters(std::string sim_json, std::string output_path): sim_json(sim_json), output_path(output_path) {

        std::ifstream f(sim_json);
        json data = json::parse(f);

        //std::string sensor_file = data['sensor_file'];
        std::cout << data["sensor_file"] << std::endl;

        //Parse top level file
        std::string sensor_config_file = data["sensor_file"];
        std::ifstream g(sensor_config_file);
        json sensor_data = json::parse(g);

        //Parse sensor file
        std::cout<< sensor_data << std::endl;
        std::cout << "Sensor Noise = " << sensor_data["noise"] << std::endl;
        std::cout << "Sensor Type = " << sensor_data["sensor type"] << std::endl;

        sensor = build_sensor(sensor_data);
    }

    static std::shared_ptr<Sensor> build_sensor(const json &sensor_json) {

        std::shared_ptr<Sensor> sensor;
        if (sensor_json["sensor type"] == "bearing") {
            sensor = std::make_shared<BearingSensor>(BearingSensor(sensor_json["s"]));
        }

        return sensor;
    }

    std::shared_ptr<Sensor> get_sensor() {
        return sensor;
    }
};

#endif //PARAMS_H
