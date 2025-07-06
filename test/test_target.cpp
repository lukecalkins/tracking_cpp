//
// Created by Luke Calkins on 7/5/25.
//

#include <armadillo>
#include "target.h"
#include "json.hpp"
using json = nlohmann::json;


int main(void) {

    std::string sim_json = "/Users/lukecalkins/CLionProjects/Tracking/config/sim.json";

    std::ifstream f(sim_json);
    json data = json::parse(f);
    std::cout << data << std::endl;

    std::string target_config_file = data["target_file"];
    std::ifstream g(target_config_file);
    json target_data = json::parse(g);
    std::cout << target_data << std::endl;

    double sampling_period = data["sampling_period"];
    std::cout << "Sampling Period: " << sampling_period << std::endl;

    arma::mat A(4, 4, arma::fill::eye);
    A(0, 2) = sampling_period;
    A(1, 3) = sampling_period;

    arma::mat W(4, 4, arma::fill::eye);

    arma::vec x_init = {0, 0, 1, 1};

    std::cout << "A = " << A << std::endl;
    std::cout << "W = " << W << std::endl;
    std::cout << "x_init = " << std::endl << x_init << std::endl;

    Target2DLinear target(0, x_init, A, W);

    int num_steps = data["steps"];
    for (int i=0; i < num_steps; i++) {
        target.forward_simulate(1);
        std::cout << "new state = " << std::endl << target.get_state() << std::endl;
    }


}