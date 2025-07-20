//
// Created by Luke Calkins on 6/28/25.
//


#include <stdlib.h>
#include <string>
#include "params.h"

int main() {

    std::string sim_json = "/Users/lukecalkins/CLionProjects/Tracking/config/sim.json";

    Parameters p(sim_json);

    return EXIT_SUCCESS;
}
