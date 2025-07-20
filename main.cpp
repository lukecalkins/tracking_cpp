#include <iostream>
#include "armadillo"
#include "tracker.h"
#include "json.hpp"
#include "sensor.h"
#include "target.h"
#include "params.h"
// TIP To <b>Run</b> code, press <shortcut actionId="Run"/> or click the <icon src="AllIcons.Actions.Execute"/> icon in the gutter.
int main() {

    std::string sim_file = "/Users/lukecalkins/CLionProjects/Tracking/config/sim.json";
    Parameters p(sim_file);

    std::shared_ptr<Sensor> sensor = p.get_sensor();
    std::shared_ptr<Target> target = p.get_target();
    std::shared_ptr<Tracker> tracker = p.get_tracker();
    const arma::vec ownship = {0, 0, 0};

    for (unsigned int i = 0; i < p.get_num_steps(); i++) {
        target->forward_simulate(p.get_sampling_period());
        arma::vec measurement = sensor->get_measurement(target->get_state(), ownship);
        tracker->update_belief(measurement, ownship);
        if (p.is_logging_enabled()) {
            //std::cout << "Logging measurement: " << measurement << std::endl
            p.write_to_log(measurement, target->get_state(), ownship);
            std::cout << "wrote log entry" << std::endl;
        }
    }

    return EXIT_SUCCESS;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}