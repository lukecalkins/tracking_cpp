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
    std::vector<arma::vec> ownships;
    ownships.push_back(ownship);
    const arma::vec ownship_2 =  {50, 0, 0};
    ownships.push_back(ownship_2);
    std::vector<arma::vec> measurements;

    for (unsigned int i = 0; i < p.get_num_steps(); i++) {
        target->forward_simulate(p.get_sampling_period());
        for (auto item : ownships) {
            measurements.push_back(sensor->get_measurement(target->get_state(), item));
        }
        tracker->update_belief(measurements, ownships);
        if (p.is_logging_enabled()) {
            arma::vec target_state = tracker->get_target_belief_state();
            arma::mat target_cov = tracker->get_target_belief_cov();
            p.write_to_log(measurements, target->get_state(), ownships, target_state, target_cov);
            std::cout << "wrote log entry" << std::endl;
        }
        measurements.clear();
    }

    return EXIT_SUCCESS;
    // TIP See CLion help at <a href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>. Also, you can try interactive lessons for CLion by selecting 'Help | Learn IDE Features' from the main menu.
}