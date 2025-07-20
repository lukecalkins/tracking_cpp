//
// Created by Luke Calkins on 7/19/25.
//

#ifndef TRACKER_H
#define TRACKER_H

#include "target.h"
#include "targetBelief.h"
#include "sensor.h"


class Tracker {

    TargetLinear2DBelief infoTarget;
    std::shared_ptr<Sensor> sensor;

public:

    Tracker(TargetLinear2DBelief init_target_estimate, std::shared_ptr<Sensor> sensor) : infoTarget((init_target_estimate)), sensor(sensor){};

    virtual void update_belief(double measurement) = 0;

    virtual ~Tracker() = default;

};


class KalmanFilter final : public Tracker {

public:

    KalmanFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor): Tracker(init_target_belief, sensor){};

    void update_belief(double measurement) override {
        std::cout << "Inside update belief" << std::endl;
    };

    ~KalmanFilter() override = default;
};

class UnscentedKalmanFilter final : public Tracker {

public:

    UnscentedKalmanFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor) : Tracker(init_target_belief, sensor){};

    void update_belief(double measurement) override;

    ~UnscentedKalmanFilter() override = default;
};
#endif //TRACKER_H
