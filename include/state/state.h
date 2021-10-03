#ifndef SIM_H
#define SIM_H

#include "algorithm/controller.h"
#include "algorithm/state_estimator.h"
#include "maths/geometry.h"
#include "maths/motion_model.h"
#include "state/sim.h"
#include "utils/step.h"

#include <iostream>


struct State: public Step<State> {

    double dt;
    Sim& sim;
    StateEstimator& state_estimator;
    Controller& controller;

    State(  Sim& sim,
            StateEstimator& state_estimator,
            Controller& controller):
        sim(sim),
        state_estimator(state_estimator),
        controller(controller)
    {
        addStep(&State::stepSim);
        addStep(&State::stepStateEstimator);
        addStep(&State::stepController);
    }

    void start(double dt)
    {
        this->dt = dt;
        step_number = 0;
        Step::start();
    }

    bool stepSim()
    {
        sim.step(dt, controller.command);
        return true;
    }

    bool stepStateEstimator()
    {
        if (!state_estimator.started()) {
            state_estimator.start(
                &sim.robot.twistEstimate,
                &sim.range_sensor.features,
                &sim.terrain
            );
        }
        return state_estimator.step();
    }

    bool stepController()
    {
        if (!controller.started()) {
            controller.start(state_estimator.getStateEstimate());
        }
        return controller.step();
    }
};

#endif
