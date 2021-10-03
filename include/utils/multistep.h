#ifndef MULTISTEP_H
#define MULTISTEP_H

#include <string>
#include <vector>
#include <functional>


class Multistep {
    // step_t is a function pointer to a method of class T
    typedef std::function<bool()> step_t;

public:
    bool step()
    {
        if (step_number >= steps.size()) return true;

        bool current_step_done = steps[step_number]();

        if (current_step_done) {
            step_number++;
            if (step_number == steps.size()) {
                is_started = false;
                return true;
            }
        }

        return false;
    }
    void stepRemaining()
    {
        while (!step()) {}
    }

    bool started()
    {
        return is_started;
    }
    const std::string& stepName()
    {
        return step_names[step_number];
    }
protected:
    void start()
    {
        step_number = 0;
        is_started = true;
    }
    void addStep(step_t step, const std::string& step_name)
    {
        steps.push_back(step);
        step_names.push_back(step_name);
    }

    int step_number = 0;
    std::vector<step_t> steps;
    std::vector<std::string> step_names;
    bool is_started = false;
};

#endif
