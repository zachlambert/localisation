#ifndef STEP_H
#define STEP_H

#include <string>
#include <vector>


template <typename T>
class Step {
    // step_t is a function pointer to a method of class T
    typedef bool (T::*step_t)();

public:
    // Returns true if this object has finished stepping
    bool step()
    {
        if (step_number >= steps.size()) return true;

        // Class T will extend Step<T>. Then, this static cast will compile.
        bool current_step_done = ((static_cast<T*>(this))->*steps[step_number])();

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
    void addStep(step_t step)
    {
        steps.push_back(step);
    }

    int step_number = 0;
    std::vector<step_t> steps;
    std::vector<std::string> step_names;
    bool is_started = false;
};


#endif
