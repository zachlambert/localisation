#ifndef STEP_H
#define STEP_H

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
        bool step_finished = ((static_cast<T*>(this))->*steps[step_number])();
        if (step_finished) {
            step_number++;
        }
        return step_number == steps.size();;
    }
    void stepRemaining()
    {
        while (!step()) {}
    }
protected:
    void start()
    {
        step_number = 0;
    }
    void addStep(step_t step)
    {
        steps.push_back(step);
    }

    int step_number;
    std::vector<step_t> steps;
};


#endif
