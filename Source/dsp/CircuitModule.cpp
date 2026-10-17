#include "CircuitModule.h"

#include <cmath>

namespace urei1176
{
void CircuitModule::prepare (double sampleRate)
{
    circ = ar::circuit::Circuit();
    std::vector<float> oldTargets;
    oldTargets.reserve (params.size());
    for (const auto& param : params)
        oldTargets.push_back (param.target);
    params.clear();

    build (sampleRate);
    built = true;

    // Keep the control positions across a rebuild (a sample-rate change).
    for (size_t i = 0; i < params.size() && i < oldTargets.size(); ++i)
        params[i].target = oldTargets[i];

    circ.prepare (sampleRate);
    reset();
}

void CircuitModule::reset()
{
    if (! built)
        return;
    for (auto& param : params)
    {
        param.current = param.target;
        param.apply (param.current);
    }
    circ.refactorIfNeeded();
    circ.findOperatingPoint();
}

int CircuitModule::addParam (float initial, float step, std::function<void (float)> apply)
{
    params.push_back ({ step, std::move (apply), initial, initial });
    return (int) params.size() - 1;
}

void CircuitModule::setTarget (int index, float value) noexcept
{
    if (index >= 0 && index < (int) params.size())
        params[(size_t) index].target = value;
}

void CircuitModule::controlTick() noexcept
{
    if (! built)
        return;
    bool changed = false;
    for (auto& param : params)
    {
        const float want = param.target;
        float& cur = param.current;
        if (cur == want)
            continue;
        const float step = param.step;
        if (step > 0.0f && std::abs (want - cur) > step)
            cur += want > cur ? step : -step;
        else
            cur = want;
        param.apply (cur);
        changed = true;
    }
    if (changed)
        circ.refactorIfNeeded();
}
} // namespace urei1176
