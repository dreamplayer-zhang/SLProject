#include "SENSOrientation.h"
#include <Utils.h>

SENSOrientation::Quat SENSOrientation::getOrientation()
{
    const std::lock_guard<std::mutex> lock(_orientationMutex);
    return _orientation;
}

void SENSOrientation::setOrientation(SENSOrientation::Quat orientation)
{
    //estimate time before running into lock
    SENSTimePt timePt = SENSClock::now();

    {
        const std::lock_guard<std::mutex> lock(_orientationMutex);
        _orientation = orientation;
        _timePt      = timePt;
    }

    {
        std::lock_guard<std::mutex> lock(_listenerMutex);
        for (SENSOrientationListener* l : _listeners)
            l->onOrientation(timePt, orientation);
    }
}

void SENSOrientation::registerListener(SENSOrientationListener* listener)
{
    std::lock_guard<std::mutex> lock(_listenerMutex);
    if (std::find(_listeners.begin(), _listeners.end(), listener) == _listeners.end())
        _listeners.push_back(listener);
}
void SENSOrientation::unregisterListener(SENSOrientationListener* listener)
{
    std::lock_guard<std::mutex> lock(_listenerMutex);
    for (auto it = _listeners.begin(); it != _listeners.end(); ++it)
    {
        if (*it == listener)
        {
            _listeners.erase(it);
            break;
        }
    }
}
