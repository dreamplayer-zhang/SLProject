#ifndef SENSOR_TEST_VIEW_H
#define SENSOR_TEST_VIEW_H

#include <string>
#include <SLInputManager.h>
#include <SLSceneView.h>
#include <SensorTestGui.h>
#include <ErlebAR.h>
#include <sens/SENSGps.h>
#include <sens/SENSOrientation.h>

class SensorTestView : public SLSceneView
{
public:
    SensorTestView(sm::EventHandler&   eventHandler,
                   SLInputManager&     inputManager,
                   const ImGuiEngine&  imGuiEngine,
                   ErlebAR::Resources& resources,
                   SENSGps*            sensGps,
                   SENSOrientation*    sensOrientation,
                   SENSCamera*         sensCamera,
                   const DeviceData&   deviceData);
    bool update();

    //call when view becomes visible
    void onShow() { _gui.onShow(); }
    void onHide();

private:
    SensorTestGui     _gui;
    SENSGps*          _gps         = nullptr;
    SENSOrientation*  _orientation = nullptr;
    const DeviceData& _deviceData;
};

#endif //SENSOR_TEST_VIEW_H
