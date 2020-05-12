#include "ErlebAR.h"

namespace ErlebAR
{

const char* mapLocationIdToName(LocationId id)
{
    switch (id)
    {
        case LocationId::NONE:
            return "Undefined location";
        case LocationId::AUGST:
            return "Augst";
        case LocationId::AVENCHES:
            return "Avenches";
        case LocationId::BIEL:
            return "Biel";
        case LocationId::CHRISTOFFEL:
            return "Christoffelturm";
        default:
            return "Missing id to name mapping!";
    }
}

const char* mapAreaIdToName(AreaId id)
{
    switch (id)
    {
        case AreaId::NONE:
            return "Undefined location";
        //augst
        case AreaId::AUGST_TEMPLE_HILL_MARKER:
            return "Temple-Hill";
        //avenches
        case AreaId::AVENCHES_ARENA:
            return "Arena";
        //christoffel
        case AreaId::CHRISTOFFEL_SBB:
            return "Sbb";
        //biel
        case AreaId::BIEL_GERECHTIGKEITSBRUNNEN:
            return "Gerechtigkeitsbrunnen";
        case AreaId::BIEL_JACOB_ROSINUS:
            return "Jacob-Rosinus";
        case AreaId::BIEL_LEUBRINGENBAHN:
            return "Leubringenbahn";
        case AreaId::BIEL_RING:
            return "Ring";
        case AreaId::BIEL_SOUTHWALL:
            return "Southwall";
        default:
            return "Missing id to name mapping!";
    }
}

const Location defineLocationAugst()
{
    Location loc;
    loc.id                   = LocationId::AUGST;
    loc.name                 = mapLocationIdToName(loc.id);
    loc.areaMapImageFileName = "locations/augst/locationMapImgAugst.jpg";
    {
        Area area;
        area.id            = AreaId::AUGST_TEMPLE_HILL_MARKER;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 50;
        area.yPosPix       = 50;
        area.viewAngleDeg  = 0;
        loc.areas[area.id] = area;
    }
    return loc;
}
const Location defineLocationAvenches()
{
    Location loc;
    loc.id                   = LocationId::AVENCHES;
    loc.name                 = mapLocationIdToName(loc.id);
    loc.areaMapImageFileName = "locations/avenches/locationMapImgAvenches.jpg";
    {
        Area area;
        area.id            = AreaId::AVENCHES_ARENA;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 50;
        area.yPosPix       = 50;
        area.viewAngleDeg  = 0;
        loc.areas[area.id] = area;
    }
    return loc;
}
const Location defineLocationChristoffel()
{
    Location loc;
    loc.id                   = LocationId::CHRISTOFFEL;
    loc.name                 = mapLocationIdToName(loc.id);
    loc.areaMapImageFileName = "locations/christoffel/locationMapImgChristoffel.jpg";
    {
        Area area;
        area.id            = AreaId::CHRISTOFFEL_SBB;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 50;
        area.yPosPix       = 50;
        area.viewAngleDeg  = 0;
        loc.areas[area.id] = area;
    }
    return loc;
}

const Location defineLocationBiel()
{
    Location loc;
    loc.id                   = LocationId::BIEL;
    loc.name                 = mapLocationIdToName(loc.id);
    loc.areaMapImageFileName = "locations/biel/locationMapImgBiel.jpg";
    {
        Area area;
        area.id            = AreaId::BIEL_GERECHTIGKEITSBRUNNEN;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 606;
        area.yPosPix       = 277;
        area.viewAngleDeg  = 10.f;
        loc.areas[area.id] = area;
    }
    {
        Area area;
        area.id            = AreaId::BIEL_JACOB_ROSINUS;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 1387;
        area.yPosPix       = 730;
        area.viewAngleDeg  = 25.f;
        loc.areas[area.id] = area;
    }
    {
        Area area;
        area.id            = AreaId::BIEL_LEUBRINGENBAHN;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 606;
        area.yPosPix       = 50;
        area.viewAngleDeg  = 60.f;
        loc.areas[area.id] = area;
    }
    {
        Area area;
        area.id            = AreaId::BIEL_RING;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 200;
        area.yPosPix       = 200;
        area.viewAngleDeg  = 110.f;
        loc.areas[area.id] = area;
    }
    {
        Area area;
        area.id            = AreaId::BIEL_SOUTHWALL;
        area.name          = mapAreaIdToName(area.id);
        area.xPosPix       = 250;
        area.yPosPix       = 250;
        area.viewAngleDeg  = 270.f;
        loc.areas[area.id] = area;
    }
    return loc;
}

const std::map<LocationId, Location> defineLocations()
{
    std::map<LocationId, Location> locations;
    locations[LocationId::AUGST]       = defineLocationAugst();
    locations[LocationId::AVENCHES]    = defineLocationAvenches();
    locations[LocationId::CHRISTOFFEL] = defineLocationChristoffel();
    locations[LocationId::BIEL]        = defineLocationBiel();

    return locations;
}
};
