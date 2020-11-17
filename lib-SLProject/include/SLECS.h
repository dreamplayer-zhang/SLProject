#ifndef SL_ECS_H
#define SL_ECS_H

#include <inttypes.h>
#include <SLMat4.h>

#define MAX_ENTITY_COUNT 100

enum SLComponentType
{
    ComponentType_None      = 0,
    ComponentType_Parent    = 1,
    ComponentType_Transform = 2
};

struct SLEntity
{
    uint32_t componentTypes;
};

struct SLComponentNodeTree
{
    uint32_t nodeId;
    uint32_t parentId;
};

struct SLComponentTransform
{
    SLMat4f om;
    SLMat4f wm;
};

struct SLWorld
{
    int                  entityCount;
    SLEntity             entities[MAX_ENTITY_COUNT];
    SLComponentNodeTree  nodeTreeComponents[MAX_ENTITY_COUNT];
    SLComponentTransform transformComponents[MAX_ENTITY_COUNT];
};

void transformUpdateSystem(SLWorld& world);

#endif