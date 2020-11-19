#ifndef SL_ECS_H
#define SL_ECS_H

#include <inttypes.h>
#include <SLMat4.h>
#include <SLNode.h>

namespace ECS
{

#define MAX_ENTITY_COUNT 100

typedef int32_t entity_id;

enum ComponentType
{
    ComponentType_None      = 0,
    ComponentType_TreeNode  = 1,
    ComponentType_Transform = 2
};

struct Entity
{
    uint32_t componentFlags;
    SLNode*  node;
};

struct TreeNodeComponent
{
    entity_id parentNodeId;
};

struct TransformComponent
{
    SLMat4f om;
    SLMat4f wm;
};

struct World
{
    int                entityCount;
    Entity             entities[MAX_ENTITY_COUNT];
    TreeNodeComponent  treeNodeComponents[MAX_ENTITY_COUNT];
    TransformComponent transformComponents[MAX_ENTITY_COUNT];
};

void convertToComponents(SLNode* root, World& world, entity_id parentNodeId = -1);
void convertToNodes(World& world);

// entity adders
entity_id addTreeNode(World&    world,
                      entity_id parentNodeId,
                      SLMat4f   om);

// systems
void transformUpdateSystem(World& world);

};

#endif