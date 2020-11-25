#ifndef SL_ECS_H
#define SL_ECS_H

#include <inttypes.h>

#include <SLLight.h>
#include <SLMat4.h>
#include <SLNode.h>

namespace ECS
{

#define MAX_ENTITY_COUNT 100

typedef int32_t  entity_id;
typedef uint32_t bool32;

enum ComponentType
{
    ComponentType_None      = 0,
    ComponentType_TreeNode  = 1,
    ComponentType_Transform = 2,
    ComponentType_Light     = 4,
    ComponentType_Render    = 8
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

struct RenderComponent
{
    uint32_t programId;
    uint32_t vaoId;

    SLCol4f  ambient, diffuse, specular, emissive;
    float    shininess;
    uint32_t texId;
    SLMat4f  texMat;

    SLGLPrimitiveType primitiveType;
    uint32_t          numIndexes;
    SLGLBufferType    indexDataType;
    uint32_t          indexOffset;
};

struct LightComponent
{
    bool32 isOn;

    SLVec4f ambient;
    SLVec4f diffuse;
    SLVec4f specular;

    float spotCutoff;
    float spotCosCut;
    float spotExp;

    float  kc, kl, kq;
    bool32 isAttenuated;
};

struct World
{
    int                entityCount;
    Entity             entities[MAX_ENTITY_COUNT];
    TreeNodeComponent  treeNodeComponents[MAX_ENTITY_COUNT];
    TransformComponent transformComponents[MAX_ENTITY_COUNT];
    LightComponent     lightComponents[MAX_ENTITY_COUNT];
    RenderComponent    renderComponents[MAX_ENTITY_COUNT];
};

void convertToComponents(SLNode* root, World& world, SLVLight* lights, entity_id parentNodeId = -1);
void convertToNodes(World& world);

// entity adders
entity_id addEntity(World& world);
void      addTreeNodeComponent(World&    world,
                               entity_id entityId,
                               entity_id parentNodeId,
                               SLMat4f   om);
void      addLightComponent(World&    world,
                            entity_id entityId,
                            SLVec4f   ambient,
                            SLVec4f   diffuse,
                            SLVec4f   specular,
                            float     spotCutoff,
                            float     spotCosCut,
                            float     spotExp,
                            float     kc,
                            float     kl,
                            float     kq,
                            bool32    isAttenuated);
void      addRenderComponent(World&            world,
                             entity_id         entityId,
                             uint32_t          programId,
                             uint32_t          vaoId,
                             SLCol4f           ambient,
                             SLCol4f           diffuse,
                             SLCol4f           specular,
                             SLCol4f           emissive,
                             float             shininess,
                             uint32_t          texId,
                             SLMat4f           texMat,
                             SLGLPrimitiveType primitiveType,
                             uint32_t          numIndexes,
                             SLGLBufferType    indexDataType,
                             uint32_t          indexOffset);

// systems
void transformUpdateSystem(World& world);
void renderSystem(World&  world,
                  SLCol4f globalAmbient,
                  SLfloat gamma,
                  SLMat4f viewMat,
                  SLMat4f projectionMat);

};

#endif