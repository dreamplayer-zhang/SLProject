#include <SLECS.h>

ECS::entity_id ECS::addTreeNode(ECS::World& world,
                                entity_id   parentNodeId,
                                SLMat4f     om)
{
    ECS::entity_id nextFreeId = world.entityCount;

    world.entityCount++;

    world.entities[nextFreeId].componentFlags |= ECS::ComponentType_TreeNode;
    world.entities[nextFreeId].componentFlags |= ECS::ComponentType_Transform;
    world.treeNodeComponents[nextFreeId].parentNodeId = parentNodeId;
    world.transformComponents[nextFreeId].om          = om;

    return nextFreeId;
}

void ECS::convertToComponents(SLNode*     root,
                              ECS::World& world,
                              entity_id   parentNodeId)
{
    ECS::entity_id nodeId       = ECS::addTreeNode(world, parentNodeId, root->om());
    world.entities[nodeId].node = root;

    std::vector<SLNode*> children = root->findChildren<SLNode>("", false);

    for (SLNode* child : children)
    {
        ECS::convertToComponents(child, world, nodeId);
    }
}

void ECS::convertToNodes(ECS::World& world)
{
    for (int i = 0; i < world.entityCount; i++)
    {
        ECS::Entity& e = world.entities[i];
        if ((e.componentFlags & ComponentType_TreeNode) &&
            (e.componentFlags & ComponentType_Transform) &&
            e.node)
        {
            ECS::TransformComponent& tC = world.transformComponents[i];
            e.node->wm(tC.wm);
        }
    }
}

void ECS::transformUpdateSystem(ECS::World& world)
{
    int nodeIndices[MAX_ENTITY_COUNT];
    int nodeIndexCount   = 1;
    int currentNodeIndex = 0;
    nodeIndices[0]       = -1;

    while (currentNodeIndex < nodeIndexCount)
    {
        int currentParentIndex = nodeIndices[currentNodeIndex];
        currentNodeIndex++;

        for (int i = 0; i < world.entityCount; i++)
        {
            ECS::Entity& e = world.entities[i];
            if ((e.componentFlags & ComponentType_TreeNode) &&
                (e.componentFlags & ComponentType_Transform))
            {
                ECS::TreeNodeComponent& c = world.treeNodeComponents[i];
                if (c.parentNodeId == currentParentIndex)
                {
                    //printf("Tree node found with entity index %i for parent entity index %i\n", i, currentParentIndex);

                    ECS::TransformComponent& tC = world.transformComponents[i];
                    if (currentParentIndex < 0)
                        tC.wm = tC.om;
                    else
                        tC.wm = world.transformComponents[currentParentIndex].wm * tC.om;

                    bool nodeIndexInList = false;
                    for (int j = 0; j < nodeIndexCount; j++)
                    {
                        if (nodeIndices[j] == i)
                        {
                            nodeIndexInList = true;
                            break;
                        }
                    }

                    if (!nodeIndexInList)
                    {
                        nodeIndices[nodeIndexCount] = i;
                        nodeIndexCount++;
                    }
                }
            }
        }
    }

    //printf("End of system\n");
}