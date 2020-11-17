#include <SLECS.h>

void transformUpdateSystem(SLWorld& world)
{
    struct TreeNode
    {
        uint32_t entityId;
        int      childCount;

        SLMat4f om;
        SLMat4f wm;
    };

    TreeNode treeNodes[MAX_ENTITY_COUNT];
    int      treeNodeCount = 0;

    // find root node
    for (int i = 0; i < world.entityCount; i++)
    {
        if ((world.entities[i].componentTypes & ComponentType_Parent) &&
            (world.entities[i].componentTypes & ComponentType_Transform))
        {
            if (world.nodeTreeComponents[i].parentId == -1)
            {
                treeNodes[0].entityId   = i;
                treeNodes[0].childCount = 0;
                treeNodes[0].om         = world.transformComponents[i].om;
                treeNodes[0].wm         = world.transformComponents[i].om;

                break;
            }
        }
    }

    // TODO: build rest of tree

    // if no root node, exit
    if (treeNodeCount <= 0) return;

    // bfs traversal of tree
    int parentIndex = 0;
    int childIndex  = 1;

    for (parentIndex = 0; parentIndex < treeNodeCount; parentIndex++)
    {
        TreeNode parentNode     = treeNodes[parentIndex];
        int      lastChildIndex = childIndex + parentNode.childCount;
        for (childIndex; childIndex < lastChildIndex; childIndex++)
        {
            TreeNode& childNode = treeNodes[childIndex];
            childNode.wm        = parentNode.wm * childNode.om;
        }
    }
}