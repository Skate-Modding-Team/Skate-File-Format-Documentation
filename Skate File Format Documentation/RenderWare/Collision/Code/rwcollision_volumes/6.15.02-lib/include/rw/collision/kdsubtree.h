// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDSUBTREE_H
#define PUBLIC_RW_COLLISION_KDSUBTREE_H

/*************************************************************************************************************

File: kdsubtree.h

Purpose: KDSubTree spatial map.
*/


#include "rw/collision/common.h"
#include "rw/collision/kdtreebase.h"
#include "rw/collision/aabbox.h"

namespace rw
{
    namespace collision
    {

        // ***********************************************************************************************************
        //                                           rw::collision::KDSubTree CLASS
        // ***********************************************************************************************************

        /**

        \brief
        The KDSubTree is derived from the KDTree spatial map. When created using the utility functions it uses the 
        same branchnodes as its parent tree, and provides direct access to the nodes relating to a specific cluster
        in a clusteredmesh.
        
        \par Usage
        The KDSubTree supports the same functionality of the KDTree, line and bbox queries. The KDSubtree
        supports setting of a root branch node. This must be done if the tree is to be uploaded to SPU. (When created 
        using CreateKDSubTreeArray(...) the branchnodes still 'belong' to the parent KDTree - so must also be copied to SPU.

        CreateKDSubTreeArray(...) will not always produce consistent KDTrees - the bounding box set will be the union of 
        all the vertices in the cluster for which it corresponds. The actual extents of the KDTree may be outside of this 
        if the clustered mesh is compressed - depending on the compression granularity.
           
        \importlib rwccore
        */

        class KDSubTree : public KDTreeBase
        {
        private:
            
            uint32_t    m_branchNodeOffset;
            uint32_t    m_defaultEntry;

            struct ValidityCheckNodeData
            {
                uint32_t parent; ///< the index of the parent branch node
                AABBox   bbox;   ///< the bounding box of the node which is the union of the bounding boxes of all its children.
            };

        public:

            KDSubTree():KDTreeBase() {}
            /**
            \brief
            Initializes the KDSubTree
    
            \param parentKDTree Pointer to ParentKDTree
            \param branchNodeIndex  Index of KDSubTree root node within parents branchnode array 
            \param numBranchNodes Number of branchnodes belonging to KDSubTree
            \param numEntries Number of Entries belonging to KDSubTree
            \param bbox Bounding Box containing all entries to KDSubTree

            */
            void
                Initialize(KDTreeBase* parentKDTree,
                uint32_t branchNodeIndex,
                uint32_t numBranchNodes,
                uint32_t numEntries,
                uint32_t defaultEntry,
                const AABBox &bbox)
            {
                m_numBranchNodes = numBranchNodes;
                m_numEntries = numEntries;
                m_defaultEntry= defaultEntry;
                m_bbox = bbox;
                m_branchNodes = &(parentKDTree->m_branchNodes[branchNodeIndex]);
                m_branchNodeOffset = branchNodeIndex;
            }


            /**
            \brief
            Gets a pointer to the root branchnode

            \return BranchNode* pointer to cluster root branchnode
            */
            BranchNode* GetRootNode() const
            {
                return m_branchNodes;
            }

            /**
            \brief
            Sets the Root branchnode of the KDSubTree

            \param clusterBranchNodes Pointer to the root node of the KDSubTree.

            \return uint32_t value of branch node offset
            */
            void SetRootNode(BranchNode* clusterBranchNodes)
            {
                m_branchNodes=clusterBranchNodes;
            }


            /**
            \brief
            Gets the default entry for use in the case of a single leaf KDSubTree

            \return uint32_t Default entry to return from queries on a single leaf KDTree.
            */
            uint32_t GetDefaultEntry() const
            {
                return m_defaultEntry;
            }

            /**
            \brief
            Sets the default entry for use in the case of a single leaf KDSubTree

            \param uint32_t Default entry to return from queries on a single leaf KDTree.
            */
            void SetDefaultEntry(uint32_t defaultEntry)
            {
                m_defaultEntry=defaultEntry;
            }

            /**
            \brief
            Gets the branch node offset value

            \return uint32_t value of branch node offset
            */
            uint32_t GetBranchNodeOffset() const
            {
                return m_branchNodeOffset;
            }

            /**
            \brief
            Sets the branch node offset value
            */
            void SetBranchNodeOffset(uint32_t branchNodeOffset)
            {
                m_branchNodeOffset=branchNodeOffset;
            }

            void
                Release();

            RwpBool
                IsValid() const;

            /**
            \brief Replace the root node pointer with new value based on current offset into branch nodes from the given KDTree.
            For use after de-serializing the KDSubTree.
            */
            inline void AttachToKDTree(KDTreeBase * kdtree)
            {
                SetRootNode(kdtree->m_branchNodes + m_branchNodeOffset);
            }

            // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
            // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
            template <class Archive>
            void Serialize(Archive &ar, uint32_t version)
            {
                // Does *not* serialize the branch nodes (we don't own these) or even the pointer
                // to them. Expect caller to use AttachToKDTree() to set this up.
                KDTreeBase::SerializeData(ar, version);
                ar & EA_SERIALIZATION_NAMED_VALUE(m_branchNodeOffset);
                ar & EA_SERIALIZATION_NAMED_VALUE(m_defaultEntry);

            }

            /**
            \brief
            Computes the bounding box for a branchnode of a KDTree

            \param bbox Reference to result bbox
            \param branchIndex Index of branchnode for which bounding box is needed.
            \param kdtree Pointer to KDTreeBase to be queried.

            void ComputeNodeBoundingBox(AABBox &bbox, uint32_t branchIndex, KDTreeBase *kdtree)
            {   
                KDTree::BranchNode *branchNodes = kdtree->m_branchNodes;
                bbox = kdtree->GetBBox();
                uint32_t currentNode=0;
                while(currentNode!=branchIndex)
                {
                    bool leftBranch = branchNodes[currentNode].m_childRefs[0].m_content==rwcKDTREE_BRANCH_NODE;
                    bool rightBranch = branchNodes[currentNode].m_childRefs[1].m_content==rwcKDTREE_BRANCH_NODE;
                    bool rightSmaller = rightBranch && branchNodes[currentNode].m_childRefs[1].m_index<=branchIndex;
                    //left is branchnode, right is not, or is bigger- move left
                    if(leftBranch && !rightSmaller)
                    {
                        bbox.m_max.GetComponent((int)branchNodes[currentNode].m_axis)=branchNodes[currentNode].m_extents[0]; 
                        currentNode=branchNodes[currentNode].m_childRefs[0].m_index;
                        continue;
                    }
                    //left is leafnode, right is not, or is smaller - move right;
                    if(rightSmaller)
                    {
                        bbox.m_min.GetComponent((int)branchNodes[currentNode].m_axis)=branchNodes[currentNode].m_extents[1]; 
                        currentNode=branchNodes[currentNode].m_childRefs[1].m_index;
                        continue;
                    }
                    EA_ASSERT(0);  //Malformed KDTree
                }
            }
            */
        };

    } // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_KDSUBTREE_H

