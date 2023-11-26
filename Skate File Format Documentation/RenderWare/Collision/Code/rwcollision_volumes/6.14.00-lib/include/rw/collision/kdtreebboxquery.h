// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREE_BB_QUERY_H
#define PUBLIC_RW_COLLISION_KDTREE_BB_QUERY_H

/*************************************************************************************************************

File: kdtreebboxquery.h

Purpose: Base class for KDTree AABB queries.

*/

#include "rw/collision/common.h"
#include "rw/collision/kdtreebboxquerybase.h"
#include "rw/collision/kdsubtree.h"
#include "rw/collision/kdtree.h"

namespace rw
{
namespace collision
{


/**
A KDTree bounding box query may be used to find all leaf nodes of the kd tree
that intersect the bounding box, and return the entries they contain.

The KDTree does not store and test individual entry bounding boxes so this can be quite a 'loose'
set of results depending on the size of the leaf nodes. The caller may refine the results
by testing the query box against individual entry boxes (computed or stored elsewhere), or by
doing more precise intersection tests appropriate to the problem.

\par Usage
To find all the entities that this bbox may intersect, use the following code.

\code
KDTree::BBoxQuery query(kdtree, testbbox);
uint32_t index;

while (query.GetNext(index))
{
do something with yourObject[index]
}
\endcode
*/

class KDSubTree;

class KDTreeBBoxQuery : public KDTreeBBoxQueryBase
{
public:

    KDTreeBBoxQuery(const KDTreeBase *kdtree, const AABBox &bbox);
    KDTreeBBoxQuery(const KDSubTree *kdtree, const AABBox &bbox);
    RwpBool GetNext(uint32_t &entry, uint32_t &count);
    RwpBool GetNext(uint32_t &entry);
};

/**
A KDTree bounding box query may be used to find all leaf nodes of the kd tree
that intersect the bounding box, and return the entries they contain.

The KDTree does not store and test individual entry bounding boxes so this can be quite a 'loose'
set of results depending on the size of the leaf nodes. The caller may refine the results
by testing the query box against individual entry boxes (computed or stored elsewhere), or by
doing more precise intersection tests appropriate to the problem.

\par Usage
To find all the entities that this bbox may intersect, use the following code.

\code
KDTree::BBoxQuery query(kdtree, testbbox);
uint32_t index;

while (query.GetNext(index))
{
do something with yourObject[index]
}
\endcode
*/

RW_COLLISION_FORCE_INLINE
KDTreeBBoxQuery::KDTreeBBoxQuery(const KDTreeBase *kdtree, const AABBox &bbox)
: KDTreeBBoxQueryBase(kdtree, bbox)
{

}

RW_COLLISION_FORCE_INLINE
KDTreeBBoxQuery::KDTreeBBoxQuery(const KDSubTree *kdtree, const AABBox &bbox)
: KDTreeBBoxQueryBase(kdtree, bbox, kdtree->GetBranchNodeOffset(), kdtree->GetDefaultEntry())
{

}
/**
\brief Find next kdtree entry from the leaf nodes that are intersected by the query box.

This return an entry index that \i might be intersected by the query box.  Be aware that the index
returned is the sorted index, and you will have to use the table returned by
GraphKDTree::GetSortedEntryIndices to convert the index back to the original index of the entry.

\param  entry  Reference to variable that will receive the next entry index.

\return FALSE if there are no more results
\see GraphKDTree
*/

RW_COLLISION_FORCE_INLINE RwpBool
KDTreeBBoxQuery::GetNext(uint32_t &entry)
{
    while (m_resultCount == 0)
    {
        if (m_top == 0)
        {
            return FALSE; // No more nodes to process - end of query
        }
        ProcessBranchNode();
    }

    entry = m_nextEntry++;
    m_resultCount--;

    return TRUE;
}

/**
brief Gets the next set of entries from the same leaf nodes that are intersected by the query box.

This returns a set of entry indices that \i might be intersected by the query box.
Be aware that the indices returned are "sorted" so that they are grouped by leaf.  You will have
to use the table returned by GraphKDTree::GetSortedEntryIndices to convert them back to the
original indices.

Usually the entries returned by this method are all in one leaf, so the maximum count is the
splitThreshold specified in the GraphKDTree::Build.  The only exception is that, if the box
intersects two leaf nodes whose entries are consecutive (the left and right child of the same
parent), then the first index of the left child and the sum of the count of both leaves is returned.

\param entry output variable that will receive the index of the first entry.
\param count output number of entries found
\return FALSE if there are no more results
*/
RW_COLLISION_FORCE_INLINE RwpBool
KDTreeBBoxQuery::GetNext(uint32_t &entry, uint32_t &count)
{
    if (!GetNext(entry))
    {
        return FALSE; // No more nodes to process - end of query
    }
    count = m_resultCount + 1;
    m_resultCount = 0;

    return TRUE;
}
}
}
#endif
