// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREE_LINE_QUERY_H
#define PUBLIC_RW_COLLISION_KDTREE_LINE_QUERY_H

/*************************************************************************************************************

File: kdtreelinequery.h

Purpose: KDTree line query implementation.

*/

#include "rw/collision/common.h"
#include "rw/collision/kdtreelinequerybase.h"
#include "rw/collision/kdsubtree.h"
#include "rw/collision/kdtree.h"

namespace rw
{
namespace collision
{


/**
\brief
This class is to perform line queries against a KDTree.

The line used to query the KDTree has a start point and an end point.  The KDTree is traversed from
top to bottom and the child branch that is nearest to the start of the line query is always
traversed first.  When a leaf node is found that intersects the query line, all the entries in the
leaf node are returned.

\par Usage
To find all the entities that this line may intersect, use the following code.

\code
KDTree::LineQuery query(kdtree, start, end);
uint32_t index;

while (query.GetNext(index))
{
do something with yourObject[index]
}
\endcode

Notice that the entries returned by GetNext are in a leaf node that is intersected by the line, but
not all entries returned by GetNext are necessarily intersected by the line.  You will need to test
each entry individually against the line if you want to determine which ones are actually intersected.

To find the first entry intersected by a line, you call the GetNext method with two parameters.
This returns all the entries in the first leaf (or possibly two adjacent leaves) that are intersected by
the line.  The following code illustrates finding the first entry intersected by the line.  You must
provide the LineTouchesObject function which returns a boolean and distance, if result is true.

\code
KDTree::LineQuery query(kdtree, start, end);
uint32_t i, firstIndex, count, nearest;
float nearestDist = MaxReal32;

while (query.GetNext(firstIndex, count))
{
// Test all entries in the leaf. If the line intersects any entries, keep the nearest one.

for (i = 0; i < count; ++i)
{
if (LineTouchesObject(firstIndex + i, start, end, distance) && distance < nearestDist)
{
nearestDist = distance;
nearest = firstIndex + i;
}
}

//  If a nearest entry in found in a leaf node, there is no need to look in other leaf nodes.

if (nearestDistance < MaxReal32)
{
break;
}
}
\endcode

\importlib rwccore
*/

class KDTreeLineQuery:
        public KDTreeLineQueryBase
{
public:
    KDTreeLineQuery(const KDTreeBase *kdtree,
        rwpmath::Vector3::InParam start,
        rwpmath::Vector3::InParam end,
        const float fatness = 0.0f);

    KDTreeLineQuery(const KDSubTree *kdtree,
        rwpmath::Vector3::InParam start,
        rwpmath::Vector3::InParam end,
        const float fatness = 0.0f);


    RwpBool GetNext(uint32_t &entry);
    RwpBool GetNext(uint32_t &entry, uint32_t &count);
    void   ClipEnd(float endVal);
};


/**
\brief
This class is to perform line queries against a KDTree.

The line used to query the KDTree has a start point and an end point.  The KDTree is traversed from
top to bottom and the child branch that is nearest to the start of the line query is always
traversed first.  When a leaf node is found that intersects the query line, all the entries in the
leaf node are returned.

\par Usage
To find all the entities that this line may intersect, use the following code.

\code
KDTree::LineQuery query(kdtree, start, end);
uint32_t index;

while (query.GetNext(index))
{
do something with yourObject[index]
}
\endcode

Notice that the entries returned by GetNext are in a leaf node that is intersected by the line, but
not all entries returned by GetNext are necessarily intersected by the line.  You will need to test
each entry individually against the line if you want to determine which ones are actually intersected.

To find the first entry intersected by a line, you call the GetNext method with two parameters.
This returns all the entries in the first leaf (or possibly two adjacent leaves) that are intersected by
the line.  The following code illustrates finding the first entry intersected by the line.  You must
provide the LineTouchesObject function which returns a boolean and distance, if result is true.

\code
KDTree::LineQuery query(kdtree, start, end);
uint32_t i, firstIndex, count, nearest;
float nearestDist = MaxReal32;

while (query.GetNext(firstIndex, count))
{
// Test all entries in the leaf. If the line intersects any entries, keep the nearest one.

for (i = 0; i < count; ++i)
{
if (LineTouchesObject(firstIndex + i, start, end, distance) && distance < nearestDist)
{
nearestDist = distance;
nearest = firstIndex + i;
}
}

//  If a nearest entry in found in a leaf node, there is no need to look in other leaf nodes.

if (nearestDistance < MaxReal32)
{
break;
}
}
\endcode

\importlib rwccore
*/

RW_COLLISION_FORCE_INLINE
KDTreeLineQuery::KDTreeLineQuery(const KDTreeBase *kdtree,
                                 rwpmath::Vector3::InParam start,
                                 rwpmath::Vector3::InParam end,
                                 const float fatness /* = 0.0f */)
                                 :KDTreeLineQueryBase(kdtree, start, end, fatness)
{

}

RW_COLLISION_FORCE_INLINE
KDTreeLineQuery::KDTreeLineQuery(const KDSubTree *kdtree,
                                 rwpmath::Vector3::InParam start,
                                 rwpmath::Vector3::InParam end,
                                 const float fatness /* = 0.0f */)
                                 :KDTreeLineQueryBase(kdtree, start, end, fatness, kdtree->GetBranchNodeOffset(), kdtree->GetDefaultEntry())
{

}

/**
Find next kdtree entry from the leaf nodes that are intersected by the query line.

This return an entry index that \i might be intersected by the query line.  Be aware that the index
returned is the sorted index, and you will have to use the table returned by
GraphKDTree::GetSortedEntryIndices to convert the index back to the original index of the entry.

\note Although the line query visits each leaf node in nearest-first order, the entries within each
leaf are not returned in nearest-first order.  So if you want to make sure you find the \b nearest
entry that intersects the line, you should call GetNext splitThreshold times, where
splitThreshold is the maximum number of entries per leaf node, and compare the the entries to
see which is nearest.

\param  entry  Reference to variable that will receive the next entry index.

\return FALSE if there are no more results
\see GraphKDTree
*/
RW_COLLISION_FORCE_INLINE RwpBool
KDTreeLineQuery::GetNext(uint32_t &entry)
{
    // Use local variables to shadow member variables and reduce load-hit-storeds on Xenon
    uint32_t leafCount = m_leafCount;
    uint32_t nextEntry = m_nextEntry;

    while (leafCount == 0)
    {
        for ( ;; )
        {
            if (m_top == 0)
            {
                m_nextEntry = nextEntry;
                m_leafCount = leafCount;
                return FALSE; // No more nodes to process - end of query
            }
            if (m_stack[m_top-1].m_nodeRef.m_content != rwcKDTREE_BRANCH_NODE)
            {
                break; // Found leaf
            }
            ProcessBranchNode();
        }
        uint32_t top = m_top-1;
        leafCount = m_stack[top].m_nodeRef.m_content;
        nextEntry = m_stack[top].m_nodeRef.m_index;
        // ProcessBranchNode() uses m_top, so we need to write back here
        m_top = top;
    }

    m_nextEntry = nextEntry+1;
    m_leafCount = leafCount-1;

    entry = nextEntry;

    return TRUE;
}

/**
Gets the next set of entries from the same leaf nodes that are intersected by the query line.

This returns a set of entry indices that \i might be intersected by the query line.
Be aware that the indices returned are "sorted" so that they are grouped by leaf.  You will have
to use the table returned by GraphKDTree::GetSortedEntryIndices to convert them back to the
original indices.

Usually the entries returned by this method are all in one leaf, so the maximum count is the
splitThreshold specified in the GraphKDTree::Build.  The only exception is that, if the line
intersects two leaf nodes whose entries are consecutive (the left and right child of the same
parent), then the first index of the left child and the sum of the count of both leaves is returned.

\note Although the line query visits each leaf node in nearest-first order, the entries within each
leaf are not returned in nearest-first order.  So if you want to make sure you find the \b nearest
entry that intersects the line, you should compare all the entries returned to see which is nearest.

\param  entry  Reference to variable that will receive the index of the first entry.
\param count output number of entries found
\return FALSE if there are no more results
*/
RW_COLLISION_FORCE_INLINE RwpBool
KDTreeLineQuery::GetNext(uint32_t &entry, uint32_t &count)
{
    if (!GetNext(entry))
    {
        return FALSE;            // No more nodes to process - end of query
    }
    count = m_leafCount + 1;
    m_leafCount = 0;

    return TRUE;
}


/**
Sets the parametric length of the query line.

This allows you to shorten the query line while iterating through the results of a line query.
The clip end is initially set to 1.0f, which means the query line extends all the way from the
start point to the end point.  If you set the clip end to 0.8f, for example, the query line is
shortened by 20% so that the new end point is (start + 0.8f * (end - start) ). This will
eliminate from the iteration process any leaf nodes of the KDTree that
lie further along the line than the given point.

\param endVal  End clip parameter (should lie between 0 and 1).
*/
inline void
KDTreeLineQuery::ClipEnd(float endVal)
{
    uint32_t  i, iKeep;
    for (i=0, iKeep=0; i < m_top; i++)
    {
        if (m_stack[i].m_pa <= endVal)
        {
            m_stack[iKeep] = m_stack[i];
            m_stack[iKeep].m_pb = rwpmath::Min(m_stack[iKeep].m_pb, endVal);
            iKeep++;
        }
    }
    m_top = iKeep;

}
}
}
#endif
