// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_KDTREE_BB_BATCH_QUERY_H
#define PUBLIC_RW_COLLISION_KDTREE_BB_BATCH_QUERY_H

/*************************************************************************************************************

File: kdtreebatchbboxquery.h

Purpose: Definition for kdtree aabbox batch queries.

*/

#include "rw/collision/kdsubtree.h"
#include "rw/collision/kdtreebboxquery.h"

namespace rw
{
namespace collision
{


/** 
\brief Structure to contain the results of a batch bbox query
*/
struct KDTreeBatchBBoxQueryResult
{
    uint32_t firstEntry; ///<The start offset of the result
    uint32_t count;      ///<The number of entries that were returned
};


/**
\internal
\brief
Class to query the kd tree against an AABB and return a batch of results
\importlib rwccore
*/
class KDTreeBatchBBoxQuery : public KDTreeBBoxQueryBase
{
public:
    /**
    \brief Constructor
    */
    KDTreeBatchBBoxQuery()
        : KDTreeBBoxQueryBase()
    {

    }


    /**
    \brief Constructor
    \param kdtree The KDTree to query
    \param bbox The axis aligned bounding box to query against the kd tree
    */
    KDTreeBatchBBoxQuery(const KDTree *kdtree, const AABBox &bbox)
        : KDTreeBBoxQueryBase(kdtree, bbox)
    {

    };

    KDTreeBatchBBoxQuery(const KDSubTree *kdtree, const AABBox &bbox)
        : KDTreeBBoxQueryBase(kdtree, bbox, kdtree->GetBranchNodeOffset(), kdtree->GetDefaultEntry())
    {

    };

    RwpBool GetBatch(KDTreeBatchBBoxQueryResult * results, uint32_t maxResults, uint32_t & numResults);
};


/**
\brief Returns a batch of results.

This will continue traversing the kdtree looking for leaf nodes which are within the aabb until either it has
exhausted all the node or it has run out of room in the buffer.

\param results A buffer to hold the results in
\param maxResults The maximum number of results to return (usually the same size as the supplied buffer)
\param numResults The number of results that actaully got returned
\return TRUE if the query has finished traversing the kd tree, false if there is more work to do

\par Usage
This example shows how to query the kd tree in a batch

\code
KDTreeBatchBBoxQuery kdTreeBatchAABBQuery = KDTreeBatchBBoxQuery(queryKDTree, queryAABBox);
uint32_t numResults=0;
KDTreeBatchBBoxQueryResult results[500];

while(kdTreeBatchAABBQuery.GetBatch(results, 500, numResults)
{
    for(uint32_t i=0; i<numResults; i++)
    {
        uint32_t offset = results[i].node;
        for(uint32_t j=0; j<results[i].count; j++)
        {
            //do something with the offset
            //increment the offset
        }
    }
}
\endcode

*/
inline RwpBool
KDTreeBatchBBoxQuery::GetBatch(KDTreeBatchBBoxQueryResult * results, uint32_t maxResults, uint32_t &numResults)
{
    EA_ASSERT(m_kdtree);

    uint32_t numResultsInOutput=0;

    //Keep processing branch nodes until we run out of room
    while(numResultsInOutput < maxResults)
    {
        while (m_resultCount == 0)
        {
            if (m_top == 0)
            {
                numResults = numResultsInOutput;
                return TRUE; // No more nodes to process - end of query
            }
            ProcessBranchNode();
        }

        results[numResultsInOutput].firstEntry = m_nextEntry;
        results[numResultsInOutput].count      = m_resultCount;
        numResultsInOutput++;
        m_resultCount = 0;
    }

    numResults = numResultsInOutput;
    return FALSE;
}

}
}
#endif
