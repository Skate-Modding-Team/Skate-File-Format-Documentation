// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: rwctriangleclusterprocedural.cpp

Purpose: Procedural aggregate of a single clusteredmeshcluster.

*/

// ***********************************************************************************************************
// Includes

#include <rw/collision/common.h>

#include "rw/collision/triangleclusterprocedural.h"
#include "rw/collision/aggregate.h"

#include "rw/collision/volumelinequery.h"
#include "rw/collision/volumebboxquery.h"

#include "rw/collision/triangle.h"
#include "rw/collision/aalineclipper.h"

#include "rw/collision/clustertriangleiterator.h"

#include "sharedclustermethods.h"

using namespace rw;
using namespace rwpmath;

namespace rw
{
namespace collision
{

/*
This is the default vtable used by objects of type TriangleClusterProcedural.
*/
rw::collision::Procedural::VTable TriangleClusterProcedural::sm_vTable =
{
    RWCOBJECTTYPE_TRIANGLECLUSTERPROCEDURAL,
    static_cast<rw::collision::Aggregate::GetSizeFn>              (&TriangleClusterProcedural::GetSizeThis),
    rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT,
    TRUE,
    static_cast<rw::collision::Aggregate::UpdateFn>               (&TriangleClusterProcedural::UpdateThis),
    static_cast<rw::collision::Aggregate::LineIntersectionQueryFn>(&TriangleClusterProcedural::LineIntersectionQueryThis),
    static_cast<rw::collision::Aggregate::BBoxOverlapQueryFn>     (&TriangleClusterProcedural::BBoxOverlapQueryThis),
    0, // rw::collision::Aggregate::GetNextVolumeFn
    0, // rw::collision::Aggregate::ClearAllProcessedFlags
    0  // rw::collision::Aggregate::ApplyUniformScale
};


EA::Physics::SizeAndAlignment
TriangleClusterProcedural::GetResourceDescriptor(
    const rw::collision::ClusterConstructionParameters & parameters)
{
    uint32_t size = 0;
    size = EA::Physics::SizeAlign<uint32_t>(sizeof(TriangleClusterProcedural), rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    size += rw::collision::ClusteredMeshCluster::GetSize(parameters);

    return EA::Physics::SizeAndAlignment(
        size,
        (rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT > rwcCLUSTEREDMESHCLUSTER_ALIGNMENT ?
            rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT : rwcCLUSTEREDMESHCLUSTER_ALIGNMENT));
}


TriangleClusterProcedural *
TriangleClusterProcedural::Initialize(
    const EA::Physics::MemoryPtr & resource,
    const rw::collision::ClusterConstructionParameters & parameters)
{
    // Check the alignment of the resource
    rwcASSERTALIGN(resource.GetMemory(), rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);

    uintptr_t res = reinterpret_cast<uintptr_t>(resource.GetMemory());
    res = EA::Physics::SizeAlign<uintptr_t>(res, rwcTRIANGLECLUSTERPROCEDURAL_ALIGNMENT);

    // Construct the object
    TriangleClusterProcedural *agg = new (reinterpret_cast<void*>(res)) TriangleClusterProcedural(parameters, &sm_vTable);

    // Advance the resource pointer
    res += sizeof(TriangleClusterProcedural);

    // Allocate the ClusteredMeshCluster
    res = EA::Physics::SizeAlign<uintptr_t>(res, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);
    agg->mCluster = rw::collision::ClusteredMeshCluster::Initialize(reinterpret_cast<void*>(res), parameters);

    // Return the object
    return agg;
}


TriangleClusterProcedural::TriangleClusterProcedural(
    const rw::collision::ClusterConstructionParameters & parameters,
    rw::collision::Procedural::VTable *vTable)
    : Procedural(static_cast<uint32_t>(parameters.mTriangleUnitCount + parameters.mQuadUnitCount), vTable)
    , mCluster(NULL)
    , mSizeOfThis(GetResourceDescriptor(parameters).GetSize())
{
    mClusterParams.mFlags = CMFLAG_ONESIDED;
    mClusterParams.mGroupIdSize = 0;
    mClusterParams.mSurfaceIdSize = 0;
    mClusterParams.mVertexCompressionGranularity = 0.0f;
}


TriangleClusterProcedural::TriangleClusterProcedural(
    rw::collision::ClusteredMeshCluster * cluster,
    rw::collision::Procedural::VTable *vTable)
    : Procedural(0u, vTable)
    , mCluster(cluster)
    , mSizeOfThis(0u)
{
    // The members initialized here will be overwritten during serialization.
    mClusterParams.mFlags = CMFLAG_ONESIDED;
    mClusterParams.mGroupIdSize = 0;
    mClusterParams.mSurfaceIdSize = 0;
    mClusterParams.mVertexCompressionGranularity = 0.0f;
}

void
TriangleClusterProcedural::UpdateWithBBox(const AABBox & bbox)
{
	m_AABB = bbox;

	// Set the num tag bits required to address 
	// units and one bit for storing the unit-triangle index
	const uint32_t numUnitTagBits = static_cast<uint32_t>(rwpmath::Log(static_cast<float>(mCluster->unitDataSize)) / rwpmath::Log(2.0f)) + 1u;
	m_numTagBits = numUnitTagBits + 1u;
}

void
TriangleClusterProcedural::UpdateThis()
{
    // Update the overall bounding box
    rwpmath::Vector3 minV, maxV;
    minV = maxV = mCluster->GetVertex(0, mClusterParams.mVertexCompressionGranularity);
    for (uint32_t vertexIndex = 1; vertexIndex < mCluster->vertexCount; ++vertexIndex)
    {
        const rwpmath::Vector3 v = mCluster->GetVertex(static_cast<uint8_t>(vertexIndex), mClusterParams.mVertexCompressionGranularity);
        minV = rwpmath::Min(minV, v);
        maxV = rwpmath::Max(maxV, v);
    }

	UpdateWithBBox(AABBox(minV, maxV));
}


RwpBool
TriangleClusterProcedural::LineIntersectionQueryThis(
    VolumeLineQuery *lineQuery,
    const rwpmath::Matrix44Affine *tm)
{
    uint32_t unitOffset = 0;
    uint32_t unitCount = 0;
    uint32_t numTrisLeftInUnit = 0;

    void * queryInProgress = lineQuery->m_curSpatialMapQuery;

    // Map line into cluster space
    rwpmath::Matrix44Affine invTm(*tm);
    invTm= InverseOfMatrixWithOrthonormal3x3(invTm);
    const rwpmath::Vector3 localLineStart = TransformPoint(lineQuery->m_pt1, invTm);
    const rwpmath::Vector3 localLineEnd   = TransformPoint(lineQuery->m_pt2, invTm);
    const rwpmath::Vector3 localLineDelta = localLineEnd - localLineStart;

    // Determine if a query is already in progress
    if (!queryInProgress)
    {
        // Set the pointer to a non-NULL value to indicate a query is in progress.
        lineQuery->m_curSpatialMapQuery = lineQuery->m_spatialMapQueryMem;
        // Initialize the unit count to the number of units in the cluster.
        unitCount = mCluster->unitCount;
        // Initialize the unitOffset to 0, indicating the first unit in the cluster.
        unitOffset = 0;
        // Initialize the number of triangles in the current unit.
        // NOTE: Here 0 is a special case value when used to initialize a
        // ClusterTriangleIterator which indicates that all triangles in
        // the current unit need to be processed.
        numTrisLeftInUnit = 0;
    }
    else
    {
        // Restore the state of the ongoing query
        unitOffset = lineQuery->m_clusteredMeshRestartData.entry;
        unitCount = lineQuery->m_clusteredMeshRestartData.unitCount;
        numTrisLeftInUnit = lineQuery->m_clusteredMeshRestartData.numTrisLeftInUnit;
    }

    // Initialize a ClusterTriangleIterator
    ClusterTriangleIterator<> cti(*mCluster, mClusterParams, unitOffset, unitCount, numTrisLeftInUnit);
    EA_ASSERT(cti.IsValid());

    // While there are still triangles to iterate
    for(;!cti.AtEnd(); cti.Next())
    {
        // Extract the vertices from the current triangle
        Vector3 v0, v1, v2;
        cti.GetVertices(v0, v1, v2);

        RwpBool hit = false;
        VolumeLineSegIntersectResult tmpRes;

        // TODO : add IsOneSided method to ClusteredMeshCluster
        if (0 != (mClusterParams.mFlags & CMFLAG_ONESIDED))
        {
            hit = rw::collision::TriangleLineSegIntersect(tmpRes, localLineStart, localLineDelta, v0, v1, v2, lineQuery->m_fatness);
        }
        else
        {
            hit = rw::collision::TriangleLineSegIntersectTwoSided(tmpRes, localLineStart, localLineDelta, v0, v1, v2, lineQuery->m_fatness);
        }

        if (hit)
        {
            // Check that the query buffers are not full
            if (lineQuery->m_resCount == lineQuery->m_resMax ||
                lineQuery->m_instVolCount == lineQuery->m_instVolMax)
            {
                // Cache current position in the query so we can restart from this exact point.
                lineQuery->m_clusteredMeshRestartData.entry = cti.GetOffset();
                lineQuery->m_clusteredMeshRestartData.unitCount = cti.GetRemainingUnits();
                lineQuery->m_clusteredMeshRestartData.numTrisLeftInUnit = cti.GetNumTrianglesLeftInCurrentUnit();
                // Return false indicating the query has not finished
                return FALSE;
            }

            // Get the next free result from the query
            VolumeLineSegIntersectResult *res = &lineQuery->m_resBuffer[lineQuery->m_resCount++];

            // Get the next free volume from the query volume pool
            rw::collision::Volume *vol = &lineQuery->m_instVolPool[lineQuery->m_instVolCount++];
            // Initialize the volume to a triangle volume
            rw::collision::TriangleVolume *triangleVolume = TriangleVolume::Initialize(EA::Physics::MemoryPtr(vol), v0, v1, v2);
            // Instance the intersected triangle
            InitializeTriangleVolumeDetails(
                *triangleVolume,
                cti);

            res->inputIndex = lineQuery->m_currInput-1;
            res->v = lineQuery->m_inputVols[res->inputIndex];

            // Map intersect result back into query space
            res->position = TransformPoint(tmpRes.position, *tm);
            res->normal = TransformVector(tmpRes.normal, *tm);
            res->volParam = tmpRes.volParam;
            res->lineParam = tmpRes.lineParam;

            // In future the vref should be in a freelist
            res->vRef.volume = vol;
            res->vRef.tmContents = *tm;
            res->vRef.tm = &tmpRes.vRef.tmContents;

            // Setup tag to this triangle
            uint32_t tag = lineQuery->m_tag;
            uint32_t numTagBits = lineQuery->m_numTagBits;

            // Get the child index of the triangle
            uint32_t childIndex = GetChildIndex(
                                      cti.GetOffset(),
                                      cti.GetNumTrianglesLeftInCurrentUnit() - 1u);

            // Update the result tag with the child index of this triangle
            UpdateTagWithChildIndex(tag, numTagBits, childIndex);

            // Set the result tag
            res->vRef.tag = tag;
            res->vRef.numTagBits = static_cast<uint8_t>(numTagBits);
        }
    }

    // Return True indicating we have added all intersections to the results buffer.
    return TRUE;
}


RwpBool
TriangleClusterProcedural::BBoxOverlapQueryThis(
    VolumeBBoxQuery *bboxQuery,
    const rwpmath::Matrix44Affine *tm)
{
    uint32_t unitOffset = 0;
    uint32_t unitCount = 0;
    uint32_t numTrisLeftInUnit = 0;

    // Pointer is used to determine whether or not a query is in progress
    void * queryInProgress = bboxQuery->m_curSpatialMapQuery;

    // The spatial map query memory is used to store the potentially-adjusted bbox
    // used during this query.
    AABBox * queryBBox = reinterpret_cast<AABBox*>(bboxQuery->m_spatialMapQueryMem);

    // Check if a query is already in progress
    if (!queryInProgress)
    {
        AABBox bboxGranular;

        // Map line into cluster space
        if (tm)
        {
            const Matrix44Affine invTm(InverseOfMatrixWithOrthonormal3x3(*tm));
            bboxGranular = bboxQuery->m_aabb.Transform(&invTm);
        }
        else
        {
            bboxGranular = bboxQuery->m_aabb;
        }

        // Expand the the query aabbox by vertex compression granularity
        // NOTE: This is to work around the fact that vertex compression
        // makes vertex coordinates move around within the compression granularity
        VecFloat granularityImprecision = GetVecFloat_Two() * VecFloat(mClusterParams.mVertexCompressionGranularity );
        bboxGranular.m_min -= granularityImprecision;
        bboxGranular.m_max += granularityImprecision;

        // Use the bboxQuery->m_spatialMapQueryMem to store the adjusted query bbox.
        (*queryBBox) = bboxGranular;

        // Set the pointer to a non-NULL value to indicate a query is in progress.
        bboxQuery->m_curSpatialMapQuery = bboxQuery->m_spatialMapQueryMem;
        // Initialize the unit count to the number of units in the cluster.
        unitCount = mCluster->unitCount;
        // Initialize the unitOffset to 0, indicating the first unit in the cluster.
        unitOffset = 0;
        // Initialize the number of triangles in the current unit.
        // NOTE: Here 0 is a special case value when used to initialize a
        // ClusterTriangleIterator which indicates that all triangles in
        // the current unit need to be processed.
        numTrisLeftInUnit = 0;
    }
    else
    {
        // Resume from last saved point
        unitOffset = bboxQuery->m_clusteredMeshRestartData.entry;
        unitCount = bboxQuery->m_clusteredMeshRestartData.unitCount;
        numTrisLeftInUnit = bboxQuery->m_clusteredMeshRestartData.numTrisLeftInUnit;
    }

    // Initialize a ClusterTriangleIterator
    rw::collision::ClusterTriangleIterator<> cti(*mCluster, mClusterParams, unitOffset, unitCount, numTrisLeftInUnit);
    EA_ASSERT(cti.IsValid());

    // While there are still triangles to iterate
    for(;!cti.AtEnd(); cti.Next())
    {
        // Extract the vertices from the current triangle
        rwpmath::Vector3 v0, v1, v2;
        cti.GetVertices(v0, v1, v2);

        // Calculate the triangle's aabbox
        const Vector3 bboxMin(Min(Min(v0, v1), v2));
        const Vector3 bboxMax(Max(Max(v0, v1), v2));
        const AABBox triangleAABBox(bboxMin, bboxMax);

        // Test the bbox of the triangle against the query bbox.
        if (queryBBox->Overlaps(triangleAABBox))
        {
            // Check that the query buffers are not full
            if (bboxQuery->m_primNext == bboxQuery->m_primBufferSize ||
                bboxQuery->m_instVolCount == bboxQuery->m_instVolMax)
            {

                // Set flags to indicate why the query has paused
                if (bboxQuery->m_primNext == bboxQuery->m_primBufferSize)
                {
                    bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFRESULTBUFFERSPACE);
                }

                if (bboxQuery->m_instVolCount == bboxQuery->m_instVolMax)
                {
                    bboxQuery->SetFlags(bboxQuery->GetFlags() | VolumeBBoxQuery::VOLUMEBBOXQUERY_RANOUTOFINSTANCEBUFFERSPACE);
                }

                // Cache current position in the query so we can restart from this exact point.
                bboxQuery->m_clusteredMeshRestartData.entry = cti.GetOffset();
                bboxQuery->m_clusteredMeshRestartData.unitCount = cti.GetRemainingUnits();
                bboxQuery->m_clusteredMeshRestartData.numTrisLeftInUnit = cti.GetNumTrianglesLeftInCurrentUnit();
                // Return false indicating the query has not finished
                return FALSE;
            }

            // Get the next free volume from the query volume pool
            rw::collision::Volume *vol = &bboxQuery->m_instVolPool[bboxQuery->m_instVolCount++];
            // Initialize the volume to a triangle volume
            rw::collision::TriangleVolume *triangleVolume = TriangleVolume::Initialize(EA::Physics::MemoryPtr(vol), v0, v1, v2);
            // Instance the intersected triangle
            InitializeTriangleVolumeDetails(
                *triangleVolume,
                cti);

            // Setup tag to this triangle
            uint32_t tag = bboxQuery->m_tag;
            uint32_t numTagBits = bboxQuery->m_numTagBits;

            // Get the child index of the triangle
            uint32_t childIndex = GetChildIndex(
                                      cti.GetOffset(),
                                      cti.GetNumTrianglesLeftInCurrentUnit() - 1u);

            // Update the result tag with the child index of this triangle
            UpdateTagWithChildIndex(tag, numTagBits, childIndex);

            if (tm)
            {
                const Vector3 v0_t(TransformPoint(v0, *tm));
                const Vector3 v1_t(TransformPoint(v1, *tm));
                const Vector3 v2_t(TransformPoint(v2, *tm));

                const Vector3 tribboxMin(Min(Min(v0_t, v1_t), v2_t));
                const Vector3 tribboxMax(Max(Max(v0_t, v1_t), v2_t));
                const AABBox triangleAABBox_t(tribboxMin, tribboxMax);

                bboxQuery->AddPrimitiveRef(vol, tm, triangleAABBox_t, tag, static_cast<uint8_t>(numTagBits));
            }
            else
            {
                bboxQuery->AddPrimitiveRef(vol, tm, triangleAABBox, tag,static_cast<uint8_t>(numTagBits));
            }
        }
    }

    // Return True indicating we have added all intersections to the results buffer.
    return TRUE;
}


void
TriangleClusterProcedural::GetVolumeFromChildIndex(
    rw::collision::TriangleVolume & triangleVolume,
    const uint32_t childIndex) const
{
    // Extract the indices/offsets from the child index
    const uint32_t unitOffset = GetUnitOffsetFromChildIndex(childIndex);
    const uint32_t triangleIndex = GetTriangleIndexWithinUnitFromChildIndex(childIndex);

    // Get the volume from the cluster
    mCluster->GetTriangleVolume(
        triangleVolume,
        unitOffset,
        triangleIndex,
        mClusterParams);
}


void
TriangleClusterProcedural::GetVertexIndicesFromChildIndex(
    uint8_t &v0,
    uint8_t &v1,
    uint8_t &v2,
    const uint32_t childIndex) const
{
    // Extract the indices/offsets from the child index
    const uint32_t unitOffset = GetUnitOffsetFromChildIndex(childIndex);
    const uint32_t triangleIndex = GetTriangleIndexWithinUnitFromChildIndex(childIndex);

    // Get the volume from the cluster
    mCluster->GetTriangleVertexIndices(
        v0, v1, v2,
        unitOffset,
        triangleIndex,
        mClusterParams);
}


const TriangleClusterProcedural::ObjectDescriptor
TriangleClusterProcedural::GetObjectDescriptor() const
{
    return ObjectDescriptor(GetClusterSize(*mCluster));
}

} // namespace collision
} // namespace rw
