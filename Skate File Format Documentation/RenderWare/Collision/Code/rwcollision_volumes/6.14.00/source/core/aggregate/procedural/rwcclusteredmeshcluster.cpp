// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: rwcClusteredMeshCluster.cpp

Purpose: Procedural aggregate of triangles with KDTree spatial map.

*/

// ***********************************************************************************************************
// Includes

#include "rw/collision/clusteredmeshcluster.h"
#include "rw/collision/clusteredmeshcluster_methods.h"

#if !defined(EA_PLATFORM_PS3_SPU)
#include "rw/collision/triangle.h"
#include "sharedclustermethods.h"
#endif // !defined(EA_PLATFORM_PS3_SPU)


#include "rw/collision/clustertriangleiterator.h"


namespace rw
{
namespace collision
{

/**
\brief Queries the cluster specified unit with an AABB and returns all of the triangles that are within it

\param offset               The offset to the unit to query
\param bbox                 The axis aligned bounding box to query against
\param transform            The transform of the mesh volume
\param instances            A pointer to a buffer big enough to hold the triangles that this function instances
\param numPrimitivesInUnit  Returns the number of GPTriangles that were instanced (1 for a triangle, 2 for a quad)
\param clusterParam         The ClusterParams struct from the mesh that the cluster belongs to
\param 
*/
uint32_t
ClusteredMeshCluster::UnitGetOverlappingGPInstances(uint32_t offset, const AABBox &bbox, const rwpmath::Matrix44Affine *transform,
                                                    GPTriangle *instances, uint32_t &numPrimitivesInUnit, const ClusterParams & clusterParams)
{
    rwcDEPRECATED("GPInstance is deprecated. Use ClusterUnitWalker API to extract data from clusters.");
    uint8_t *dataStream = &UnitData()[offset];
    uint32_t unitType = (uint32_t)(dataStream[0] & UNITTYPE_MASK);
    EA_ASSERT(unitType <= UNITTYPE_TRILIST);
    uint32_t triCount;


    // TODO: PAB: compare code generation for branch-free approach

    // compute unit type masks
    uint32_t triangleMask = ~(uint32_t)((-(int32_t)((uint8_t)UNITTYPE_TRIANGLE ^ (uint8_t)unitType)) >> 31);
    uint32_t quadMask     = ~(uint32_t)((-(int32_t)((uint8_t)UNITTYPE_QUAD     ^ (uint8_t)unitType)) >> 31);
    uint32_t triListMask  = ~(uint32_t)((-(int32_t)((uint8_t)UNITTYPE_TRILIST  ^ (uint8_t)unitType)) >> 31);

    // branch free triangle count computation
    triCount = (1 & triangleMask) + (2 & quadMask) + (*(dataStream + 1) & triListMask);

    // The vertex indices follow the type and optional count
    uint8_t *vIndex = (dataStream + 1) + (triListMask & 1);

    // The optional edge angles follow the vertex indices
    uint8_t *edge = vIndex + triCount + 2;

    // Miscellaneous data follows the optional edge angles
    uint32_t edgeAngleMask  = (uint32_t)((-(int32_t)((uint8_t)UNITFLAG_EDGEANGLE  & dataStream[0])) >> 31);
    uint8_t *misc = edge + ( edgeAngleMask & (triCount + 2) );


    // TODO: PAB: default ids could be a mesh property, then the mesh property can default to zero
    // The default ids are zero, see volume.hpp
    uint32_t groupId = 0;
    uint32_t surfaceId = 0;


    //  Parse the misc data, will be put into the userTag
    if (dataStream[0] & UNITFLAG_GROUPID)
    {
        groupId = *(misc++);
        groupId += (clusterParams.mGroupIdSize==2) ? *(misc++) * 256 : 0;
    }
    if (dataStream[0] & UNITFLAG_SURFACEID)
    {
        surfaceId = *(misc++);
        surfaceId += (clusterParams.mSurfaceIdSize==2) ? *(misc++) * 256 : 0;
    }

    uint32_t size = (uint32_t)(misc - dataStream);
    rwpmath::Vector3 v[4];

    //uint32_t volumeRef = reinterpret_cast<uint32_t>(const_cast<ClusteredMesh*>(this));
    // TODO: PAB: could try moving vertex get / decompression outside of these
    // clauses - always do 4 vertices and duplicate one for the triangle case
    // then the bbox construction and early-out overlap test could be moved
    // outside of both clauses...

    if (triCount == 1)          // Single triangle
    {
        // Decompress the vertices
        Get3Vertices(v, vIndex[0], vIndex[1], vIndex[2], clusterParams.mVertexCompressionGranularity);

        // generate an AABBox from the vertices
        AABBox unitBBox(Min(Min(v[0], v[1]), v[2]), Max(Max(v[0], v[1]), v[2]));

        // if not overlapping, do not generate triangle instance
        if(!bbox.Overlaps(unitBBox))
        {
            numPrimitivesInUnit = 0;
            return size;
        }

        // transform the vertices into world space
        v[0] = TransformPoint(v[0], *transform);
        v[1] = TransformPoint(v[1], *transform);
        v[2] = TransformPoint(v[2], *transform);

        // generate the triangle normal
        rwpmath::Vector3 normal = Normalize(Cross((v[1] - v[0]), (v[2] - v[0])));

        // default triangle flags
        uint32_t triflags = GPInstance::FLAG_TRIANGLEDEFAULT;
        float edgeCos0, edgeCos1, edgeCos2;

        // TODO: PAB: this initialization to removes 'possibly uninitialized variable' warnings
        edgeCos0 = edgeCos1 = edgeCos2 = 0.0f;

        if (unitType == UNITTYPE_OLDTRIANGLE)
        {
            // For "old triangles" just copy the upper nibble of the unitflags onto the triangle flags
            // and turn off the "edgecos" flag.   TODO: deprecate and remove OLDTRIANGLE.

            triflags = (uint32_t)(dataStream[0] & GPInstance::FLAG_TRIANGLEOLDMASK);
        }
        else if (dataStream[0] & UNITFLAG_EDGEANGLE)
        {
            edgeCos0 = DecodeEdgeCos((uint32_t)(edge[0] & EDGEFLAG_ANGLEMASK));
            edgeCos1 = DecodeEdgeCos((uint32_t)(edge[1] & EDGEFLAG_ANGLEMASK));
            edgeCos2 = DecodeEdgeCos((uint32_t)(edge[2] & EDGEFLAG_ANGLEMASK));
            triflags = ComputeTriangleFlags(edge[0], edge[1], edge[2], clusterParams.mFlags);
        }

        // TODO: PAB: consider edgeCos vector construction - change initialization parameter type
        // TODO: TP Volume ref
        instances[0].Initialize(v[0], v[1], v[2], 0.0f, triflags, edgeCos0, edgeCos1, edgeCos2,
            0, (groupId | (surfaceId<<16)), normal);
    }
    else if (triCount == 2)        // Quad
    {
        // Decompress the vertices
        Get4Vertices(v, vIndex[0], vIndex[1], vIndex[2], vIndex[3], clusterParams.mVertexCompressionGranularity);

        // generate an AABBox from the vertices
        AABBox unitBBox(Min(Min(v[0], v[1]), Min(v[2], v[3])), Max(Max(v[0], v[1]), Max(v[2], v[3])));

        // if not overlapping, do not generate quad instance
        if(!bbox.Overlaps(unitBBox))
        {
            numPrimitivesInUnit = 0;
            return size;
        }

        // transform the vertices into world space
        v[0] = TransformPoint(v[0], *transform);
        v[1] = TransformPoint(v[1], *transform);
        v[2] = TransformPoint(v[2], *transform);
        v[3] = TransformPoint(v[3], *transform);

        // generate normals for the triangles
        rwpmath::Vector3 normal0 = Normalize(Cross((v[1] - v[0]), (v[2] - v[0])));
        rwpmath::Vector3 normal1 = Normalize(Cross((v[2] - v[3]), (v[1] - v[3])));

        uint32_t triflags0 = GPInstance::FLAG_TRIANGLEDEFAULT;
        uint32_t triflags1 = GPInstance::FLAG_TRIANGLEDEFAULT;
        float edgeCos0, edgeCos1, edgeCos2, edgeCos3, edgeCos4, edgeCos5;

        // TODO: PAB: this initialization to removes 'possibly uninitialized variable' warnings
        edgeCos0 = edgeCos1 = edgeCos2 = edgeCos3 = edgeCos4 = edgeCos5 = 0.0f;

        if (dataStream[0] & UNITFLAG_EDGEANGLE)
        {
            int8_t innerFlags;

            // Compute the edge cosine and flags for the interior edge.
            float innerEdgeCos = ComputeEdgeCos(innerFlags, v[0], v[1], v[2], v[3]);

            // decode edgecos and flags of FIRST TRIANGLE
            edgeCos0  = DecodeEdgeCos((uint32_t)(edge[0] & EDGEFLAG_ANGLEMASK));
            edgeCos1  = innerEdgeCos;
            edgeCos2  = DecodeEdgeCos((uint32_t)(edge[2] & EDGEFLAG_ANGLEMASK));
            triflags0 = ComputeTriangleFlags(edge[0], static_cast<uint8_t>
                ((edge[1] & EDGEFLAG_VERTEXDISABLE) | innerFlags), edge[2], clusterParams.mFlags);

            // decode edgecos and flags of SECOND TRIANGLE
            edgeCos3  = DecodeEdgeCos((uint32_t)(edge[3] & EDGEFLAG_ANGLEMASK)); 
            edgeCos4  = innerEdgeCos;
            edgeCos5  = DecodeEdgeCos((uint32_t)(edge[1] & EDGEFLAG_ANGLEMASK));
            triflags1 = ComputeTriangleFlags(edge[3], static_cast<uint8_t>
                ((edge[2] & EDGEFLAG_VERTEXDISABLE) | innerFlags), edge[1], clusterParams.mFlags);
        }

        // TODO: TP Volume ref
        // FIRST TRIANGLE (0,1,2)
        instances[0].Initialize(v[0], v[1], v[2], 0.0f, triflags0, edgeCos0, edgeCos1, edgeCos2,
            0, (groupId | (surfaceId<<16)), normal0);

        // SECOND TRIANGLE (3,2,1)
        instances[1].Initialize(v[3], v[2], v[1], 0.0f, triflags1, edgeCos3, edgeCos4, edgeCos5,
            0, (groupId | (surfaceId<<16)), normal1);
    }
    else
    {
        EA_FAIL_MSG(("Trilist size > 2 not implemented yet."));
    }

    numPrimitivesInUnit = triCount;
    return size;
}


/**
\brief Set the vertex offset.

The offset is only relevant to clusters with a vertex compression mode of VERTICES_16BIT_COMPRESSED.

\param clusterOffset                The vertex offset.
*/
void
ClusteredMeshCluster::SetVertexOffset(const rw::collision::ClusteredMeshCluster::Vertex32 clusterOffset)
{
    if (compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED)
    {
        CompressedVertexDataUnion vdUnion;
        vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
        rw::collision::ClusteredMeshCluster::Vertex32 *offsetData = const_cast<rw::collision::ClusteredMeshCluster::Vertex32 *>(vdUnion.m_asVertex32Ptr);
        offsetData[0] = clusterOffset;
    }
}


/**
\brief Set the next vertex.

Methods adds a vertex to the Clusters vertex collection

\param v                            The vertex to add to the cluster
\param vertexCompressionGranularity The vertex compression granularity
*/
void
ClusteredMeshCluster::SetVertex(rwpmath::Vector3::InParam v,
                                const float vertexCompressionGranularity)
{
    if (compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED)
    {
        // Get the vertex offset
        CompressedVertexDataUnion vdUnion;
        vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
        const rw::collision::ClusteredMeshCluster::Vertex32 *clusterOffset = vdUnion.m_asVertex32Ptr;

        // Get start of vertex array
        rw::collision::ClusteredMeshCluster::Vertex16 *compressedVertexData = const_cast<rw::collision::ClusteredMeshCluster::Vertex16 *>(vdUnion.m_asVertex16Ptr);

        // Advance the pointer past the first 12 bytes which hold the offset data
        compressedVertexData += 2;

        // Advance the pointer to the first free vertex
        compressedVertexData += vertexCount;

        // Write the compressed vertex
        compressedVertexData->x = (uint16_t)((int32_t)( v.GetX() / vertexCompressionGranularity ) - clusterOffset->x);
        compressedVertexData->y = (uint16_t)((int32_t)( v.GetY() / vertexCompressionGranularity ) - clusterOffset->y);
        compressedVertexData->z = (uint16_t)((int32_t)( v.GetZ() / vertexCompressionGranularity ) - clusterOffset->z);

#if defined EA_ASSERT_ENABLED
        rwpmath::Vector3 c;
        c.X() = (compressedVertexData->x + clusterOffset->x) * vertexCompressionGranularity;
        c.Y() = (compressedVertexData->y + clusterOffset->y) * vertexCompressionGranularity;
        c.Z() = (compressedVertexData->z + clusterOffset->z) * vertexCompressionGranularity;
        EA_ASSERT_MSG(IsSimilar(v, c, 2.0f*vertexCompressionGranularity), ("Bad vertex compression."));
#endif
    }
    else if (compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED )
    {
        // Get the start of the vertex array
        CompressedVertexDataUnion vdUnion;
        vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
        rw::collision::ClusteredMeshCluster::Vertex32 *compressedVertexData = const_cast<rw::collision::ClusteredMeshCluster::Vertex32 *>(vdUnion.m_asVertex32Ptr);

        // Advance the pointer to the first free vertex
        compressedVertexData += vertexCount;

        // Write the compressed vertex data
        compressedVertexData->x = (int32_t)( v.GetX() / vertexCompressionGranularity );
        compressedVertexData->y = (int32_t)( v.GetY() / vertexCompressionGranularity );
        compressedVertexData->z = (int32_t)( v.GetZ() / vertexCompressionGranularity );
    }
    else // if (g->compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED)
    {
        // Write the vertex data to the first free space
        vertexArray[vertexCount].Set(v.GetX(), v.GetY(), v.GetZ());
    }

    // Increment the vertex count
    ++vertexCount;
}


/**
\brief Set the next unit as a triangle.

Adds a triangle unit to the clusters collection of units

\param UnitParameters   The unit parameters
\param groupID          The group ID of the triangle
\param surfaceID        The surface ID of the triangle
\param v0               The index of the triangles first vertex
\param v1               The index of the triangles second vertex
\param v2               The index of the triangles third vertex
\param edgeCode0        The edge code of the first edge
\param edgeCode1        The edge code of the second edge
\param edgeCode2        The edge code of the third edge
*/
void
ClusteredMeshCluster::SetTriangle(const UnitParameters & unitParameters,
                                  const uint32_t groupID,
                                  const uint32_t surfaceID,
                                  const uint8_t v0,
                                  const uint8_t v1,
                                  const uint8_t v2,
                                  const uint8_t edgeCode0,
                                  const uint8_t edgeCode1,
                                  const uint8_t edgeCode2)
{
    // Get the unit data array
    uint8_t * unitData = UnitData();

    // Check the size of the unit data
    EA_ASSERT_MSG((reinterpret_cast<uint8_t*>(this) + totalSize) >= unitData + unitDataSize + 4, ("Unit data does not fit into cluster"));

    // Get the unit code
    const uint8_t unitCode = ClusteredMeshCluster::GetUnitCode(UNITTYPE_TRIANGLE,
                                                               unitParameters.unitFlagsDefault,
                                                               groupID,
                                                               surfaceID);

    // Set the unit code
    unitData[unitDataSize++] = unitCode;

    // Set the vertex codes
    unitData[unitDataSize++] = v0;
    unitData[unitDataSize++] = v1;
    unitData[unitDataSize++] = v2;

    if (unitCode & rw::collision::UNITFLAG_EDGEANGLE)
    {
        // Check the size of the unit data
        EA_ASSERT_MSG((reinterpret_cast<uint8_t*>(this) + totalSize) >= (unitData + (unitDataSize + 3)), ("Unit data does not fit into cluster."));

        // compute edge bytes
        unitData[unitDataSize++] = edgeCode0;
        unitData[unitDataSize++] = edgeCode1;
        unitData[unitDataSize++] = edgeCode2;
    }

    // Set the group and surface IDs
    SetGroupAndSurfaceID(unitCode,
                         groupID,
                         unitParameters.groupIDSize,
                         surfaceID,
                         unitParameters.surfaceIDSize);

    // Increment the unit count
    ++unitCount;
}


/**
\brief Set the next unit as a quad.

Adds a quad unit to the clusters collection of units

\param unitParameters   The unit parameters
\param groupID          The group ID of the triangle
\param surfaceID        The surface ID of the triangle
\param v0               The index of the quad's first vertex
\param v1               The index of the quad's second vertex
\param v2               The index of the quad's third vertex
\param v3               The index of the quad's fourth vertex
\param edgeCode0        The edge code of the first edge
\param edgeCode1        The edge code of the second edge
\param edgeCode2        The edge code of the third edge
\param edgeCode3        The edge code of the fourth edge
*/
void
ClusteredMeshCluster::SetQuad(const UnitParameters & unitParameters,
                              const uint32_t groupID,
                              const uint32_t surfaceID,
                              const uint8_t v0,
                              const uint8_t v1,
                              const uint8_t v2,
                              const uint8_t v3,
                              const uint8_t edgeCode0,
                              const uint8_t edgeCode1,
                              const uint8_t edgeCode2,
                              const uint8_t edgeCode3)
{
    // Get the unit data array
    uint8_t * unitData = UnitData();

    // Check the size of the unit data
    EA_ASSERT_MSG((reinterpret_cast<uint8_t*>(this) + totalSize) >= unitData + unitDataSize + 5, ("Unit data does not fit into cluster"));

    // Get the unit code
    const uint8_t unitCode = ClusteredMeshCluster::GetUnitCode(UNITTYPE_QUAD,
                                                               unitParameters.unitFlagsDefault,
                                                               groupID,
                                                               surfaceID);

    // Set the unit code
    unitData[unitDataSize++] = unitCode;

    // Set the vertex indices
    unitData[unitDataSize++] = v0;
    unitData[unitDataSize++] = v1;
    unitData[unitDataSize++] = v2;
    unitData[unitDataSize++] = v3;

    if (unitCode & rw::collision::UNITFLAG_EDGEANGLE)
    {
        // Check the size of the unit data
        EA_ASSERT_MSG((reinterpret_cast<uint8_t*>(this) + totalSize) >= unitData + unitDataSize + 4, ("Unit data does not fit into cluster"));

        // Set the edge bytes
        unitData[unitDataSize++] = edgeCode0;
        unitData[unitDataSize++] = edgeCode1;
        unitData[unitDataSize++] = edgeCode2;
        unitData[unitDataSize++] = edgeCode3;
    }

    // Set the group and surface IDs
    SetGroupAndSurfaceID(unitCode,
                         groupID,
                         unitParameters.groupIDSize,
                         surfaceID,
                         unitParameters.surfaceIDSize);

    // Increment the unit count
    ++unitCount;
}


/**
\brief Set the current units group and surface IDs.

Adds a quad unit to the clusters collection of units

\param unitCode         The unit code of the current unit
\param groupID          The group ID of the current unit
\param groupSize        The group ID size
\param surfaceID        The surface ID of the current unit
\param surfaceSize      The surface ID size
*/
void
ClusteredMeshCluster::SetGroupAndSurfaceID(const uint8_t unitCode,
                                           const uint32_t groupID,
                                           const uint8_t groupSize,
                                           const uint32_t surfaceID,
                                           const uint8_t surfaceSize)
{
    // Get the unit data array
    uint8_t * unitData = UnitData();

    if (unitCode & rw::collision::UNITFLAG_GROUPID)
    {
        // Check the size of the unit data
        EA_ASSERT_MSG((reinterpret_cast<uint8_t*>(this) + totalSize) >= unitData + unitDataSize + (groupSize == 2 ? 2 : 1), ("Unit data does not fit into cluster"));

        unitData[unitDataSize++] = static_cast<uint8_t>(groupID & 255);
        if (groupSize==2)
        {
            unitData[unitDataSize++] = static_cast<uint8_t>((groupID >> 8)&255);
        }
    }

    if (unitCode & rw::collision::UNITFLAG_SURFACEID)
    {
        // Check the size of the unit data
        EA_ASSERT_MSG((reinterpret_cast<uint8_t*>(this) + totalSize) >= unitData + unitDataSize + (surfaceSize == 2 ? 2 : 1), ("Unit data does not fit into cluster"));

        unitData[unitDataSize++] = static_cast<uint8_t>(surfaceID & 255);
        if (surfaceSize==2)
        {
            unitData[unitDataSize++] = static_cast<uint8_t>((surfaceID >> 8)&255);
        }
    }
}


/**
\brief Calculates a unit code given a default flag and ID descriptors.

\param unitType         The unit type.
\param flagsDefault     The default value of the unit flags.
\param groupID          The group ID of the current unit.
\param surfaceID        The surfaceID of the current unit.

\return                 The unit code.
*/
uint8_t
ClusteredMeshCluster::GetUnitCode(const uint8_t unitType,
                                  const uint8_t flagsDefault,
                                  const uint32_t groupID,
                                  const uint32_t surfaceID)
{
    // TODO: This would be safer as bitwise or rather addition, otherwise its easy
    // to accidentally set flagsDefault to UNITTYPE_TRIANGLE and get nonsense out.
    uint8_t unitCode = static_cast<uint8_t>(unitType + flagsDefault);

    // If this triangle group/surface id matches the default, do not store it
    if ((unitCode & rw::collision::UNITFLAG_GROUPID) && groupID == DEFAULT_GROUPID)
    {
        unitCode = static_cast<uint8_t>(unitCode & ~rw::collision::UNITFLAG_GROUPID);
    }
    if ((unitCode & rw::collision::UNITFLAG_SURFACEID) && surfaceID == DEFAULT_SURFACEID)
    {
        unitCode = static_cast<uint8_t>(unitCode & ~rw::collision::UNITFLAG_SURFACEID);
    }

    return unitCode;
}

/**
\brief Calculates the size required to store a collection of units.

The unit is described by a number of 

Adds a quad unit to the clusters collection of units

\param unitCode         The unit code of the current unit
\param groupID          The group ID of the current unit
\param groupSize        The group ID size
\param surfaceID        The surface ID of the current unit
\param surfaceSize      The surface ID size
*/
uint16_t
ClusteredMeshCluster::GetUnitDataSize(const uint16_t triangleUnitCount,
                                      const uint16_t quadUnitCount,
                                      const uint16_t edgeCosineCount,
                                      const uint16_t groupIDCount,
                                      const uint16_t groupIDSize,
                                      const uint16_t surfaceIDCount,
                                      const uint16_t surfaceIDSize)
{
    uint16_t size = 0;

    // Size of the unit codes, each unit has a code (uint8_t).
    size = static_cast<uint16_t>(size + triangleUnitCount + quadUnitCount);

    // Size of the standard unit data
    // Size of standard triangle unit = 3 vertex indices(uint_8)
    size = static_cast<uint16_t>(size + 3 * triangleUnitCount);
    // Size of standard triangle unit = 4 vertex indices(uint_8)
    size = static_cast<uint16_t>(size + 4 * quadUnitCount);

    // Size of the edge cosine data
    size = static_cast<uint16_t>(size + edgeCosineCount);

    // Size of the group and surface ID data
    size = static_cast<uint16_t>(size + groupIDCount * groupIDSize);
    size = static_cast<uint16_t>(size + surfaceIDCount * surfaceIDSize);

    return size;
}


/**
\brief Gets the size of the vertex data given a vertex count and compression mode.

Methods adds a vertex to the Clusters vertex collection

\param vertexCount              The number of vertices
\param vertexCompressionMode    The vertex compression mode
*/
uint16_t
ClusteredMeshCluster::GetVertexDataSize(const uint16_t vertexCount,
                                        const uint16_t vertexCompressionMode)
{
    // Size of the vertices
    if ( vertexCompressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED )
    {
        // Size = the vertex offset + the vertices
        uint32_t bytes = ( 3 * sizeof(int32_t) ) +
            ( sizeof(rw::collision::ClusteredMeshCluster::Vertex16) * vertexCount );
        bytes = EA::Physics::SizeAlign<uint32_t>( bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
        return static_cast<uint16_t>(bytes);
    }
    else if ( vertexCompressionMode == rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED )
    {
        // Size = the vertices
        uint32_t bytes =( sizeof(rw::collision::ClusteredMeshCluster::Vertex32) * vertexCount );
        bytes = EA::Physics::SizeAlign<uint32_t>( bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
        return static_cast<uint16_t>(bytes);
    }
    else
    {
        // Size = the vertices
        uint16_t quadwords = vertexCount;
        return static_cast<uint16_t>(16u * quadwords);
    }
}


/**
\brief Gets the size of a cluster described by the parameters.

\param unitType         The type of the unit
\param groupID          The group ID of the triangle
\param surfaceID        The surface ID of the triangle
\param unitParameters   The unit parameters

\return the size in bytes of the clusters resource requirements
*/
uint32_t
ClusteredMeshCluster::GetUnitSize(const uint8_t unitType,
                                  const UnitParameters & unitParameters,
                                  const uint32_t groupID,
                                  const uint32_t surfaceID)
{
    // Sizes of standard components of a unit.
    const uint32_t unitCodeSize = 1u;
    const uint32_t vertexIDSize = (rw::collision::UNITTYPE_QUAD == unitType) ? 4u : 3u;

    // Sizes of optional components of a unit.
    const uint32_t edgeCosineSize = (rw::collision::UNITTYPE_QUAD == unitType) ? 4u : 3u;

    // Sum the standard components
    uint32_t unitSize = unitCodeSize + vertexIDSize;

    // If the unit contains edge cosines
    if (unitParameters.unitFlagsDefault & rw::collision::UNITFLAG_EDGEANGLE)
    {
        unitSize += edgeCosineSize;
    }

    // If the unit contains a group ID
    if ((unitParameters.unitFlagsDefault & rw::collision::UNITFLAG_GROUPID) && (groupID != DEFAULT_GROUPID))
    {
        unitSize += unitParameters.groupIDSize;
    }

    // If the unit contains a surface ID
    if ((unitParameters.unitFlagsDefault & rw::collision::UNITFLAG_SURFACEID) && (surfaceID != DEFAULT_SURFACEID))
    {
        unitSize += unitParameters.surfaceIDSize;
    }

    return unitSize;
}


/**
\brief Gets the size of a cluster described by the parameters.

\param parameters               The parameters used to describe the cluster

\return the size in bytes of the clusters resource requirements
*/
uint16_t
ClusteredMeshCluster::GetSize(const ClusterConstructionParameters & parameters)
{
    // Size of the vertex data
    const uint16_t vertexDataSize = ClusteredMeshCluster::GetVertexDataSize(parameters.mVertexCount, parameters.mVertexCompressionMode);

    // Size of the unit data
    const uint16_t unitDataSize = ClusteredMeshCluster::GetUnitDataSize(
        parameters.mTriangleUnitCount,
        parameters.mQuadUnitCount,
        parameters.mEdgeCosineCount,
        parameters.mGroupIDCount,
        parameters.mGroupIDSize,
        parameters.mSurfaceIDCount,
        parameters.mSurfaceIDSize);

    // Size of the ClusteredMeshCluster
    uint16_t classSize = sizeof(ClusteredMeshCluster);

    // The ClusteredMeshCluster contains an rwpmath::Vector3 entry. We have to determine
    // how this resource has been used.
    if (vertexDataSize + unitDataSize > sizeof(rwpmath::Vector3))
    {
        classSize = static_cast<uint16_t>(classSize + vertexDataSize + unitDataSize - sizeof(rwpmath::Vector3));
    }

    return classSize;
}


/**
\brief Initializes a ClusteredMeshCluster.

Initializes a cluster which is described by the ClusterConstructionParamaters.
The cluster will contain the required resources to store the complete collection
of units and vertices indicated by the parameters, however it will not contain
any vertices or units, these must be set using the SetVertex and SetTriangle/Quad
methods.

\param buffer                   The resource into which the cluster will be initialized
\param parameters               The parameters used to describe the cluster

\return Initialized cluster.
*/
ClusteredMeshCluster *
ClusteredMeshCluster::Initialize(void * buffer,
                                 const ClusterConstructionParameters & parameters)
{
    // Assert the correct alignment
    rwcASSERTALIGN(buffer, rwcCLUSTEREDMESHCLUSTER_ALIGNMENT);

    // Use in-place new to construct a new ClusteredMeshCluster
    return new (buffer) ClusteredMeshCluster(parameters);
}



/**
\brief ClusteredMeshCluster constructor.

Initializes the clustered mesh cluster using a construction parameters struct.

\param parameters               The parameters used to describe the cluster
*/
ClusteredMeshCluster::ClusteredMeshCluster(const ClusterConstructionParameters & parameters)
{
    totalSize = ClusteredMeshCluster::GetSize(parameters);

    unitCount = 0;

    unitDataSize = 0;

    vertexCount = 0;

    // Normals are not supported
    normalCount = 0;

    compressionMode = parameters.mVertexCompressionMode;

    if ( compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED )
    {
        uint32_t bytes = 3*sizeof(int32_t) + sizeof(rw::collision::ClusteredMeshCluster::Vertex16) * parameters.mVertexCount;
        bytes = EA::Physics::SizeAlign<uint32_t>( bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
        normalStart = static_cast<uint16_t>( bytes / rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
        unitDataStart = (uint16_t) (normalStart + normalCount);
    }
    else if ( compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED )
    {
        uint32_t bytes = sizeof(rw::collision::ClusteredMeshCluster::Vertex32) * parameters.mVertexCount;
        bytes = EA::Physics::SizeAlign<uint32_t>( bytes, rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
        normalStart = static_cast<uint16_t>( bytes / rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT );
        unitDataStart = (uint16_t) (normalStart + normalCount);
    }
    else // if (cluster->compressionMode == rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED)
    {
        normalStart = parameters.mVertexCount;
        unitDataStart = (uint16_t) (parameters.mVertexCount + normalCount);
    }
}

/**
NOTE: This method is currently disabled/removed for SPU builds since the Volume system is
unusable on SPU and the method has a dependency on the Volume system. It is unknown how
much work is required to fix this issue, as it involves dealing with rwcore, but at the
time of writing it was decided that the work be carried out at a later date.
*/
#if !defined(EA_PLATFORM_PS3_SPU)
/**
\brief Gets a triangle volume referred to by a a unit offset and triangle offset.

\param triangleVolume the destination triangle volume
\param unitOffset the offset of the unit containing the triangle.
\param triangleIndex the index of the triangle withing the unit.
\param clusterParameters the parameters which describe this cluster.
*/
void
ClusteredMeshCluster::GetTriangleVolume(rw::collision::TriangleVolume & triangleVolume,
                                        const uint32_t unitOffset,
                                        const uint32_t triangleIndex,
                                        const rw::collision::ClusterParams & clusterParameters)
{

    // Setup a cluster triangle iterator to address the single triangle referred to by the indices/offsets
    ClusterTriangleIterator<> triangleIterator(*this, clusterParameters, unitOffset, 1u, triangleIndex + 1u);

    // Set the triangle vertex positions
    rwpmath::Vector3 v0(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 v1(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 v2(rwpmath::GetVector3_Zero());

    triangleIterator.GetVertices(v0, v1, v2);
    triangleVolume.SetPoints(v0, v1, v2);

    // Set the triangle Flags and IDs
    InitializeTriangleVolumeDetails(
        triangleVolume,
        triangleIterator);
}


/**
\brief Gets a triangle vertex indices referred to by a a unit offset and triangle offset.

\param v0 index of first triangle vertex.
\param v1 index of second triangle vertex.
\param v2 index of third triangle vertex.
\param unitOffset the offset of the unit containing the triangle.
\param triangleIndex the index of the triangle withing the unit.
\param clusterParameters the parameters which describe this cluster.
*/
void
ClusteredMeshCluster::GetTriangleVertexIndices(uint8_t & v0,
                                               uint8_t & v1,
                                               uint8_t & v2,
                                               const uint32_t unitOffset,
                                               const uint32_t triangleIndex,
                                               const rw::collision::ClusterParams & clusterParameters) const
{
    // Setup a cluster triangle iterator to address the single triangle referred to by the indices/offsets
    ClusterTriangleIterator<> triangleIterator(*this, clusterParameters, unitOffset, 1u, triangleIndex + 1u);

    // Get the vertex indices
    triangleIterator.GetVertexIndices(v0, v1, v2);
}

#endif // !defined(EA_PLATFORM_PS3_SPU)

} // namespace collision
} // namespace rw
