// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHCLUSTER_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHCLUSTER_H

/*************************************************************************************************************

File: rwcclusteredmesh.hpp

Purpose: Compressed aggregate of triangles and quads with KDTree spatial map.

*/

#include "rw/collision/volume.h"
#if !defined(EA_PLATFORM_PS3_SPU)
#include "procedural.h"

// Forward Declarations
namespace rw
{
namespace collision
{
    class TriangleVolume;
}
}
#endif // !defined(EA_PLATFORM_PS3_SPU)

// Alignment must be 16 to support loading legacy data
#define rwcCLUSTEREDMESHCLUSTER_ALIGNMENT 16
#define rwcCLUSTEREDMESHCLUSTER_VERTEXDATA_ALIGNMENT 16

namespace rw
{
namespace collision
{

    // Forward declare ClusteredMesh in the rw::collision namespace so
    // that we can use it in the EA_SERIALIZATION_CLASS_* macros
    class ClusteredMeshCluster;

} // namespace collision
} // namespace rw


// We need to specify the class serialization version prior to the class definition
// due to a problem with ps2 gcc.
EA_SERIALIZATION_CLASS_VERSION(rw::collision::ClusteredMeshCluster, 5)
// These macro provide the type name used in text-based archives' serialization.
EA_SERIALIZATION_CLASS_NAME(rw::collision::ClusteredMeshCluster, "rw::collision::ClusteredMeshCluster")


namespace rw
{
namespace collision
{


///Enums for the clustered mesh flags
enum CMFlags
{
    CMFLAG_INT16VERTEX = 1,            ///< vertices are 6 bytes, using UInt16 offsets
    CMFLAG_INT16NORMAL = 2,            ///< normals are 6 bytes each
    CMFLAG_20BITCLUSTERINDEX = 4,   ///< the kdtree index uses 20 bits for the cluster index (16 if false) the value of this flag must be 4 (because 16 + 4 = 20).
    CMFLAG_ONESIDED = 16,           ///< disallow collision with back side of triangle faces and edges the value of this must be 16 (same as VOLUMEFLAG_TRIANGLEONESIDED).

    CMFLAG_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


///Enums for unit type and unit flags. The cluster stores units.  Each unit has a one-byte prefix indicating the type and flags.
enum UnitTypeAndFlags
{
    UNITTYPE_OLDTRIANGLE = 0,                    ///< same as triangle, but the unitflags area is used for triangle edge/face flags
    UNITTYPE_TRIANGLE = 1,                    ///< unit is a triangle acw
    UNITTYPE_QUAD = 2,                        ///< a quad, three verts, the fourth is implied d = a - b + c
    UNITTYPE_TRILIST = 3,                    ///< several triangles, count-byte prefix
    //  add more unit types here, up to 15.
    UNITTYPE_MASK = 15,                        ///<
    UNITFLAG_NORMAL = 16,                    //< each unit is followed by a normal byte
    UNITFLAG_EDGEANGLE = 32,                    ///< each triangle is followed by 3 angle bytes, quads followed by 4
    UNITFLAG_GROUPID = 64,                    ///< each unit is followed by a group id (mGroupIdSize size)
    UNITFLAG_SURFACEID = 128,                    ///< each unit is followed by a surface id (mSurfaceIdSize size)
    UNITFLAG_USEOLDTRI = 255,                    ///<
    UNITFLAG_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


/// Enums for decoding the edgecos bytes in the clustered mesh edge data.
enum UnitEdgeFlags
{
    EDGEFLAG_ANGLEZERO = 26,        ///< this is the value to set the angle if you want to disable the edge
    EDGEFLAG_ANGLEMASK = 0x1F,        ///< mask of the bits used by the angle number 0..26 (higher number means smaller angle).
    EDGEFLAG_EDGECONVEX = 0x20,        ///< this bit is set for a convex edge, cleared for a concave edge.
    EDGEFLAG_VERTEXDISABLE = 0x40,    ///< set to disable collision with the vertex, clear to allow collisions
    EDGEFLAG_EDGEUNMATCHED = 0x80    ///< set if no match was found for this edge (to compute the edge angle)
};


///Enums for special tags for common normals.  Note values 0..249 are reserved for normal lookup table indices.
enum SpecialNormalTags
{
    NORMAL_NA = -1,
    NORMAL_POSX = 250,            ///< normal is (1, 0, 0)
    NORMAL_NEGX = 251,            ///< normal is (-1, 0, 0)
    NORMAL_POSY = 252,            ///< normal is (0, 1, 0)
    NORMAL_NEGY = 253,            ///< normal is (0, -1, 0)
    NORMAL_POSZ = 254,            ///< normal is (0, 0, 1)
    NORMAL_NEGZ = 255,            ///< normal is (0, 0, -1)
    NORMAL_INVALID = 999,
    NORMAL_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


/** \brief A collection of parameters that are needed to decode the values in a cluster.

This was moved out of the clustered mesh itself to enable the clusters to be uploaded an spu on their own
**/
struct ClusterParams
{
    float mVertexCompressionGranularity;    ///< The vertex compression granularity
    uint16_t  mFlags;                            ///< Mesh-wide flags /see CMFlags
    uint8_t   mGroupIdSize;                        ///< The group ID size in bytes
    uint8_t   mSurfaceIdSize;                    ///< The surface ID size in bytes
};

float DecodeEdgeCos(uint32_t B);

uint32_t  ComputeTriangleFlags(uint8_t ec0, uint8_t ec1, uint8_t ec2, uint16_t meshFlags);

float ComputeEdgeCos(int8_t &convexFlag, rwpmath::Vector3::InParam v0, rwpmath::Vector3::InParam v1,
                     rwpmath::Vector3::InParam v2, rwpmath::Vector3::InParam v3);


/*
\brief A collection of unit parameters
**/
struct UnitParameters
{
    UnitParameters()
        : unitFlagsDefault(0)
        , groupIDSize(0)
        , surfaceIDSize(0)
    {
    }

    uint8_t unitFlagsDefault;
    uint8_t groupIDSize;
    uint8_t surfaceIDSize;
};


/*
\brief A collection of parameters that are needed to determine the memory requirements of the cluster
**/
struct ClusterConstructionParameters
{
    uint8_t mVertexCount;              // The vertex count
    uint8_t mVertexCompressionMode;    // The vertex compression mode
    uint16_t mTriangleUnitCount;       // The triangle unit count
    uint16_t mQuadUnitCount;           // The quad unit count
    uint16_t mEdgeCosineCount;         // The edge cosine count
    uint16_t mGroupIDCount;            // The group ID count
    uint16_t mGroupIDSize;             // The size of the groupID
    uint16_t mSurfaceIDCount;          // The surface ID count
    uint16_t mSurfaceIDSize;           // The size of the surfaceID

    ClusterConstructionParameters()
        : mVertexCount(0)
        , mVertexCompressionMode(0)
        , mTriangleUnitCount(0)
        , mQuadUnitCount(0)
        , mEdgeCosineCount(0)
        , mGroupIDCount(0)
        , mGroupIDSize(0)
        , mSurfaceIDCount(0)
        , mSurfaceIDSize(0)
    {
    }

    ClusterConstructionParameters(const ClusterConstructionParameters & other)
    {
        mVertexCount = other.mVertexCount;
        mVertexCompressionMode = other.mVertexCompressionMode;
        mTriangleUnitCount = other.mTriangleUnitCount;
        mQuadUnitCount = other.mQuadUnitCount;
        mEdgeCosineCount = other.mEdgeCosineCount;
        mGroupIDCount = other.mGroupIDCount;
        mGroupIDSize = other.mGroupIDSize;
        mSurfaceIDCount = other.mSurfaceIDCount;
        mSurfaceIDSize = other.mSurfaceIDSize;
    }
};

/**
\brief The cluster is used by the ClusteredMesh for storing compressed triangle data.

The cluster header is 16 bytes, followed by vertex list, normal list, and the unitData.

\member unitCount number of units
\member unitDataSize size of unit data in bytes
\member unitDataStart offset in quadwords from start of vertices to start of unit data
\member normalStart offset quadwords from start of vertices to start of normal array
\member vertexCount number of vertices
\member normalCount number of normals
\member vertexArray the list of vertices (followed by normals, and unitData)

when verts are not compressed, normalStart = vertexCount.  And when normal and vert are uncompressed,
unitDataStart = vertexCount + normalCount.
*/
class ClusteredMeshCluster
{
public:

    // Life Cycle API
    static uint16_t GetSize(const ClusterConstructionParameters & parameters);

    static ClusteredMeshCluster * Initialize(void * buffer,
                                             const ClusterConstructionParameters & parameters);

    static uint32_t GetUnitSize(const uint8_t unitType,
                                const UnitParameters & unitParameters,
                                const uint32_t groupID,
                                const uint32_t surfaceID);

    ClusteredMeshCluster()
    {

    };

    /// Compressed vertex data, with 16-bit xyz components
    enum
    {
        VERTICES_UNCOMPRESSED = 0,        ///<Uncompressed
        VERTICES_16BIT_COMPRESSED = 1,    ///<Compressed to 16 bit
        VERTICES_32BIT_COMPRESSED = 2    ///<Compressed to 32 bit
    };
    /// Used to indicate to templated methods that the compression mode is not known statically and 
    /// should be read from the cluster at runtime. Only for use as parameter to templated GetVertex() methods.
    static const uint8_t COMPRESSION_DYNAMIC = 255;

    /// The default unit surface ID
    static const uint8_t DEFAULT_GROUPID = 0u;
    /// The default unit group ID
    static const uint8_t DEFAULT_SURFACEID = 0u;
    /// The maximum number of vertices a cluster can contain
    static const uint32_t MAX_VERTEX_COUNT = 255u;

    /// Compressed vertex data, with 16-bit xyz components
    struct Vertex16
    {
        uint16_t x;
        uint16_t y;
        uint16_t z;
    };

    /// Granularised vertex data, with 32-bit xyz components
    struct Vertex32
    {
        int32_t x;
        int32_t y;
        int32_t z;
    };

    /// union to gain access to the vertex data using any form of compression
    union CompressedVertexDataUnion
    {
        const rwpmath::Vector3 *m_as_rwpmathVector3Ptr; ///< Vertex data as rwpmath::Vector3*
        const Vertex32         *m_asVertex32Ptr;        ///< Vertex data as ClusteredMeshCluster::Vertex32*
        const Vertex16         *m_asVertex16Ptr;        ///< Vertex data as ClusteredMeshCluster::Vertex16*
        const int32_t          *m_asInt32Ptr;           ///< Vertex data as int32_t*
    };

    void SetVertexOffset(const rw::collision::ClusteredMeshCluster::Vertex32 clusterOffset);

    void SetVertex(rwpmath::Vector3::InParam v,
                   const float vertexCompressionGranularity);

    void SetTriangle(const UnitParameters & unitParameters,
                     const uint32_t groupID,
                     const uint32_t surfaceID,
                     const uint8_t v0,
                     const uint8_t v1,
                     const uint8_t v2,
                     const uint8_t edgeCode0,
                     const uint8_t edgeCode1,
                     const uint8_t edgeCode2);

    void SetQuad(const UnitParameters & unitParameters,
                 const uint32_t groupID,
                 const uint32_t surfaceID,
                 const uint8_t v0,
                 const uint8_t v1,
                 const uint8_t v2,
                 const uint8_t v3,
                 const uint8_t edgeCode0,
                 const uint8_t edgeCode1,
                 const uint8_t edgeCode2,
                 const uint8_t edgeCode3);

    RW_COLLISION_FORCE_INLINE rwpmath::Vector3 GetVertex(uint8_t vertid, const float &vertexGranularity) const;
    RW_COLLISION_FORCE_INLINE void Get3Vertices(rwpmath::Vector3 *out, uint8_t v0, uint8_t v1, uint8_t v2, const float &vertexGranularity) const;
    RW_COLLISION_FORCE_INLINE void Get4Vertices(rwpmath::Vector3 *out, uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &vertexGranularity) const;

    template <uint8_t COMPRESSION> 
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3 GetVertexBase(uint8_t vertid, const float &vertexGranularity) const;
    template <uint8_t COMPRESSION> 
    RW_COLLISION_FORCE_INLINE void Get3VerticesBase(rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, 
        uint8_t v0, uint8_t v1, uint8_t v2, const float &vertexGranularity) const;
    template <uint8_t COMPRESSION> 
    RW_COLLISION_FORCE_INLINE void Get4VerticesBase(rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, rwpmath::Vector3 & out3, 
        uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &vertexGranularity) const;

    RW_COLLISION_FORCE_INLINE uint32_t GetUnitSize(uint32_t offset, const ClusterParams & clusterParams);
    RW_COLLISION_FORCE_INLINE uint32_t GetUnitType(uint32_t offset);

    RW_COLLISION_FORCE_INLINE uint8_t *UnitData();
    RW_COLLISION_FORCE_INLINE const uint8_t *UnitData() const;
    RW_COLLISION_FORCE_INLINE rwpmath::Vector3 *NormalArray();
    inline uint32_t NumVolumesInUnit(uint32_t offset);
    uint32_t UnitGetOverlappingGPInstances(uint32_t offset, const AABBox &bbox, const rwpmath::Matrix44Affine *transform,
                                           GPTriangle *instances, uint32_t &numPrimitivesInUnit, const ClusterParams & clusterParams);
    inline void GetGroupAndSurfaceId(uint32_t offset, const ClusterParams & clusterParams, uint32_t & groupID, uint32_t & surfaceID);

    /**
    \brief Serializes the ClusteredMeshCluster.

    \note This is not intended to be used directly and is only intended to be used by higher level
    classes such as the TriangleClusterProcedural

    \param ar serialization archive
    \param version the version of the clustered mesh cluster
    */
    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
        // Serialize the counts, sizes and starts
        ar & EA_SERIALIZATION_NAMED_VALUE(unitCount);
        ar & EA_SERIALIZATION_NAMED_VALUE(unitDataSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(unitDataStart);
        ar & EA_SERIALIZATION_NAMED_VALUE(normalStart);
        ar & EA_SERIALIZATION_NAMED_VALUE(totalSize);
        ar & EA_SERIALIZATION_NAMED_VALUE(compressionMode);
        ar & EA_SERIALIZATION_NAMED_VALUE(vertexCount);
        ar & EA_SERIALIZATION_NAMED_VALUE(normalCount);

        // Serialize the vertices
        if (compressionMode == ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED)
        {
            // Access the vertex array as a collection of rwpmath::Vector3
            rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
            vdUnion.m_as_rwpmathVector3Ptr = vertexArray;

            // Access the vertex array as a collection of uint32_t
            uint32_t* vertexArrayHeader = const_cast<uint32_t *>(reinterpret_cast<const uint32_t *>(vdUnion.m_asInt32Ptr));
            // Serialize the vertex array header
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArrayHeader, 3u);

            // Access the vertex array as a collection of uint16_t and offset the pointer to the end of the header
            uint16_t* vertexArray = const_cast<uint16_t *>(&vdUnion.m_asVertex16Ptr[0].x) + 3 * 2;
            // Serialize the vertices
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, vertexCount * 3u);
        }
        else if (compressionMode == ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED)
        {
            // Access the vertex array as a collection of rwpmath::Vector3
            rw::collision::ClusteredMeshCluster::CompressedVertexDataUnion vdUnion;
            vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
            // Access the vertex array as a collection of int32_t
            int32_t* vertexArray = const_cast<int32_t *>(vdUnion.m_asInt32Ptr);
            // Serialize the vertices
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, vertexCount * 3u);
        }
        else
        {
            if (version == 1)
            {
                // Using fpu math it is not valid to serialize the normals with the vertices
                // as there will be padding between the two arrays.
                uint32_t size = uint32_t(vertexCount + normalCount);
                rwpmath::Vector3 * vertexAndNormalArray = vertexArray;
                // Serialize the vertices and normals
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexAndNormalArray, size);
            }
            else
            {
                // Archive the vertices
                ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(vertexArray, vertexCount);
            }
        }

        // Serialize Normals
        if ((compressionMode != ClusteredMeshCluster::VERTICES_UNCOMPRESSED) || (version > 1))
        {
            // normalStart is a quad-word offset
            rwpmath::Vector3* normalArray =
                reinterpret_cast<rwpmath::Vector3*>(reinterpret_cast<uintptr_t>(vertexArray) + normalStart * 16);
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(normalArray, normalCount);
        }

        // Serialize Unit data
        // unitDataStart is a quad-word offset
        uint8_t* unitData = reinterpret_cast<uint8_t*>(vertexArray) + unitDataStart * 16;
        // Serialize the normals
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(unitData, unitDataSize);
    }

#if !defined(EA_PLATFORM_PS3_SPU)
    void GetTriangleVolume(rw::collision::TriangleVolume & triangleVolume,
                           const uint32_t unitOffset,
                           const uint32_t triangleIndex,
                           const rw::collision::ClusterParams & clusterParameters);
#endif // !defined(EA_PLATFORM_PS3_SPU)

    void GetTriangleVertexIndices(uint8_t & v0,
                                  uint8_t & v1,
                                  uint8_t & v2,
                                  const uint32_t unitOffset,
                                  const uint32_t triangleIndex,
                                  const rw::collision::ClusterParams & clusterParameters) const;

public:

    //  All the data in this class is public so that the MeshWorkingData class can access it.
    //  (Maybe it should just be a friend?)

    uint16_t unitCount;                    ///< The number of units in this cluster
    uint16_t unitDataSize;                ///< The size of the unit data
    uint16_t unitDataStart;                ///< The quad word offset to the beginning of the unit data
    uint16_t normalStart;                ///< The quad word offset to the beginning of the normal data
    uint16_t totalSize;                    ///< The total size of the entire cluster
    uint8_t vertexCount;                ///< The total number of vertices
    uint8_t normalCount;                ///< The total number of normal vectors
    uint8_t compressionMode;            ///< The compression mode
    uint8_t padding[3];                    ///< force 16 byte alignment for the vertex array
    rwpmath::Vector3 vertexArray[1];    ///< The first of the array of vertices, the rest are immediately after this class
private:

    static uint16_t GetVertexDataSize(const uint16_t vertexCount, const uint16_t vertexCompressionMode);

    static uint16_t GetUnitDataSize(const uint16_t triangleUnitCount, const uint16_t triangleQuadCount,
                                    const uint16_t edgeCosineCount, const uint16_t groupIDCount,
                                    const uint16_t groupIDSize, const uint16_t surfaceIDCount,
                                    const uint16_t surfaceIDSize);

    static uint8_t GetUnitCode(const uint8_t unitType,
                               const uint8_t unitFlagsDefault,
                               const uint32_t groupID,
                               const uint32_t surfaceID);

    ClusteredMeshCluster(const ClusterConstructionParameters & parameters);

    void SetGroupAndSurfaceID(const uint8_t unitCode,
                              const uint32_t groupID,
                              const uint8_t groupSize,
                              const uint32_t surfaceID,
                              const uint8_t surfaceSize);
};

inline float
DecodeEdgeCos(uint32_t B);

inline uint32_t
ComputeTriangleFlags(uint8_t ec0, uint8_t ec1, uint8_t ec2, uint16_t meshFlags);

inline float
ComputeEdgeCos(int8_t &convexFlag, rwpmath::Vector3::InParam v0, rwpmath::Vector3::InParam v1,
               rwpmath::Vector3::InParam v2, rwpmath::Vector3::InParam v3);
}
}

#endif //PUBLIC_RW_COLLISION_CLUSTEREDMESHCLUSTER_H
