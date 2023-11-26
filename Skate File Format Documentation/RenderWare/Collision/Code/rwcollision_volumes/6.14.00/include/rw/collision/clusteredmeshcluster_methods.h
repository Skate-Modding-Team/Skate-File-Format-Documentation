// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESHCLUSTER_METHODS_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESHCLUSTER_METHODS_H

/*************************************************************************************************************/

#include "clusteredmeshcluster.h"


namespace rw
{
namespace collision
{

#if defined(EA_PLATFORM_PS3_PPU)
// The version of gcc in SDK270 introduced a bug with vec_lvlx when used in the way in this header file.
// This causes the TestClusteredMesh unit tests to crash. This doesn't appear to affect lvrx.
// See https://ps3.scedev.net/support/issue/48182/_vec_lvlx_problems_with_gcc#n584985
// Sony's workaround is to cast the address to a uint32_t pointer, but without the ".x" member, so lines like
// vector signed int   vert_l      = (vector signed int)vec_lvlx(0,  &vertData[vertid].x);
// become
// vector signed int   vert_l      = (vector signed int)vec_lvlx(0,  (uint32_t *) &vertData[vertid]);
// but this is hard to express in a macro, so instead we'll use __builtin_altivec_lvlx which doesn't appear
// to be broken. The only downside is that it doesn't exist with the SN compiler.
#if defined(__SN_VER__)
#define clusteredmeshcluster_vec_lvlx(X,A) vec_lvlx(X, A)
#else
#define clusteredmeshcluster_vec_lvlx(X,A) __builtin_altivec_lvlx(X, A)
#endif
#endif  // defined(EA_PLATFORM_PS3_PPU)

inline float GetFloatPISquared()
{
    return 3.14159265358979323846f * 3.14159265358979323846f;
}

// Default implementation should not be called
template <uint8_t COMPRESSION>
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ClusteredMeshCluster::GetVertexBase(uint8_t /* vertid */, const float & /* vertexGranularity */) const
{
    EA_FAIL_MSG(("Unsupported clustered mesh compression type"));
    return rwpmath::GetVector3_Zero();
}

// Specialization for no compression
template <>
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ClusteredMeshCluster::GetVertexBase<ClusteredMeshCluster::VERTICES_UNCOMPRESSED>(uint8_t vertid, const float &/*vertexGranularity*/) const
{
    EA_ASSERT(vertid < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_UNCOMPRESSED);
    return vertexArray[vertid];
}

// Specialization for 16-bit compression
template <>
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ClusteredMeshCluster::GetVertexBase<ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(uint8_t vertid, const float &vertexGranularity) const
{
    EA_ASSERT(vertid < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_16BIT_COMPRESSED);
    CompressedVertexDataUnion vdUnion;
    vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
    const int32_t *vertexOffsetData = vdUnion.m_asInt32Ptr;
    const Vertex16 *vertData = vdUnion.m_asVertex16Ptr + 2; // skip the first 6 bytes taken up by cluster offset

#if defined(EA_PLATFORM_PS3_PPU)

        // first vector in cluster - is quadword aligned
        EA_ASSERT((reinterpret_cast<int>(vertexOffsetData) & 0xf) == 0);
        vector float zero        = (vector float)vec_splat_s32(0);
        vector signed int   vert_offset = vec_ld(0, vertexOffsetData);
        vector signed int   vert_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vertData[vertid].x);
        vector signed int   vert_r      = (vector signed int)vec_lvrx(16, &vertData[vertid].x);
        vector signed int   vert_i      = vec_or(vert_l, vert_r);
        vert_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_i);
        vert_i      = vec_add(vert_i, vert_offset);
        vector float floatvert   = vec_ctf(vert_i, 0);
        vector float vertGran    = (vector float)clusteredmeshcluster_vec_lvlx(0, &vertexGranularity);
        vertGran    = vec_splat(vertGran, 0);
    return rwpmath::Vector3(vec_madd(floatvert, vertGran, zero));

#elif defined(EA_PLATFORM_XENON)

        // first vector in cluster - is quadword aligned
        EA_ASSERT((reinterpret_cast<int>(vertexOffsetData) & 0xf) == 0);
        __vector4 zero        = __vspltisb(0);
        __vector4 vert_offset = __lvx(vertexOffsetData, 0);
        __vector4 vert_l      = __lvlx(&vertData[vertid].x, 0);
        __vector4 vert_r      = __lvrx(&vertData[vertid].x, 16);
        __vector4 vert_i      = __vor(vert_l, vert_r);
        vert_i      = __vmrghh(zero, vert_i);
        vert_i      = __vaddsws(vert_i, vert_offset);
        __vector4 floatvert   = __vcsxwfp(vert_i, 0);
        __vector4 vertGran    = __lvlx(&vertexGranularity, 0);
        vertGran    = __vspltw(vertGran, 0);
    return rwpmath::Vector3(__vmulfp(floatvert, vertGran));

#else
        int32_t offsetX = vertexOffsetData[0], offsetY = vertexOffsetData[1], offsetZ = vertexOffsetData[2];
    return rwpmath::Vector3( (offsetX + vertData[vertid].x) * vertexGranularity,
            (offsetY + vertData[vertid].y) * vertexGranularity,
        (offsetZ + vertData[vertid].z) * vertexGranularity );
#endif
    }

// Specialization for 32-bit compression
template <>
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ClusteredMeshCluster::GetVertexBase<ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>(uint8_t vertid, const float &vertexGranularity) const
    {
    EA_ASSERT(vertid < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_32BIT_COMPRESSED);
    CompressedVertexDataUnion vdUnion;
    vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
    const Vertex32 *vertData = vdUnion.m_asVertex32Ptr;

#if defined(EA_PLATFORM_PS3_PPU)

            vector float zero      = (vector float)vec_splat_s32(0);
            vector signed int vert_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vertData[vertid].x);
            vector signed int vert_r      = (vector signed int)vec_lvrx(16, &vertData[vertid].x);
            vector signed int vert_i      = vec_or(vert_l, vert_r);
            vector float floatvert = vec_ctf(vert_i, 0);
            vector float vertGran  = (vector float)clusteredmeshcluster_vec_lvlx(0, &vertexGranularity);
            vertGran  = vec_splat(vertGran, 0);
    return rwpmath::Vector3(vec_madd(floatvert, vertGran, zero));

#elif defined(EA_PLATFORM_XENON)

            __vector4 vert_l      = __lvlx(&vertData[vertid].x, 0);
            __vector4 vert_r      = __lvrx(&vertData[vertid].x, 16);
            __vector4 vert_i      = __vor(vert_l, vert_r);
            __vector4 floatvert   = __vcsxwfp(vert_i, 0);
            __vector4 vertGran    = __lvlx(&vertexGranularity, 0);
            vertGran    = __vspltw(vertGran, 0);
    return rwpmath::Vector3(__vmulfp(floatvert, vertGran));

#else
    return rwpmath::Vector3( vertData[vertid].x * vertexGranularity,
                vertData[vertid].y * vertexGranularity,
        vertData[vertid].z * vertexGranularity );
#endif
        }

/**
\brief Decompresses and returns single vertex based on the vertex ID.

\param  vertid                The id of the vertex to decompress
\param  vertexGranularity    The vertex compression granularity

\return The vertex
*/
template <>
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ClusteredMeshCluster::GetVertexBase<ClusteredMeshCluster::COMPRESSION_DYNAMIC>(uint8_t vertid, const float &vertexGranularity) const
{
    switch (compressionMode)
    {
    default:
    case VERTICES_UNCOMPRESSED:
        return GetVertexBase<VERTICES_UNCOMPRESSED>(vertid, vertexGranularity);
    case VERTICES_16BIT_COMPRESSED:
        return GetVertexBase<VERTICES_16BIT_COMPRESSED>(vertid, vertexGranularity);
    case VERTICES_32BIT_COMPRESSED:
        return GetVertexBase<VERTICES_32BIT_COMPRESSED>(vertid, vertexGranularity);
    }
}

RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ClusteredMeshCluster::GetVertex(uint8_t vertid, const float &vertexGranularity) const
{
    return GetVertexBase<COMPRESSION_DYNAMIC>(vertid, vertexGranularity);
}

template <uint8_t COMPRESSION> RW_COLLISION_FORCE_INLINE void
ClusteredMeshCluster::Get3VerticesBase(rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, 
                                       uint8_t v0, uint8_t v1, uint8_t v2, const float &g) const
{
    out0 = GetVertexBase<COMPRESSION>(v0, g);
    out1 = GetVertexBase<COMPRESSION>(v1, g);
    out2 = GetVertexBase<COMPRESSION>(v2, g);
}

template <> RW_COLLISION_FORCE_INLINE void
ClusteredMeshCluster::Get3VerticesBase<ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, 
    uint8_t v0, uint8_t v1, uint8_t v2, const float &g) const
    {
    EA_ASSERT(v0 < vertexCount);
    EA_ASSERT(v1 < vertexCount);
    EA_ASSERT(v2 < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_16BIT_COMPRESSED);
    CompressedVertexDataUnion vdUnion;
    vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
    const int32_t *offset = vdUnion.m_asInt32Ptr;
    const Vertex16 *vert = vdUnion.m_asVertex16Ptr + 2;

#if defined(EA_PLATFORM_PS3_PPU)

        // first vector in cluster - is quadword aligned
        EA_ASSERT((reinterpret_cast<int>(offset) & 0xf) == 0);
        vector float zero        = (vector float)vec_splat_s32(0);
        vector signed int   vert_offset = vec_ld(0, offset);
        vector float vertGran    = (vector float)clusteredmeshcluster_vec_lvlx(0, &g);
        vertGran    = vec_splat(vertGran, 0);

        // operations for all 3 vertices interleaved

        // left unaligned load for all vertices
        vector signed int   vert_0_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v0].x);
        vector signed int   vert_1_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v1].x);
        vector signed int   vert_2_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v2].x);

        // right unaligned load for all vertices
        vector signed int   vert_0_r      = (vector signed int)vec_lvrx(16, &vert[v0].x);
        vector signed int   vert_1_r      = (vector signed int)vec_lvrx(16, &vert[v1].x);
        vector signed int   vert_2_r      = (vector signed int)vec_lvrx(16, &vert[v2].x);

        // combine unaligned loads for all vertices
        vector signed int   vert_0_i      = vec_or(vert_0_l, vert_0_r);
        vector signed int   vert_1_i      = vec_or(vert_1_l, vert_1_r);
        vector signed int   vert_2_i      = vec_or(vert_2_l, vert_2_r);

        // expand unsigned shorts to words for all vertices
        vert_0_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_0_i);
        vert_1_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_1_i);
        vert_2_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_2_i);

        // add cluster offset to all vertices
        vert_0_i      = vec_add(vert_0_i, vert_offset);
        vert_1_i      = vec_add(vert_1_i, vert_offset);
        vert_2_i      = vec_add(vert_2_i, vert_offset);

        // convert all vertices to floating point
        vector float floatvert_0   = vec_ctf(vert_0_i, 0);
        vector float floatvert_1   = vec_ctf(vert_1_i, 0);
        vector float floatvert_2   = vec_ctf(vert_2_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(vec_madd(floatvert_0, vertGran, zero));
    out1 = rwpmath::Vector3(vec_madd(floatvert_1, vertGran, zero));
    out2 = rwpmath::Vector3(vec_madd(floatvert_2, vertGran, zero));

#elif defined(EA_PLATFORM_XENON)

        // first vector in cluster - is quadword aligned
        EA_ASSERT((reinterpret_cast<int>(offset) & 0xf) == 0);
        __vector4 vert_offset = __lvx(offset, 0);
        __vector4 vertGran    = __lvlx(&g, 0);
        vertGran    = __vspltw(vertGran, 0);
        __vector4 zero        = __vspltisb(0);

        // operations for all 3 vertices interleaved

        // left unaligned load for all vertices
        __vector4 vert_0_l      = __lvlx(&vert[v0].x, 0);
        __vector4 vert_1_l      = __lvlx(&vert[v1].x, 0);
        __vector4 vert_2_l      = __lvlx(&vert[v2].x, 0);

        // right unaligned load for all vertices
        __vector4 vert_0_r      = __lvrx(&vert[v0].x, 16);
        __vector4 vert_1_r      = __lvrx(&vert[v1].x, 16);
        __vector4 vert_2_r      = __lvrx(&vert[v2].x, 16);

        // combine unaligned loads for all vertices
        __vector4 vert_0_i      = __vor(vert_0_l, vert_0_r);
        __vector4 vert_1_i      = __vor(vert_1_l, vert_1_r);
        __vector4 vert_2_i      = __vor(vert_2_l, vert_2_r);

        // expand unsigned shorts to words for all vertices
        vert_0_i      = __vmrghh(zero, vert_0_i);
        vert_1_i      = __vmrghh(zero, vert_1_i);
        vert_2_i      = __vmrghh(zero, vert_2_i);

        // add cluster offset to all vertices
        vert_0_i      = __vaddsws(vert_0_i, vert_offset);
        vert_1_i      = __vaddsws(vert_1_i, vert_offset);
        vert_2_i      = __vaddsws(vert_2_i, vert_offset);

        // convert all vertices to floating point
        __vector4 floatvert_0   = __vcsxwfp(vert_0_i, 0);
        __vector4 floatvert_1   = __vcsxwfp(vert_1_i, 0);
        __vector4 floatvert_2   = __vcsxwfp(vert_2_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(__vmulfp(floatvert_0, vertGran));
    out1 = rwpmath::Vector3(__vmulfp(floatvert_1, vertGran));
    out2 = rwpmath::Vector3(__vmulfp(floatvert_2, vertGran));

#else

        int32_t x = offset[0];
        int32_t y = offset[1];
        int32_t z = offset[2];

    out0.Set((x + vert[v0].x) * g, (y + vert[v0].y) * g, (z + vert[v0].z) * g);
    out1.Set((x + vert[v1].x) * g, (y + vert[v1].y) * g, (z + vert[v1].z) * g);
    out2.Set((x + vert[v2].x) * g, (y + vert[v2].y) * g, (z + vert[v2].z) * g);

#endif
    }

template <> RW_COLLISION_FORCE_INLINE void
ClusteredMeshCluster::Get3VerticesBase<ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, 
    uint8_t v0, uint8_t v1, uint8_t v2, const float &g) const
    {
    EA_ASSERT(v0 < vertexCount);
    EA_ASSERT(v1 < vertexCount);
    EA_ASSERT(v2 < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_32BIT_COMPRESSED);
    CompressedVertexDataUnion vdUnion;
    vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
    const Vertex32 *vert = vdUnion.m_asVertex32Ptr;

#if defined(EA_PLATFORM_PS3_PPU)

        vector float zero      = (vector float)vec_splat_s32(0);
        vector float vertGran  = (vector float)clusteredmeshcluster_vec_lvlx(0, &g);
        vertGran  = vec_splat(vertGran, 0);

        // operations for all 3 vertices interleaved

        // left unaligned load for all vertices
        vector signed int vert_0_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v0].x);
        vector signed int vert_1_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v1].x);
        vector signed int vert_2_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v2].x);

        // right unaligned load for all vertices
        vector signed int vert_0_r      = (vector signed int)vec_lvrx(16, &vert[v0].x);
        vector signed int vert_1_r      = (vector signed int)vec_lvrx(16, &vert[v1].x);
        vector signed int vert_2_r      = (vector signed int)vec_lvrx(16, &vert[v2].x);

        // combine unaligned loads for all vertices
        vector signed int vert_0_i      = vec_or(vert_0_l, vert_0_r);
        vector signed int vert_1_i      = vec_or(vert_1_l, vert_1_r);
        vector signed int vert_2_i      = vec_or(vert_2_l, vert_2_r);

        // convert all vertices to floating point
        vector float floatvert_0 = vec_ctf(vert_0_i, 0);
        vector float floatvert_1 = vec_ctf(vert_1_i, 0);
        vector float floatvert_2 = vec_ctf(vert_2_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(vec_madd(floatvert_0, vertGran, zero));
    out1 = rwpmath::Vector3(vec_madd(floatvert_1, vertGran, zero));
    out2 = rwpmath::Vector3(vec_madd(floatvert_2, vertGran, zero));

#elif defined(EA_PLATFORM_XENON)

        __vector4 vertGran    = __lvlx(&g, 0);
        vertGran    = __vspltw(vertGran, 0);

        // operations for all 3 vertices interleaved

        // left unaligned load for all vertices
        __vector4 vert_0_l      = __lvlx(&vert[v0].x, 0);
        __vector4 vert_1_l      = __lvlx(&vert[v1].x, 0);
        __vector4 vert_2_l      = __lvlx(&vert[v2].x, 0);

        // right unaligned load for all vertices
        __vector4 vert_0_r      = __lvrx(&vert[v0].x, 16);
        __vector4 vert_1_r      = __lvrx(&vert[v1].x, 16);
        __vector4 vert_2_r      = __lvrx(&vert[v2].x, 16);

        // combine unaligned loads for all vertices
        __vector4 vert_0_i      = __vor(vert_0_l, vert_0_r);
        __vector4 vert_1_i      = __vor(vert_1_l, vert_1_r);
        __vector4 vert_2_i      = __vor(vert_2_l, vert_2_r);

        // convert all vertices to floating point
        __vector4 floatvert_0   = __vcsxwfp(vert_0_i, 0);
        __vector4 floatvert_1   = __vcsxwfp(vert_1_i, 0);
        __vector4 floatvert_2   = __vcsxwfp(vert_2_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(__vmulfp(floatvert_0, vertGran));
    out1 = rwpmath::Vector3(__vmulfp(floatvert_1, vertGran));
    out2 = rwpmath::Vector3(__vmulfp(floatvert_2, vertGran));

#else

    out0.Set(vert[v0].x * g, vert[v0].y * g, vert[v0].z * g);
    out1.Set(vert[v1].x * g, vert[v1].y * g, vert[v1].z * g);
    out2.Set(vert[v2].x * g, vert[v2].y * g, vert[v2].z * g);

#endif
    }

/**
\brief Decompresses and returns three vertices.

\param  out                    A pointer to an array of three vertices which gets filled in by this method
\param  v0                    The ID of the first vertex
\param  v1                    The ID of the second vertex
\param  v2                    The ID of the third vertex
\param  vertexGranularity    The vertex compression granularity
*/
template <> RW_COLLISION_FORCE_INLINE void
ClusteredMeshCluster::Get3VerticesBase<ClusteredMeshCluster::COMPRESSION_DYNAMIC>(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, 
    uint8_t v0, uint8_t v1, uint8_t v2, const float &g) const
{
    switch (compressionMode)
    {
    default:
    case VERTICES_UNCOMPRESSED:
        Get3VerticesBase<VERTICES_UNCOMPRESSED>(out0, out1, out2, v0, v1, v2, g);
        break;
    case VERTICES_32BIT_COMPRESSED:
        Get3VerticesBase<VERTICES_32BIT_COMPRESSED>(out0, out1, out2, v0, v1, v2, g);
        break;
    case VERTICES_16BIT_COMPRESSED:
        Get3VerticesBase<VERTICES_16BIT_COMPRESSED>(out0, out1, out2, v0, v1, v2, g);
        break;
    }
}

RW_COLLISION_FORCE_INLINE void
ClusteredMeshCluster::Get3Vertices(rwpmath::Vector3 *out, uint8_t v0, uint8_t v1, uint8_t v2, const float &g) const
{
    // For backwards compatibility - forward to dynamic non-array output version
    Get3VerticesBase<COMPRESSION_DYNAMIC>(out[0], out[1], out[2], v0, v1, v2, g);
}

template <uint8_t COMPRESSION> RW_COLLISION_FORCE_INLINE
void ClusteredMeshCluster::Get4VerticesBase(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, rwpmath::Vector3 & out3, 
    uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &g) const
{
    out0 = GetVertexBase<COMPRESSION>(v0, g);
    out1 = GetVertexBase<COMPRESSION>(v1, g);
    out2 = GetVertexBase<COMPRESSION>(v2, g);
    out3 = GetVertexBase<COMPRESSION>(v3, g);
}

template <> RW_COLLISION_FORCE_INLINE
void ClusteredMeshCluster::Get4VerticesBase<ClusteredMeshCluster::VERTICES_16BIT_COMPRESSED>(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, rwpmath::Vector3 & out3, 
    uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &g) const
    {
    EA_ASSERT(v0 < vertexCount);
    EA_ASSERT(v1 < vertexCount);
    EA_ASSERT(v2 < vertexCount);
    EA_ASSERT(v3 < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_16BIT_COMPRESSED);
    CompressedVertexDataUnion vdUnion;
    vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
    const int32_t *offset = vdUnion.m_asInt32Ptr;
    const Vertex16 *vert = vdUnion.m_asVertex16Ptr + 2;

#if defined(EA_PLATFORM_PS3_PPU)

        // first vector in cluster - is quadword aligned
        EA_ASSERT((reinterpret_cast<int>(offset) & 0xf) == 0);
        vector float zero        = (vector float)vec_splat_s32(0);
        vector signed int   vert_offset = vec_ld(0, offset);
        vector float vertGran    = (vector float)clusteredmeshcluster_vec_lvlx(0, &g);
        vertGran    = vec_splat(vertGran, 0);

        // operations for all vertices interleaved

        // left unaligned load for all vertices
        vector signed int   vert_0_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v0].x);
        vector signed int   vert_1_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v1].x);
        vector signed int   vert_2_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v2].x);
        vector signed int   vert_3_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v3].x);

        // right unaligned load for all vertices
        vector signed int   vert_0_r      = (vector signed int)vec_lvrx(16, &vert[v0].x);
        vector signed int   vert_1_r      = (vector signed int)vec_lvrx(16, &vert[v1].x);
        vector signed int   vert_2_r      = (vector signed int)vec_lvrx(16, &vert[v2].x);
        vector signed int   vert_3_r      = (vector signed int)vec_lvrx(16, &vert[v3].x);

        // combine unaligned loads for all vertices
        vector signed int   vert_0_i      = vec_or(vert_0_l, vert_0_r);
        vector signed int   vert_1_i      = vec_or(vert_1_l, vert_1_r);
        vector signed int   vert_2_i      = vec_or(vert_2_l, vert_2_r);
        vector signed int   vert_3_i      = vec_or(vert_3_l, vert_3_r);

        // expand unsigned shorts to words for all vertices
        vert_0_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_0_i);
        vert_1_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_1_i);
        vert_2_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_2_i);
        vert_3_i      = (vector signed int)vec_mergeh((vector signed short)zero, (vector signed short)vert_3_i);

        // add cluster offset to all vertices
        vert_0_i      = vec_add(vert_0_i, vert_offset);
        vert_1_i      = vec_add(vert_1_i, vert_offset);
        vert_2_i      = vec_add(vert_2_i, vert_offset);
        vert_3_i      = vec_add(vert_3_i, vert_offset);

        // convert all vertices to floating point
        vector float floatvert_0   = vec_ctf(vert_0_i, 0);
        vector float floatvert_1   = vec_ctf(vert_1_i, 0);
        vector float floatvert_2   = vec_ctf(vert_2_i, 0);
        vector float floatvert_3   = vec_ctf(vert_3_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(vec_madd(floatvert_0, vertGran, zero));
    out1 = rwpmath::Vector3(vec_madd(floatvert_1, vertGran, zero));
    out2 = rwpmath::Vector3(vec_madd(floatvert_2, vertGran, zero));
    out3 = rwpmath::Vector3(vec_madd(floatvert_3, vertGran, zero));

#elif defined(EA_PLATFORM_XENON)

        // first vector in cluster - is quadword aligned
        EA_ASSERT((reinterpret_cast<int>(offset) & 0xf) == 0);
        __vector4 vert_offset = __lvx(offset, 0);
        __vector4 vertGran    = __lvlx(&g, 0);
        vertGran    = __vspltw(vertGran, 0);
        __vector4 zero        = __vspltisb(0);

        // operations for all vertices interleaved

        // left unaligned load for all vertices
        __vector4 vert_0_l      = __lvlx(&vert[v0].x, 0);
        __vector4 vert_1_l      = __lvlx(&vert[v1].x, 0);
        __vector4 vert_2_l      = __lvlx(&vert[v2].x, 0);
        __vector4 vert_3_l      = __lvlx(&vert[v3].x, 0);

        // right unaligned load for all vertices
        __vector4 vert_0_r      = __lvrx(&vert[v0].x, 16);
        __vector4 vert_1_r      = __lvrx(&vert[v1].x, 16);
        __vector4 vert_2_r      = __lvrx(&vert[v2].x, 16);
        __vector4 vert_3_r      = __lvrx(&vert[v3].x, 16);

        // combine unaligned loads for all vertices
        __vector4 vert_0_i      = __vor(vert_0_l, vert_0_r);
        __vector4 vert_1_i      = __vor(vert_1_l, vert_1_r);
        __vector4 vert_2_i      = __vor(vert_2_l, vert_2_r);
        __vector4 vert_3_i      = __vor(vert_3_l, vert_3_r);

        // expand unsigned shorts to words for all vertices
        vert_0_i      = __vmrghh(zero, vert_0_i);
        vert_1_i      = __vmrghh(zero, vert_1_i);
        vert_2_i      = __vmrghh(zero, vert_2_i);
        vert_3_i      = __vmrghh(zero, vert_3_i);

        // add cluster offset to all vertices
        vert_0_i      = __vaddsws(vert_0_i, vert_offset);
        vert_1_i      = __vaddsws(vert_1_i, vert_offset);
        vert_2_i      = __vaddsws(vert_2_i, vert_offset);
        vert_3_i      = __vaddsws(vert_3_i, vert_offset);

        // convert all vertices to floating point
        __vector4 floatvert_0   = __vcsxwfp(vert_0_i, 0);
        __vector4 floatvert_1   = __vcsxwfp(vert_1_i, 0);
        __vector4 floatvert_2   = __vcsxwfp(vert_2_i, 0);
        __vector4 floatvert_3   = __vcsxwfp(vert_3_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(__vmulfp(floatvert_0, vertGran));
    out1 = rwpmath::Vector3(__vmulfp(floatvert_1, vertGran));
    out2 = rwpmath::Vector3(__vmulfp(floatvert_2, vertGran));
    out3 = rwpmath::Vector3(__vmulfp(floatvert_3, vertGran));

#else

        int32_t x = offset[0];
        int32_t y = offset[1];
        int32_t z = offset[2];

    out0.Set((x + vert[v0].x) * g, (y + vert[v0].y) * g, (z + vert[v0].z) * g);
    out1.Set((x + vert[v1].x) * g, (y + vert[v1].y) * g, (z + vert[v1].z) * g);
    out2.Set((x + vert[v2].x) * g, (y + vert[v2].y) * g, (z + vert[v2].z) * g);
    out3.Set((x + vert[v3].x) * g, (y + vert[v3].y) * g, (z + vert[v3].z) * g);

#endif
    }

template <> RW_COLLISION_FORCE_INLINE
void ClusteredMeshCluster::Get4VerticesBase<ClusteredMeshCluster::VERTICES_32BIT_COMPRESSED>(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, rwpmath::Vector3 & out3, 
    uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &g) const
    {
    EA_ASSERT(v0 < vertexCount);
    EA_ASSERT(v1 < vertexCount);
    EA_ASSERT(v2 < vertexCount);
    EA_ASSERT(v3 < vertexCount);
    EA_ASSERT(compressionMode == VERTICES_32BIT_COMPRESSED);
    CompressedVertexDataUnion vdUnion;
    vdUnion.m_as_rwpmathVector3Ptr = vertexArray;
    const Vertex32 *vert = vdUnion.m_asVertex32Ptr;

#if defined(EA_PLATFORM_PS3_PPU)

        vector float zero      = (vector float)vec_splat_s32(0);
        vector float vertGran  = (vector float)clusteredmeshcluster_vec_lvlx(0, &g);
        vertGran  = vec_splat(vertGran, 0);

        // operations for all vertices interleaved

        // left unaligned load for all vertices
        vector signed int vert_0_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v0].x);
        vector signed int vert_1_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v1].x);
        vector signed int vert_2_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v2].x);
        vector signed int vert_3_l      = (vector signed int)clusteredmeshcluster_vec_lvlx(0,  &vert[v3].x);

        // right unaligned load for all vertices
        vector signed int vert_0_r      = (vector signed int)vec_lvrx(16, &vert[v0].x);
        vector signed int vert_1_r      = (vector signed int)vec_lvrx(16, &vert[v1].x);
        vector signed int vert_2_r      = (vector signed int)vec_lvrx(16, &vert[v2].x);
        vector signed int vert_3_r      = (vector signed int)vec_lvrx(16, &vert[v3].x);

        // combine unaligned loads for all vertices
        vector signed int vert_0_i      = vec_or(vert_0_l, vert_0_r);
        vector signed int vert_1_i      = vec_or(vert_1_l, vert_1_r);
        vector signed int vert_2_i      = vec_or(vert_2_l, vert_2_r);
        vector signed int vert_3_i      = vec_or(vert_3_l, vert_3_r);

        // convert all vertices to floating point
        vector float floatvert_0 = vec_ctf(vert_0_i, 0);
        vector float floatvert_1 = vec_ctf(vert_1_i, 0);
        vector float floatvert_2 = vec_ctf(vert_2_i, 0);
        vector float floatvert_3 = vec_ctf(vert_3_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(vec_madd(floatvert_0, vertGran, zero));
    out1 = rwpmath::Vector3(vec_madd(floatvert_1, vertGran, zero));
    out2 = rwpmath::Vector3(vec_madd(floatvert_2, vertGran, zero));
    out3 = rwpmath::Vector3(vec_madd(floatvert_3, vertGran, zero));

#elif defined(EA_PLATFORM_XENON)

        __vector4 vertGran    = __lvlx(&g, 0);
        vertGran    = __vspltw(vertGran, 0);

        // operations for all vertices interleaved

        // left unaligned load for all vertices
        __vector4 vert_0_l      = __lvlx(&vert[v0].x, 0);
        __vector4 vert_1_l      = __lvlx(&vert[v1].x, 0);
        __vector4 vert_2_l      = __lvlx(&vert[v2].x, 0);
        __vector4 vert_3_l      = __lvlx(&vert[v3].x, 0);

        // right unaligned load for all vertices
        __vector4 vert_0_r      = __lvrx(&vert[v0].x, 16);
        __vector4 vert_1_r      = __lvrx(&vert[v1].x, 16);
        __vector4 vert_2_r      = __lvrx(&vert[v2].x, 16);
        __vector4 vert_3_r      = __lvrx(&vert[v3].x, 16);

        // combine unaligned loads for all vertices
        __vector4 vert_0_i      = __vor(vert_0_l, vert_0_r);
        __vector4 vert_1_i      = __vor(vert_1_l, vert_1_r);
        __vector4 vert_2_i      = __vor(vert_2_l, vert_2_r);
        __vector4 vert_3_i      = __vor(vert_3_l, vert_3_r);

        // convert all vertices to floating point
        __vector4 floatvert_0   = __vcsxwfp(vert_0_i, 0);
        __vector4 floatvert_1   = __vcsxwfp(vert_1_i, 0);
        __vector4 floatvert_2   = __vcsxwfp(vert_2_i, 0);
        __vector4 floatvert_3   = __vcsxwfp(vert_3_i, 0);

        // multiply all vertices by granularity and store results
    out0 = rwpmath::Vector3(__vmulfp(floatvert_0, vertGran));
    out1 = rwpmath::Vector3(__vmulfp(floatvert_1, vertGran));
    out2 = rwpmath::Vector3(__vmulfp(floatvert_2, vertGran));
    out3 = rwpmath::Vector3(__vmulfp(floatvert_3, vertGran));

#else

    out0.Set(vert[v0].x * g, vert[v0].y * g, vert[v0].z * g);
    out1.Set(vert[v1].x * g, vert[v1].y * g, vert[v1].z * g);
    out2.Set(vert[v2].x * g, vert[v2].y * g, vert[v2].z * g);
    out3.Set(vert[v3].x * g, vert[v3].y * g, vert[v3].z * g);

#endif
    }

/**
\brief Decompresses and returns four vertices.

\param  out                    A pointer to an array of four vertices which gets filled in by this method
\param  v0                    The ID of the first vertex
\param  v1                    The ID of the second vertex
\param  v2                    The ID of the third vertex
\param  v3                    The ID of the fourth vertex
\param  vertexGranularity    The vertex compression granularity
*/
template <> RW_COLLISION_FORCE_INLINE
void ClusteredMeshCluster::Get4VerticesBase<ClusteredMeshCluster::COMPRESSION_DYNAMIC>(
    rwpmath::Vector3 & out0, rwpmath::Vector3 & out1, rwpmath::Vector3 & out2, rwpmath::Vector3 & out3, 
    uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &g) const
{
    switch (compressionMode)
    {
    default:
    case VERTICES_UNCOMPRESSED:
        Get4VerticesBase<VERTICES_UNCOMPRESSED>(out0, out1, out2, out3, v0, v1, v2, v3, g);
        break;
    case VERTICES_16BIT_COMPRESSED:
        Get4VerticesBase<VERTICES_16BIT_COMPRESSED>(out0, out1, out2, out3, v0, v1, v2, v3, g);
        break;
    case VERTICES_32BIT_COMPRESSED:
        Get4VerticesBase<VERTICES_32BIT_COMPRESSED>(out0, out1, out2, out3, v0, v1, v2, v3, g);
        break;
    }

    }

RW_COLLISION_FORCE_INLINE void
ClusteredMeshCluster::Get4Vertices(rwpmath::Vector3 *out, uint8_t v0, uint8_t v1, uint8_t v2, uint8_t v3, const float &g) const
{
    // For backwards compatibility - forward to dynamic non-array output version
    Get4VerticesBase<COMPRESSION_DYNAMIC>(out[0], out[1], out[2], out[3], v0, v1, v2, v3, g);
}



/**
\brief Gets a pointer to the unit data.

\return The unit data
*/
RW_COLLISION_FORCE_INLINE uint8_t *
ClusteredMeshCluster::UnitData()
{
    return reinterpret_cast<uint8_t*>(vertexArray) + unitDataStart * 16;
}

/**
\brief Gets a const pointer to the unit data.

\return The unit data
*/
RW_COLLISION_FORCE_INLINE const uint8_t *
ClusteredMeshCluster::UnitData() const
{
    return reinterpret_cast<const uint8_t*>(vertexArray) + unitDataStart * 16;
}

/**
\internal
Gets the unit type.
*/
RW_COLLISION_FORCE_INLINE uint32_t
ClusteredMeshCluster::GetUnitType(uint32_t offset)
{
    return uint32_t(UnitData()[offset] & UNITTYPE_MASK);

}


/**
\internal
Get the size of the whole unit in bytes.
\param offset byte offset of unit within cluster
\return size of the unit in bytes
*/
RW_COLLISION_FORCE_INLINE uint32_t
ClusteredMeshCluster::GetUnitSize(uint32_t offset, const ClusterParams & clusterParams)
{
    uint8_t *data = &UnitData()[offset];
    uint32_t i = 1;
    uint32_t triangleCount = 1;

    switch (data[0] & UNITTYPE_MASK)
    {
    case UNITTYPE_QUAD:
        triangleCount = 2;
        break;
    case UNITTYPE_TRILIST:
        triangleCount = data[i++];
        break;
    }
    i += triangleCount + 2;            // skip over vertex data
    if (data[0] & UNITFLAG_EDGEANGLE)
    {
        i += triangleCount + 2;
    }
    if (data[0] & UNITFLAG_GROUPID)
    {
        i += clusterParams.mGroupIdSize;
    }
    if (data[0] & UNITFLAG_SURFACEID)
    {
        i += clusterParams.mSurfaceIdSize;
    }
    return i;
}


/**
\brief Gets a pointer to the normal data.

\return The normal data
*/
RW_COLLISION_FORCE_INLINE rwpmath::Vector3 *
ClusteredMeshCluster::NormalArray()
{
    return reinterpret_cast<rwpmath::Vector3*>(reinterpret_cast<uint8_t*>(vertexArray) + 16 * normalStart);
}

/**
\brief Gets the surface ID and group ID of a cluster

\return The normal data
*/
inline void
ClusteredMeshCluster::GetGroupAndSurfaceId(uint32_t offset, const ClusterParams & clusterParams, uint32_t & groupID, uint32_t & surfaceID)
{
    // TODO: PAB: compare code generation for branch-free approach

    uint8_t *dataStream = &UnitData()[offset];
    uint32_t unitType = GetUnitType(offset);

    // compute unit type masks
    uint32_t triangleMask = ~(uint32_t)((-(int32_t)((uint8_t)UNITTYPE_TRIANGLE ^ (uint8_t)unitType)) >> 31);
    uint32_t quadMask     = ~(uint32_t)((-(int32_t)((uint8_t)UNITTYPE_QUAD     ^ (uint8_t)unitType)) >> 31);
    uint32_t triListMask  = ~(uint32_t)((-(int32_t)((uint8_t)UNITTYPE_TRILIST  ^ (uint8_t)unitType)) >> 31);

    // branch free triangle count computation
    uint32_t triCount = (1 & triangleMask) + (2 & quadMask) + (*(dataStream + 1) & triListMask);

    // The vertex indices follow the type and optional count
    uint8_t *vIndex = (dataStream + 1) + (triListMask & 1);

    // The optional edge angles follow the vertex indices
    uint8_t *edge = vIndex + triCount + 2;

    // Miscellaneous data follows the optional edge angles
    uint32_t edgeAngleMask  = (uint32_t)((-(int32_t)((uint8_t)UNITFLAG_EDGEANGLE  & dataStream[0])) >> 31);
    uint8_t *misc = edge + ( edgeAngleMask & (triCount + 2) );

    uint32_t groupIdTmp = 0;
    uint32_t surfaceIdTmp = 0;

    if (dataStream[0] & UNITFLAG_GROUPID)
    {
        groupIdTmp = *(misc++);
        groupIdTmp += (clusterParams.mGroupIdSize==2) ? *(misc++) * 256 : 0;
    }
    if (dataStream[0] & UNITFLAG_SURFACEID)
    {
        surfaceIdTmp = *(misc++);
        surfaceIdTmp += (clusterParams.mSurfaceIdSize==2) ? *(misc++) * 256 : 0;
    }

    groupID = groupIdTmp;
    surfaceID = surfaceIdTmp;
}


/**
\brief Gets the number of volumes in a particular unit

Report the number of volumes in this unit (1 for triangles,
2 for quads that encode 2 adjacent triangles,
or possibly a larger number in the case of trilist/strip units).

\param offset The offset of the unit

\return The unit data
*/
inline uint32_t
ClusteredMeshCluster::NumVolumesInUnit(uint32_t offset)
{
    uint8_t *data = &UnitData()[offset];

    EA_ASSERT((data[0] & UNITTYPE_MASK) <= UNITTYPE_TRILIST);

    switch (data[0] & UNITTYPE_MASK)
    {
    case UNITTYPE_QUAD:    return 2;
    case UNITTYPE_TRILIST: return data[1];
    }
    return 1;
}


/**
\brief Decode the edgecos, convert a number 0..31 into the floating point cosine.

See explanation in design doc http://globaltechdocs.ea.com/tdswiki/index.php?title=EdgeCosine
\param B the encoded edgecos value, a number from 0 to 31.
\return result = 1 - PI^2 / 2^(B+3)
*/
inline float
DecodeEdgeCos(uint32_t B)
{
    return 1.0f - (GetFloatPISquared()) / static_cast<float>(8 << B);

    //  TODO:  probably more efficient to use, 1.0f - ldexp(PI*PI, -B - 3)
}


/**
\brief Setup the convexity flags and one-sided flags for a triangle volume.
\param ec0 flags for edge 0
\param ec1 flags for edge 1
\param ec2 flags for edge 2
\param meshFlags flags for the whole mesh
\return The flags
*/
inline uint32_t
ComputeTriangleFlags(uint8_t ec0, uint8_t ec1, uint8_t ec2, uint16_t meshFlags)
{
    //  Ensure that the mapping from EDGEFLAG and CMFLAG into VOLUMEFLAG is correct.
    EA_COMPILETIME_ASSERT((VolumeFlag)(EDGEFLAG_EDGECONVEX) == VOLUMEFLAG_TRIANGLEEDGE0CONVEX);
    EA_COMPILETIME_ASSERT((EDGEFLAG_VERTEXDISABLE << 3) == VOLUMEFLAG_TRIANGLEVERT0DISABLE);
    EA_COMPILETIME_ASSERT((VolumeFlag)(CMFLAG_ONESIDED) == VOLUMEFLAG_TRIANGLEONESIDED);

    const uint32_t mask = VOLUMEFLAG_TRIANGLEONESIDED +
        (VOLUMEFLAG_TRIANGLEEDGE0CONVEX + VOLUMEFLAG_TRIANGLEVERT0DISABLE) * 7;

    return (VOLUMEFLAG_TRIANGLEDEFAULT & ~mask) |
        (ec0 & EDGEFLAG_EDGECONVEX) |
        ((ec1 & EDGEFLAG_EDGECONVEX) << 1) |
        ((ec2 & EDGEFLAG_EDGECONVEX) << 2) |
        ((ec0 & EDGEFLAG_VERTEXDISABLE) << 3) |
        ((ec1 & EDGEFLAG_VERTEXDISABLE) << 4) |
        ((ec2 & EDGEFLAG_VERTEXDISABLE) << 5) |
        (meshFlags & CMFLAG_ONESIDED);
}


#ifdef EA_DEBUG
static uint32_t gComputeEdgeCosErrorCounter = 0;
#endif // EA_DEBUG

/**
\brief Given two adjacent triangles (0,1,2) and (3,2,1) compute the edgecos and convexity flag of the common edge
between v1 and v2.
\param convexFlag OUTPUT zero if edge is reflex, and EDGEFLAG_EDGECONVEX if not.
\param v0 vertex 0
\param v1 vertex 1
\param v2 vertex 2
\param v3 vertex 3
\return the edgecos of the common edge, from -1 to 1.
*/
inline float
ComputeEdgeCos(int8_t &convexFlag, rwpmath::Vector3::InParam v0, rwpmath::Vector3::InParam v1,
               rwpmath::Vector3::InParam v2, rwpmath::Vector3::InParam v3)
{
    rwpmath::Vector3 n1 = Cross(v1 - v0, v2 - v0);
    rwpmath::Vector3 n2 = Cross(v2 - v3, v1 - v3);
    convexFlag = static_cast<int8_t>(Dot(v2 - v1, Cross(n1, n2)) > rwpmath::GetVecFloat_Zero() ? EDGEFLAG_EDGECONVEX : 0);

    rwpmath::VecFloat len1 = MagnitudeSquared(n1);
    rwpmath::VecFloat len2 = MagnitudeSquared(n2);

    float ec = 1.0f;
    rwpmath::VecFloat minFloat = rwpmath::GetVecFloat_MinValue();
    if ((len1 > minFloat) && (len2 > minFloat))
    {
        ec = rwpmath::GetFloat(Dot(n1, n2) * rwpmath::InvSqrt(len1) * rwpmath::InvSqrt(len2));
    }
#ifdef EA_DEBUG
    else
    {
        if (++gComputeEdgeCosErrorCounter < 100)
        {
            EAPHYSICS_MESSAGE("Triangle with zero area found near (%g %g %g)", rwpmath::GetFloat(v0.X()), rwpmath::GetFloat(v0.Y()), rwpmath::GetFloat(v0.Z()));
        }
    }
#endif
    return ec;
}
}
}
#endif
