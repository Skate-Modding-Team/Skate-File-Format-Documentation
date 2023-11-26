// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTER_UNIT_BASE_H
#define PUBLIC_RW_COLLISION_CLUSTER_UNIT_BASE_H

/*************************************************************************************************************

File: clusterunitbase.h

Purpose: Base class for accessor classes for ClusteredMeshCluster data.

*/
#include "rw/collision/common.h"                          // for rwpmath::*
#include "rw/collision/clusteredmesh.h"                 // for ClusteredMeshCluster

namespace rw
{
namespace collision
{

/// Base class for all unit accessors providing common and/or useful functionality.
class ClusterUnitBase
{
    // DecodeEdgeCosine(DEFAULT_EDGE_DATA) < 0.0f, no flags set
    static const uint8_t DEFAULT_EDGE_DATA = 0u;

public: // Static methods - should probably be implemented in ClusteredMeshCluster?

    /// Compute a single edge cosine given a packed edge data byte with flags.
    /// If computing 3 or 4 edge cosines, the overloads of DecodeEdgeCosines() will be faster.
    static EA_FORCE_INLINE float DecodeEdgeCosineUnmasked(uint8_t b)
    {
        uint8_t p = (uint8_t) (b & ((uint8_t) EDGEFLAG_ANGLEMASK));
        return DecodeEdgeCosine(p);
    }

    /// Compute a single edge cosine given a packed edge data byte with flags masked out.
    /// If computing 3 or 4 edge cosines, the overloads of DecodeEdgeCosines() will be faster.
    static EA_FORCE_INLINE float DecodeEdgeCosine(uint8_t p)
    {
        EA_ASSERT((p & (~EDGEFLAG_ANGLEMASK)) == 0);
        // The "bit-twiddling" approach using non-vpu registers appears to be faster for a single edge cosine.
        // For multiple edge-cosines, use the vectorized methods below.
        const float piSquaredOver8 = 1.23370055f;
        return 1.0f - ldexp(piSquaredOver8, -p);
    }

    /// Compute 3 edge cosines at once using vector unit where available.
    /// The VectorIntrinsic overload of this function will be faster on platforms that support it.
    static EA_FORCE_INLINE rwpmath::Vector3 DecodeEdgeCosinesUnmasked(uint8_t ed0, uint8_t ed1, uint8_t ed2)
    {
#if defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU)
        // Use VPL where supported natively and mask in VPL too
        rwpmath::Mask3 edgedata((uint32_t) ed0, (uint32_t) ed1, (uint32_t) ed2);
        rwpmath::Vector3 edgeCosines;
        edgeCosines.mV = DecodeEdgeCosinesUnmasked(edgedata.mV);
        return edgeCosines;
#else
        // Vector math appears to be faster than 3 lots of fpu math on PC.
        const uint8_t p0 = (uint8_t) (ed0 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        const uint8_t p1 = (uint8_t) (ed1 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        const uint8_t p2 = (uint8_t) (ed2 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        return DecodeEdgeCosines(p0, p1, p2);
#endif
    }

    /// Compute 3 edge cosines at once using vector unit where available.
    /// The VectorIntrinsic overload of this function will be faster on platforms that support it.
    static EA_FORCE_INLINE rwpmath::Vector3 DecodeEdgeCosines(uint8_t p0, uint8_t p1, uint8_t p2)
    {
        EA_ASSERT((p0 & (~EDGEFLAG_ANGLEMASK)) == 0);
        EA_ASSERT((p1 & (~EDGEFLAG_ANGLEMASK)) == 0);
        EA_ASSERT((p2 & (~EDGEFLAG_ANGLEMASK)) == 0);
#if defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU)
        // Use VPL where supported natively
        rwpmath::Mask3 edgedata((uint32_t) p0, (uint32_t) p1, (uint32_t) p2);
        rwpmath::Vector3 edgeCosines;
        edgeCosines.mV = DecodeEdgeCosines(edgedata.mV);
        return edgeCosines;
#else
        // Vector math appears to be faster than 3 lots of fpu math on PC.
        rwpmath::Vector3 p(
            static_cast<float>(8 << p0), 
            static_cast<float>(8 << p1), 
            static_cast<float>(8 << p2));
        return rwpmath::GetVector3_One() - (GetVector3PISquared() / p);
#endif
    }

    /// Compute 4 edge cosines at once using vector unit where available.
    /// The VectorIntrinsic overload of this function will be faster on platforms that support it.
    static EA_FORCE_INLINE rwpmath::Vector4 DecodeEdgeCosinesUnmasked(uint8_t ed0, uint8_t ed1, uint8_t ed2, uint8_t ed3)
    {
#if defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU)
        // Use VPL where supported natively and mask in VPL too
        rwpmath::Mask4 edgedata((uint32_t) ed0, (uint32_t) ed1, (uint32_t) ed2, (uint32_t) ed3);
        rwpmath::Vector4 edgeCosines;
        edgeCosines.mV = DecodeEdgeCosinesUnmasked(edgedata.mV);
        return edgeCosines;
#else
        // Vector math appears to be faster than 3 lots of fpu math on PC.
        const uint8_t p0 = (uint8_t) (ed0 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        const uint8_t p1 = (uint8_t) (ed1 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        const uint8_t p2 = (uint8_t) (ed2 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        const uint8_t p3 = (uint8_t) (ed3 & ((uint8_t) EDGEFLAG_ANGLEMASK));
        return DecodeEdgeCosines(p0, p1, p2, p3);
#endif
    }

    /// Compute 4 edge cosines at once using vector unit where available.
    /// The VectorIntrinsic overload of this function will be faster on platforms that support it.
    static EA_FORCE_INLINE rwpmath::Vector4 DecodeEdgeCosines(uint8_t p0, uint8_t p1, uint8_t p2, uint8_t p3)
    {
        EA_ASSERT((p0 & (~EDGEFLAG_ANGLEMASK)) == 0);
        EA_ASSERT((p1 & (~EDGEFLAG_ANGLEMASK)) == 0);
        EA_ASSERT((p2 & (~EDGEFLAG_ANGLEMASK)) == 0);
        EA_ASSERT((p3 & (~EDGEFLAG_ANGLEMASK)) == 0);
#if defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU)
        // Use VPL where supported natively
        rwpmath::Mask4 edgedata((uint32_t) p0, (uint32_t) p1, (uint32_t) p2, (uint32_t) p3);
        rwpmath::Vector4 edgeCosines;
        edgeCosines.mV = DecodeEdgeCosines(edgedata.mV);
        return edgeCosines;
#else
        // Vector math appears to be faster than 4 lots of fpu math on PC.
        rwpmath::Vector4 p(
            static_cast<float>(8 << p0),
            static_cast<float>(8 << p1), 
            static_cast<float>(8 << p2), 
            static_cast<float>(8 << p3));
        return rwpmath::GetVector4_One() - (GetVector4PISquared() / p);
#endif
    }

    /// Compute the edge cosine and convex flag for the inner edge (v1 to v2) of a quad.
    /// Vertices are specified in order around edge of quad.
    /// If either trinagle is near degenerate, the returned edge cosine is 1.0f.
    /// @param convexFlag set to true if the central edge (v1 to v2) is convex.
    static EA_FORCE_INLINE rwpmath::VecFloat ComputeCentralEdgeCosine(
        rwpmath::MaskScalar &convexFlag, 
        rwpmath::Vector3::InParam v0, 
        rwpmath::Vector3::InParam v1, 
        rwpmath::Vector3::InParam v2, 
        rwpmath::Vector3::InParam v3)
    {
        const rwpmath::Vector3 n1 = rwpmath::Cross(v1 - v0, v2 - v0);
        const rwpmath::Vector3 n2 = rwpmath::Cross(v2 - v3, v1 - v3);
        convexFlag = rwpmath::CompGreaterThan(rwpmath::Dot(v2 - v1, rwpmath::Cross(n1, n2)), rwpmath::GetVecFloat_Zero());

        const rwpmath::VecFloat len1 = rwpmath::MagnitudeSquared(n1);
        const rwpmath::VecFloat len2 = rwpmath::MagnitudeSquared(n2);

        const rwpmath::VecFloat minLen(rwpmath::GetVecFloat_MinValue());
        const rwpmath::MaskScalar valid = rwpmath::And(rwpmath::CompGreaterThan(len1, minLen), rwpmath::CompGreaterThan(len2, minLen));
        const rwpmath::VecFloat ec(rwpmath::Select(valid, 
            rwpmath::Dot(n1, n2) * rwpmath::InvSqrt(len1) * rwpmath::InvSqrt(len2), 
            rwpmath::GetVecFloat_One()));
        return ec;
    }

    /// Forms GPInstance flags for triangle based on VOLUMEFLAG_TRIANGLEDEFAULT with
    /// CMFLAG_ONESIDED from meshFlags, EDGE_FLAGCONVEX from ec0,ec1,ec2 and EDGEFLAG_VERTEXDISABLE flag from vc0,vc1,vc2.
    static EA_FORCE_INLINE uint32_t ComputeGPTriangleFlags(uint8_t ec0, uint8_t ec1, uint8_t ec2, 
        uint8_t vc0, uint8_t vc1, uint8_t vc2, uint16_t meshFlags)
    {
        //  Ensure that the mapping from EDGEFLAG and CMFLAG into VOLUMEFLAG is correct.
        EA_COMPILETIME_ASSERT((VolumeFlag)(EDGEFLAG_EDGECONVEX) == VOLUMEFLAG_TRIANGLEEDGE0CONVEX);
        EA_COMPILETIME_ASSERT((EDGEFLAG_VERTEXDISABLE << 3) == VOLUMEFLAG_TRIANGLEVERT0DISABLE);
        EA_COMPILETIME_ASSERT((VolumeFlag)(CMFLAG_ONESIDED) == VOLUMEFLAG_TRIANGLEONESIDED);

        const uint32_t mask = VOLUMEFLAG_TRIANGLEONESIDED +
            (VOLUMEFLAG_TRIANGLEEDGE0CONVEX + VOLUMEFLAG_TRIANGLEVERT0DISABLE) * 7;

        return((VOLUMEFLAG_TRIANGLEDEFAULT & ~mask) |
            (ec0 & EDGEFLAG_EDGECONVEX) |
            ((ec1 & EDGEFLAG_EDGECONVEX) << 1) |
            ((ec2 & EDGEFLAG_EDGECONVEX) << 2) |
            ((vc0 & EDGEFLAG_VERTEXDISABLE) << 3) |
            ((vc1 & EDGEFLAG_VERTEXDISABLE) << 4) |
            ((vc2 & EDGEFLAG_VERTEXDISABLE) << 5) |
            (meshFlags & CMFLAG_ONESIDED));    
    }

    // Given three edge data values and global flags, return GPInstance flags for the triangle
    static EA_FORCE_INLINE uint32_t ComputeGPTriangleFlags(uint8_t ec0, uint8_t ec1, uint8_t ec2, uint16_t meshFlags)
    {
        // return rw::collision::ComputeTriangleFlags(ec0, ec1, ec2, meshFlags);
        return ComputeGPTriangleFlags(ec0, ec1, ec2, ec0, ec1, ec2, meshFlags);
    }

    // Given three edge data values and global flags, return rwpmath masks for the triangle features
    static EA_FORCE_INLINE void ComputeTriangleMasks(
        rwpmath::Mask3 & edgeIsConvex, rwpmath::Mask3 & disableVertices,
        rwpmath::MaskScalar & oneSided,
        uint8_t ec0, uint8_t ec1, uint8_t ec2, uint16_t meshFlags)
    {
        edgeIsConvex = rwpmath::Mask3(
            (ec0 & EDGEFLAG_EDGECONVEX) != 0,
            (ec1 & EDGEFLAG_EDGECONVEX) != 0,
            (ec2 & EDGEFLAG_EDGECONVEX) != 0);
        disableVertices = rwpmath::Mask3(
            (ec0 & EDGEFLAG_VERTEXDISABLE) != 0,
            (ec1 & EDGEFLAG_VERTEXDISABLE) != 0,
            (ec2 & EDGEFLAG_VERTEXDISABLE) != 0);
        oneSided = rwpmath::MaskScalar((meshFlags & CMFLAG_ONESIDED) != 0);
    }

    // Given GPTriangle flags, return rwpmath masks for the triangle features
    static EA_FORCE_INLINE void ComputeTriangleMasks(
        rwpmath::Mask3 & edgeIsConvex, rwpmath::Mask3 & disableVertices,
        rwpmath::MaskScalar & oneSided,
        uint32_t triangleFlags)
    {
        edgeIsConvex = rwpmath::Mask3(
            (triangleFlags & VOLUMEFLAG_TRIANGLEEDGE0CONVEX) != 0,
            (triangleFlags & VOLUMEFLAG_TRIANGLEEDGE1CONVEX) != 0,
            (triangleFlags & VOLUMEFLAG_TRIANGLEEDGE2CONVEX) != 0);
        disableVertices = rwpmath::Mask3(
            (triangleFlags & VOLUMEFLAG_TRIANGLEVERT0DISABLE) != 0,
            (triangleFlags & VOLUMEFLAG_TRIANGLEVERT1DISABLE) != 0,
            (triangleFlags & VOLUMEFLAG_TRIANGLEVERT2DISABLE) != 0);
        oneSided = rwpmath::MaskScalar((triangleFlags & VOLUMEFLAG_TRIANGLEONESIDED) != 0);
    }

    /// Extract edge cosines and GPTriangle flags for a triangle from edge data stored in unit.
    static EA_FORCE_INLINE uint32_t ExtractTriEdgeData(
        rwpmath::Vector3 & edgeCosines, 
        const uint8_t * edgeData, 
        uint16_t defaultFlags)
    {
        EA_ASSERT(edgeData);
        const uint8_t ed0 = edgeData[0];
        const uint8_t ed1 = edgeData[1];
        const uint8_t ed2 = edgeData[2];
        // On Xenon, the order of the two following function calls can nearly halve execution time.
        const uint32_t flags = ComputeGPTriangleFlags(ed0, ed1, ed2, defaultFlags);
        edgeCosines = DecodeEdgeCosinesUnmasked(ed0, ed1, ed2);
        return flags;
    }

    /// Extract edge cosines and GPTriangle flags for two triangle forming a quad from edge data stored in unit.
    /// Form two triangles from (v0,v1,v2) and (v3,v2,v1) respectively.
    static EA_FORCE_INLINE void ExtractQuadEdgeData(
        rwpmath::Vector3 & edgeCosinesA, 
        uint32_t & flagsA, 
        rwpmath::Vector3 & edgeCosinesB,
        uint32_t & flagsB,
        rwpmath::Vector3::InParam v0, rwpmath::Vector3::InParam v1, 
        rwpmath::Vector3::InParam v2, rwpmath::Vector3::InParam v3,
        const uint8_t * edgeData, 
        uint16_t defaultFlags)
    {
        EA_ASSERT(edgeData);
        const uint8_t ed0 = edgeData[0];
        const uint8_t ed1 = edgeData[1];
        const uint8_t ed2 = edgeData[2];
        const uint8_t ed3 = edgeData[3];
        const rwpmath::Vector4 edgeCosines = DecodeEdgeCosinesUnmasked(ed0, ed1, ed2, ed3);
        rwpmath::MaskScalar centralEdgeIsConvex;
        const rwpmath::VecFloat centralEdgeCosine(ComputeCentralEdgeCosine(centralEdgeIsConvex, v0, v1, v2, v3));
        edgeCosinesA = rwpmath::Vector3(edgeCosines[0], centralEdgeCosine, edgeCosines[2]);
        edgeCosinesB = rwpmath::Vector3(edgeCosines[3], centralEdgeCosine, edgeCosines[1]);
        // Use vertex disable flags from appropriate vertex, and edge convex flags from new edge
        const uint8_t ed4 = static_cast<uint8_t>(centralEdgeIsConvex.GetBool() ? static_cast<uint32_t>(EDGEFLAG_EDGECONVEX) : 0u);
        flagsA = ComputeGPTriangleFlags(ed0, ed4, ed2, ed0, ed1, ed2, defaultFlags);
        flagsB = ComputeGPTriangleFlags(ed3, ed4, ed1, ed3, ed2, ed1, defaultFlags);
    }

public:

    /// Access to the cluster this unit belongs to.
    const ClusteredMeshCluster & GetCluster() const
    {
        return *mCluster;
    }

protected:

    EA_FORCE_INLINE const uint8_t * GetUnitData(uint32_t offset = 0) const
    {
        EA_ASSERT(mCluster);
        // Note that on Wii, the offset is still in 16 byte multiples, despite vectors being 12 bytes.
        return (reinterpret_cast<const uint8_t*>(mCluster->vertexArray) + mCluster->unitDataStart * 16) + offset;
    }

    ClusterUnitBase(const ClusteredMeshCluster & cluster) : mCluster(&cluster)
    {

    }

    /// Get 3 vertices of a triangle
    template<uint8_t COMPRESSION>
    EA_FORCE_INLINE void GetTriVertices(
        rwpmath::Vector3 & vertex0,
        rwpmath::Vector3 & vertex1,
        rwpmath::Vector3 & vertex2,
        const uint8_t * indices,
        float vertexCompressionGranularity = 0.0f) const
    {
        EA_ASSERT(indices);
        mCluster->template Get3VerticesBase<COMPRESSION>(vertex0, vertex1, vertex2, 
            indices[0], indices[1], indices[2], vertexCompressionGranularity);
    }

    /// Get 4 vertices for a quad. 
    /// Form two triangles from (vertex0,vertex1,vertex2) and (vertex1,vertex2,vertex3).
    template<uint8_t COMPRESSION>
    EA_FORCE_INLINE void GetQuadVertices(
        rwpmath::Vector3 & vertex0,
        rwpmath::Vector3 & vertex1,
        rwpmath::Vector3 & vertex2,
        rwpmath::Vector3 & vertex3,
        const uint8_t * indices,
        float vertexCompressionGranularity = 0.0f) const
    {
        EA_ASSERT(indices);
        mCluster->template Get4VerticesBase<COMPRESSION>(vertex0, vertex1, vertex2, vertex3,
            indices[0], indices[1], indices[2], indices[3], vertexCompressionGranularity);
    }

    static EA_FORCE_INLINE uint16_t GetSubIDDynamic(const uint8_t * idData, uint8_t numBytes, uint16_t defaultId = 0u)
    {
        switch(numBytes)
        {
        default:
        case 0: return defaultId;
        case 1:
            {
                EA_ASSERT(idData);
                return (uint16_t) idData[0];
            }
        case 2:
            {
                EA_ASSERT(idData);
                return (uint16_t) ((uint16_t) idData[0] | (((uint16_t) idData[1]) << 8));
            }
        }
    }
    /// Combine 16-bit group and surface IDs into a 32-bit value.
    static EA_FORCE_INLINE uint32_t CombineSubIDs(uint16_t groupId, uint16_t surfaceId)
    {
        return ((uint32_t) (groupId)) | ((uint32_t) (surfaceId << 16));
    }
    /// Return up to 4 bytes of combined group and surface ID
    static EA_FORCE_INLINE uint32_t LoadID(const uint8_t * idData, const uint8_t unitFlags, 
        uint8_t groupIdSize, uint8_t surfaceIdSize, uint16_t defaultGroupId = 0u, uint16_t defaultSurfaceId = 0u)
    {
        // Note: we'll assume that if the ID is zero bytes we want to return 0, not the default ID.
        // We only return the default ID if it is 1 or 2 bytes but not specified in the stream.
        switch (unitFlags & (UNITFLAG_GROUPID|UNITFLAG_SURFACEID))
        {
        default:
        case 0u:
            return CombineSubIDs(defaultGroupId, defaultSurfaceId);
        case UNITFLAG_GROUPID:
            return CombineSubIDs(GetSubIDDynamic(idData, groupIdSize), defaultSurfaceId);
        case UNITFLAG_SURFACEID:
            return CombineSubIDs(defaultGroupId, GetSubIDDynamic(idData, surfaceIdSize));
        case UNITFLAG_GROUPID|UNITFLAG_SURFACEID:
            return CombineSubIDs(GetSubIDDynamic(idData, groupIdSize), GetSubIDDynamic(idData + groupIdSize, surfaceIdSize));
        }
    }

    /// Return up to 2 bytes of group or surface ID
    static EA_FORCE_INLINE uint32_t LoadID(const uint8_t * idData, const uint8_t unitFlags, const uint8_t idFlags, 
        uint8_t groupIdSize, uint8_t surfaceIdSize, uint16_t defaultId = 0u)
    {
        // Note: we'll assume that if the ID is zero bytes we want to return 0, not the default ID.
        // We only return the default ID if it is 1 or 2 bytes but not specified in the stream.
        switch (unitFlags & idFlags)
        {
        default:
        case 0u:
            return defaultId;
        case UNITFLAG_GROUPID:
                return GetSubIDDynamic(idData, groupIdSize);
        case UNITFLAG_SURFACEID:
            {
                uint32_t offset = (unitFlags & UNITFLAG_GROUPID) ? groupIdSize : 0u;
                return GetSubIDDynamic(idData + offset, surfaceIdSize);
            }
        }
    }

public:

    static EA_FORCE_INLINE rwpmath::VecFloat GetVecFloatPISquared()
    {
        static const float value = 9.8696044f;
        return rwpmath::VecFloat(value);
    }

    static EA_FORCE_INLINE rwpmath::Vector3 GetVector3PISquared()
    {
        rwpmath::VecFloat piSquared(GetVecFloatPISquared());
        return rwpmath::Vector3(piSquared, piSquared, piSquared);
    }

    static EA_FORCE_INLINE rwpmath::Vector4 GetVector4PISquared()
    {
        rwpmath::VecFloat piSquared(GetVecFloatPISquared());
        return rwpmath::Vector4(piSquared, piSquared, piSquared, piSquared);
    }

    static EA_FORCE_INLINE rwpmath::VecFloat GetVecFloatPISquaredBy8()
    {
        static const float value = 1.23370055f;
        return rwpmath::VecFloat(value);
    }

    static EA_FORCE_INLINE rwpmath::Vector4 GetVector4PISquaredBy8()
    {
        rwpmath::VecFloat piSquaredBy8(GetVecFloatPISquaredBy8());
        return rwpmath::Vector4(piSquaredBy8, piSquaredBy8, piSquaredBy8, piSquaredBy8);
    }

#if defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU)

    /// Return a mask in each component of vector intrinsic for extracting encoded edge cosine from edge data.
    static EA_FORCE_INLINE rw::math::vpl::VectorIntrinsic GetEdgeCosineMask()
    {
        EA_COMPILETIME_ASSERT(EDGEFLAG_ANGLEMASK == ((1<<5)-1));
        const rw::math::vpl::VectorIntrinsic bitMask = rw::math::vpl::VecLoad5BitImm_Int<1>();
        const rw::math::vpl::VectorIntrinsic byteMask = rw::math::vpl::VecShiftLeftImm_UInt<5>(bitMask);
        const rw::math::vpl::VectorIntrinsic mask = rw::math::vpl::VecSub_UInt(byteMask, bitMask);
        return mask;
    }

    /// Decode up to four edge cosines from the byte-encoded versions in edgedata with possible flags set.
    static EA_FORCE_INLINE rw::math::vpl::VectorIntrinsic DecodeEdgeCosinesUnmasked(
        const rw::math::vpl::VectorIntrinsic & edgedata,
        const rw::math::vpl::VectorIntrinsic & mask = GetEdgeCosineMask())
    {
        // Mask out the flags from the edgedata, leaving just the encoded cosines
        const rw::math::vpl::VectorIntrinsic i = rw::math::vpl::VecAnd(edgedata, mask);
        return DecodeEdgeCosines(i);
    }

    /// Decode up to four edge cosines from the byte-encoded versions in edgedata once flag bits have been masked out.
    static EA_FORCE_INLINE rw::math::vpl::VectorIntrinsic DecodeEdgeCosines(
        const rw::math::vpl::VectorIntrinsic & edgedata)
    {
        const rw::math::vpl::VectorIntrinsic piSquaredOver8 = GetVector4PISquaredBy8().mV;
        // exponent = (-edgedata)<<23;
        const rw::math::vpl::VectorIntrinsic zero = rw::math::vpl::VecGetZero();
        const rw::math::vpl::VectorIntrinsic minusI = rw::math::vpl::VecSub_Int(zero, edgedata);
        const rw::math::vpl::VectorIntrinsic exponent = rw::math::vpl::VecShiftLeftImm_UInt<23>(minusI);
        // edgeCosine = 1.0f - float(piSquared/8 + exponent)
        const rw::math::vpl::VectorIntrinsic one = rw::math::vpl::VecGetOne();
        const rw::math::vpl::VectorIntrinsic OneMinusCosines = rw::math::vpl::VecAdd_Int(piSquaredOver8, exponent);
        const rw::math::vpl::VectorIntrinsic edgeCosines = rw::math::vpl::VecSub(one, OneMinusCosines);
        return edgeCosines;
    }

#endif // defined(EA_PLATFORM_XENON) || defined(EA_PLATFORM_PS3) || defined(EA_PLATFORM_PS3_SPU)

private:

    const ClusteredMeshCluster * mCluster;
};

}   // namespace collision
}   // namespace rw

#endif  // !PUBLIC_RW_COLLISION_CLUSTER_UNIT_BASE_H

