// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_TRIANGLEKDTREEPROCEDURAL_H
#define PUBLIC_RW_COLLISION_TRIANGLEKDTREEPROCEDURAL_H

/*************************************************************************************************************

 File: rwctrianglekdtreeprocedural.hpp

 Purpose: Procedural aggregate of triangles with KDTree spatial map.
 */


#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"
#include "rw/collision/kdtree.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/procedural.h"

namespace rw
{
namespace collision
{

class KDTree;


// Structure below has padding inserted by the compiler, which produces a warning.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4324)
#endif

/**
\brief A procedural aggregate consisting of a compressed triangle array indexed by a  rw::collision::KDTree.

This provides an efficient datastructure for a collision mesh. It can be created in a conditioning pipeline
using  rw::collision::conditioning::NodeTriangleKDTreeProceduralCreate.

\importlib rwccore
 */
class TriangleKDTreeProcedural : public Procedural
{
protected:
    // Dervied classes should call Initialize and Release
    TriangleKDTreeProcedural(uint32_t numVerts,
                             uint32_t numTris,
                             VTable *vTable,
                             uint32_t classSize);

public:

    /**
    \brief
    Structure defining a triangle in the mesh of the TriangleKDTreeProcedural. This refers by index to
    the vertices in the associated vertex array.

    \importlib rwccore
    */
    struct Triangle
    {
        uint32_t  indices[3];  ///< Array of three vertex indices for the triangle.
        uint32_t  id;          ///< Triangle group/surface ID.

        template <class Archive>
            void Serialize(Archive &ar, uint32_t /*version*/)
        {
            ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(indices, 3);
            ar & EA_SERIALIZATION_NAMED_VALUE(id);
        }
    };

    uint32_t
    GetSizeThis() const;

    static EA::Physics::SizeAndAlignment
    GetResourceDescriptor(uint32_t numVerts,
                          uint32_t numTris,
                          uint32_t numNodes,
                          const rw::collision::AABBox &bbox,
                          const VTable *vTable = &sm_vTable,
                          uint32_t classSize = sizeof(TriangleKDTreeProcedural));

    static TriangleKDTreeProcedural *
    Initialize(const EA::Physics::MemoryPtr& resource,
               uint32_t numVerts,
               uint32_t numTris,
               uint32_t numNodes,
               const rw::collision::AABBox &bbox,
               VTable *vTable = &sm_vTable,
               uint32_t classSize = sizeof(TriangleKDTreeProcedural));

    static void
    Release(TriangleKDTreeProcedural *triKDTree);

    void
        Release();


    /**
    \return The vertex array used for the triangles in an rw::collision::TriangleKDTreeProcedural. Use
     rw::collision::TriangleKDTreeProcedural::GetTriangles to get the triangle indices.
    */
    rwpmath::Vector3 *
    GetVertices()
    {
        return m_verts;
    }

    /**
    \return The vertex array used for the triangles. Use  GetTriangles for the triangle indices.
    */
    const rwpmath::Vector3 *
    GetVertices() const
    {
        return m_verts;
    }

    /**
    \return The triangle array used by the TriangleKDTreeProcedural. The triangles index vertices
    in the vertex array.
     */
    Triangle *
    GetTriangles()
    {
        return m_tris;
    }

    /**
    \return The triangle array for the mesh. The triangles index into the vertex array.
     */
    const Triangle *
    GetTriangles() const
    {
        return m_tris;
    }

    /**
    \brief Get the flags of the specified triangle for the edges and face.

    This returns the convexity flags for each edge, the onesided flag, and the isenabled and normalisdirty.

    \param index the index of the triangle in the TriangleKDTreeProcedural.

    \return the flags for the triangle.  \see enum rw::collision::VolumeFlags
    */
    inline uint32_t
    GetTriangleFlags(uint32_t index) const
    {
        EA_ASSERT_FORMATTED(index < m_numVolumes, ("Index %d out of range.", index));
        return (((m_flags[index >> 3] >> (4 * (index & 7))) & 15) * VOLUMEFLAG_TRIANGLEONESIDED) |
                            VOLUMEFLAG_ISENABLED | VOLUMEFLAG_TRIANGLENORMALISDIRTY;
    }


    /**
    \brief Set the flags of the specified triangle for the edges and face.
    \see enum rw::collision::VolumeFlags.
    This only uses flags VOLUMEFLAG_TRIANGLEONESIDED, and VOLUMEFLAG_TRIANGLEEDGEiCONVEX,
    and you have to shift them right 4 bits.

    \note new flags = (old flags and ~set0) or set1.  For example setflags(i,1,0) will turn on flag1 and
    leave the other flags unchanged, and setflags(i,4,2) will turn on flag4, turn off flag2,
    and leave the others unchanged.

    \param index The index of the triangle in the TriangleKDTreeProcedural.
    \param set1 a number 0..15
    \param set0 a number 0..15.  Default 15 which clears all flags not in set1.
    */
    inline void
    SetTriangleFlags(uint32_t index, uint32_t set1, uint32_t set0 = 15)
    {
        EA_ASSERT_FORMATTED(index < m_numVolumes, ("Index %d out of range.", index));
        EA_ASSERT_FORMATTED(set1 <= 15, ("new flags %d too large.", set1));
        EA_ASSERT_FORMATTED(set0 <= 15, ("new flags %d too large.", set0));
        uint32_t i,j;
        i = index >> 3;
        j = 4 * (index & 7);
        m_flags[i] = (m_flags[i] & ~(set0 << j)) | (set1 << j);
    }


    /**
    \return The KDTree used by the TriangleKDTreeProcedural.
     */
    const rw::collision::KDTree *
    GetKDTree() const
    {
        return m_map;
    }

    /**
    \return The KDTree used by the TriangleKDTreeProcedural.
     */
    rw::collision::KDTree *
    GetKDTree()
    {
        return m_map;
    }

    // Check validity (only available in debug).
    RwpBool
    IsValid() const;

    // Functions used to fill in vtable
    void
    UpdateThis(void);

    RwpBool
    LineIntersectionQueryThis(VolumeLineQuery *lineQuery,
                              const rwpmath::Matrix44Affine *tm);

    RwpBool
    BBoxOverlapQueryThis(VolumeBBoxQuery *bboxQuery,
                         const rwpmath::Matrix44Affine *tm);

    struct ObjectDescriptor;

    static TriangleKDTreeProcedural * Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc);
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(const ObjectDescriptor & objDesc);

    // Return the information needed to allocate this object when deserializing
    const ObjectDescriptor GetObjectDescriptor() const;

    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        // Serialize base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<Procedural*>(this), "Procedural");

        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVerts);

        ar.TrackInternalPointer(m_map);
        ar & EA_SERIALIZATION_NAMED_VALUE(*m_map);

        ar.TrackInternalPointer(m_tris);
        ar.TrackInternalPointer(m_verts);
        ar.TrackInternalPointer(m_flags);

        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_verts, m_numVerts);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_tris, m_numVolumes);
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_flags, ((m_numVolumes+7)>>3));

        if(ar.IsLoading())
        {
            m_vTable = &sm_vTable;
            EA_ASSERT(m_vTable != NULL);
        }

    }

    float
    GetTriangleNormal(uint32_t i, rwpmath::Vector3 &result) const;

    void
    AutoSetFlags(float tolerance = 0.1f);

private:

    //  This is a utility used by AutoSetFlags

    RwpBool
    TriangleIsOutside(uint32_t i, uint32_t j, const rwpmath::Vector3 &inorm) const;

    void
    MateTriangles(uint32_t i, uint32_t j, float eps, float inradius, const rwpmath::Vector3 &inorm);


private:

    static VTable           sm_vTable;

    uint32_t                m_numVerts;

    // Triangle array

    Triangle *              m_tris;


    // Vertex array

    rwpmath::Vector3 *      m_verts;


    // KDTree map

    KDTree *                m_map;


    // four bits per triangle

    uint32_t *              m_flags;

    /**
        The following data is inherited from Aggregate.

    AABBox             m_AABB;
    VTable            *m_vTable;
    uint32_t           m_numTagBits;
    uint32_t           m_numVolumes;
    */
};


struct TriangleKDTreeProcedural::ObjectDescriptor
{
    ObjectDescriptor(uint32_t numVerts,
                    uint32_t numTris,
                    uint32_t numNodes,
                    const rw::collision::AABBox & bbox)
    {
        m_numVerts = numVerts;
        m_numTris = numTris;
        m_numNodes = numNodes;
        m_bbox = bbox;
    }

    ObjectDescriptor()
    {
        m_numVerts = 0;
        m_numTris = 0;
        m_numNodes = 0;
        m_bbox = AABBox();
    }

    uint32_t m_numVerts;
    uint32_t m_numTris;
    uint32_t m_numNodes;
    rw::collision::AABBox m_bbox;

    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVerts);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numTris);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numNodes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_bbox);
    }
};


inline TriangleKDTreeProcedural *
TriangleKDTreeProcedural::Initialize(const EA::Physics::MemoryPtr& resource, const ObjectDescriptor & objDesc)
{
    return Initialize(resource, objDesc.m_numVerts, objDesc.m_numTris, objDesc.m_numNodes, objDesc.m_bbox);
}


inline EA::Physics::SizeAndAlignment
TriangleKDTreeProcedural::GetResourceDescriptor(const ObjectDescriptor & objDesc)
{
    return GetResourceDescriptor(objDesc.m_numVerts, objDesc.m_numTris, objDesc.m_numNodes, objDesc.m_bbox);
}


inline const TriangleKDTreeProcedural::ObjectDescriptor TriangleKDTreeProcedural::GetObjectDescriptor() const
{
    return ObjectDescriptor(m_numVerts, m_map->GetNumEntries(), m_map->GetNumBranchNodes(), m_AABB);
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_TRIANGLEKDTREEPROCEDURAL_H
