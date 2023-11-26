// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_SCALEDCLUSTEREDMESH_H
#define PUBLIC_RW_COLLISION_SCALEDCLUSTEREDMESH_H
/*************************************************************************************************************

 File: scaledclusteredmesh.h

 Purpose: A wrapper class around the given clusteredMesh which performs
 a uniform scaling before running line and bbox queries.
*/
#include "rw/collision/procedural.h"
#include "rw/collision/clusteredmeshbase.h"
#include <rw/collision/triangle.h>

namespace rw
{
namespace collision
{

class ScaledClusteredMesh : public rw::collision::Procedural
{
private:

    /// @brief Constructor for scaled mesh. Note it is private, use Initialize() instead.
    /// @param clusteredMesh Pointer to the original clustered mesh to scale
    /// @param scale The scale to apply to the clustered mesh (must be greater than zero)
    ScaledClusteredMesh(rw::collision::ClusteredMesh * clusteredMesh, const float scale = 1.0f)
        : Procedural(clusteredMesh->GetVolumeCount(), &ScaledClusteredMesh::s_vTable)
        , m_clusteredMesh(clusteredMesh)
    {
        SetScale(scale);
    }
public:
  
    /// @brief Resource descriptor for the ScaledClusteredMesh
    /// @param rw::collision::ClusteredMesh* Pointer to clustered mesh (not used)
    /// @param float Scale to apply (not used)
    /// @return EA::Physics::SizeAndAlignment Size and alignment requirements for ScaledClusteredMesh object
    static EA::Physics::SizeAndAlignment GetResourceDescriptor(rw::collision::ClusteredMesh*, const float)
    {
        return EA::Physics::SizeAndAlignment(sizeof(ScaledClusteredMesh), RWMATH_VECTOR3_ALIGNMENT);
    }

    /// @brief Function to initialize a ScaledClusteredMesh
    /// @param resource Memory to use to create ScaledClusteredMesh
    /// @param clusteredMesh Pointer to the original clustered mesh
    /// @param scale Scale to apply to the given clustered mesh (default = 1)
    /// @return ScaledClusteredMesh* Pointer to created ScaledClusteredMesh
    static ScaledClusteredMesh * Initialize(const EA::Physics::MemoryPtr & resource, rw::collision::ClusteredMesh * clusteredMesh, const float scale = 1.0f)
    {
        EA_ASSERT(clusteredMesh);
        return new (resource.GetMemory()) ScaledClusteredMesh(clusteredMesh, scale);
    }

    /// @brief API required by allocators to free any memory allocated during construction
    EA_FORCE_INLINE void Release()
    {
    }

    /// @brief Sets the scale and inverse scale on the clusteredMesh and updates its bounding box
    /// @param scale The scale to apply to the clustered mesh (must be greater than zero).
    EA_FORCE_INLINE void SetScale(const float scale)
    {
        EA_ASSERT(scale > 0.0f);
        m_scale    = scale;
        m_invScale = rwpmath::Reciprocal(scale);
        // Update the bounding box as well
        UpdateThis();
    }

    /// @brief Return the scale applied to the clusteredMesh
    /// @return float The scale applied to the clusteredMesh
    EA_FORCE_INLINE float GetScale() const
    {
        return m_scale;
    }

    /// @brief Access to the clusteredMesh
    /// @return rw::collision::ClusteredMesh* Pointer to the clusteredMesh
    EA_FORCE_INLINE rw::collision::ClusteredMesh * GetClusteredMesh() const
    {
        return m_clusteredMesh;
    }

    // *****************************************************************************************************
    // Virtual functions required by the Aggregate interface

    EA_FORCE_INLINE uint32_t GetSizeThis()
    {
        return sizeof(ScaledClusteredMesh);
    }

    /// @brief Updates the bounding box of the clusteredMesh
    void UpdateThis();

    /// @brief Performs a line intersection test against the scaled clusteredMseh.
    /// @param lineQuery The line query object used to perform the query and store the results
    /// @param tm Transform matrix which defines the transform from the clustered mesh space to world space.
    /// @return uint32_t Number of results found
    uint32_t LineIntersectionQueryThis(rw::collision::VolumeLineQuery *lineQuery, const rwpmath::Matrix44Affine *tm);

    /// @brief Performs a line intersection test against the scaled clusteredMesh.
    /// @param bboxQuery The bbox query object used to perform the query and store the results
    /// @param tm Transform matrix which defines the transform from the clustered mesh space to world space.
    /// @return uint32_t Number of results found
    uint32_t BBoxOverlapQueryThis(rw::collision::VolumeBBoxQuery *bboxQuery, const rwpmath::Matrix44Affine *tm);
       
private:
    static rw::collision::Procedural::VTable s_vTable;   ///< The vtable for a scaledClusteredMesh object
    float m_scale;                                       ///< The scale applied to the clusteredMesh
    float m_invScale;                                    ///< The inverse of the scale applied to the clusteredMesh
    rw::collision::ClusteredMesh * m_clusteredMesh;      ///< Pointer to the clustered mesh to scale;
};


} // namespace collision
} // namespace rw


#endif // PUBLIC_RW_COLLISION_SCALEDCLUSTEREDMESH_H