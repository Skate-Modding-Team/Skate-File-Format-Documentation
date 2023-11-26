// (c) Electronic Arts. All Rights Reserved.

#ifndef PUBLIC_RW_COLLISION_MESHBUILDER_VERTEXMERGER_H
#define PUBLIC_RW_COLLISION_MESHBUILDER_VERTEXMERGER_H


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <coreallocator/icoreallocator_interface.h>

#include <rw/collision/meshbuilder/common.h>

#include <rw/collision/meshbuilder/detail/containers.h>
#include <rw/collision/meshbuilder/detail/spatialmap.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


/**
\brief Static class providing utilities for merging coincident vertices of a collection of triangles.

Typically, the triangles in a mesh share vertices. This sharing comprises the connectivity, or
topology of the mesh, and for example is modelled by an artist in Maya.

Occasionally, exported triangle data has non-optimal sharing. This occurs when vertices were
unshared explicitly on export, resulting in a triangle soup with no sharing of vertices.
It can also occur when a complex model is modelled in parts, such that vertices are not shared
at the regions where the component parts touch.

In all such cases the unshared vertices that could be shared are characterized by having
identical vertex positions (for the purposes of collision we ignore the presense of different
vertex normals, texture coordinates or colors as can occur in graphics meshes). Any vertices
with identical positions can are candidates for sharing. All vertices with a given unique
position are unified and replaced by a single representative vertex at that location - duplicates
are removed. The collection of triangles must of course be updated to reference the single
remaining vertex in each merged set in place of the removed duplicates.

This utility can be used to merge coincident vertices in a collection of triangles.
The merging is fuzzy and controlled by a supplied tolerance distance within which vertex
positions are considered practically identical.

The merging is performed using a spatial map data structure which is allocated internally
using an allocator supplied by the caller. The spatial map is freed before return.

The process is phrased in two parts, each corresponding to a method. In the first part a
vertex map is built up describing which vertices are to be replaced by which other
vertices. In the second, this map is applied to the collection of triangles to update
their vertex indices to reference only the remaining vertices.

\note The vertex collection is not updated. Instead it is allowed to remain as-is,
with duplicate vertices "removed" by the merging simply remaining unreferenced in the
collection.
*/
class VertexMerger
{

public:

    typedef meshbuilder::AABBoxType AABBoxType;

    typedef detail::VertexList VertexList;
    typedef detail::TriangleList TriangleList;
    typedef detail::IDList IDList;

    /**
    \brief Merges vertices which are within a maximum tolerance distance of one other.

    This method performs the first phase of a two-part merging process. In this phase
    the sets of coincident vertices are identified. A vertex map is populated and returned
    to the caller. This map describes which vertices were replaced by which other vertices.
    The returned map should be passed to the UpdateTriangleVertexIndices method, which
    is responsible for updating the collection of triangles.

    \note The vertex collection is not updated. Instead it is allowed to remain as-is,
    with duplicate vertices "removed" by the merging simply remaining unreferenced in the
    collection.

    \param vertexGroup                      Returned vertex index map.
    \param spatialMapAllocator              An allocator to be used for internal allocation of a temporary spatial map.
    \param aabbox                           A caller-calculated tight axis-aligned bounding box containing all vertices.
    \param vertexMergeDistanceTolerance     Tolerance separating distance within which vertices are considered coincident.
    \param vertices                         The collection of vertices to be merged.
    */
    static bool MergeVertexGroups(
        IDList &vertexGroup,
        EA::Allocator::ICoreAllocator &spatialMapAllocator,
        const AABBoxType &aabbox,
        const rwpmath::VecFloat &vertexMergeDistanceTolerance,
        const VertexList &vertices);

    /**
    \brief Updates the vertex indices of a collection of triangles with a vertex index mapping
    computed by MergeVertexGroups.

    \param triangles                        A collection of triangles to be updated.
    \param vertexGroup                      Vertex index map computed by MergeVertexGroups.
    */
    static void UpdateTriangleVertexIndices(
        TriangleList &triangles,
        const IDList &vertexGroup);

private:

    typedef meshbuilder::VectorType VectorType;

    /**
    \brief Computes 1+log2 of an integer.

    \param val Integer in question.

    \return 1+log2 of integer
    */
    static uint32_t HowManyBits(uint32_t val);

    /*
    \brief Merges a range of SpatialMap Cell entries.

    \TODO should be a method of SpatialMap

    \param spatialMap The SpatialMap containing the entries.
    \param vertexGroup The grouped vertex indices.
    \param startEntry the start of the group of entries to compare.
    \param endEntry the end of the group of entries to compare.
    \param vertices The collection of input vertices.
    \param toleranceSquared The vertex merge distance tolerance squared.
    **/
    static void MergeRangeOfEntries(
        IDList &vertexGroup,
        const detail::SpatialMap &spatialMap,
        const uint32_t startEntry,
        const uint32_t endEntry,
        const VertexList &vertices,
        const rwpmath::VecFloat &toleranceSquared);

    /**
    \brief Compares a given SpatialMap entry against entries in a SpatialMap cell.

    Used to merge the given entry with any cell entries which are withing the vertex
    merge distance tolerance.

    \param spatialMap The SpatialMap containing the entries.
    \param vertexGroup The grouped vertex indices.
    \param vertices The collection of input vertices.
    \param x The X coordinate of the cell ID.
    \param y The Y coordinate of the cell ID.
    \param z The Z coordinate of the cell ID.
    \param comparisonEntry The entry to compare against the cell.
    \param toleranceSquared the vertex merge distance tolerance squared.
    */
    static void CompareEntryAgainstCell(
        IDList &vertexGroup,
        const detail::SpatialMap &spatialMap,
        const VertexList &vertices,
        const uint32_t x,
        const uint32_t y,
        const uint32_t z,
        const uint32_t comparisonEntry,
        const rwpmath::VecFloat &toleranceSquared);

    /*
    \brief Merges a spatial map entry with a range of other entries with higher vertex indices.

    \TODO should be a method of SpatialMap

    \param spatialMap The SpatialMap containing the entries.
    \param vertexGroup The grouped vertex indices.
    \param comparisonEntry The comparison entry.
    \param startEntry the start of the group of entries to compare.
    \param endEntry the end of the group of entries to compare.
    \param vertices The collection of input vertices.
    \param toleranceSquared The vertex merge distance tolerance squared.
    **/
    static bool MergeEntryWithHigherEntries(
        IDList &vertexGroup,
        const detail::SpatialMap &spatialMap,
        const uint32_t comparisonEntry,
        const uint32_t startEntry,
        const uint32_t endEntry,
        const VertexList &vertices,
        const rwpmath::VecFloat &toleranceSquared);

    /*
    \brief Merges a spatial map entry with a range of other entries with lower vertex indices.

    \TODO should be a method of SpatialMap

    \param spatialMap The SpatialMap containing the entries.
    \param vertexGroup The grouped vertex indices.
    \param comparisonEntry The comparison entry.
    \param startEntry the start of the group of entries to compare.
    \param endEntry the end of the group of entries to compare.
    \param vertices The collection of input vertices.
    \param toleranceSquared The vertex merge distance tolerance squared.
    **/
    static void MergeEntryWithLowerEntries(
        IDList &vertexGroup,
        const detail::SpatialMap &spatialMap,
        const uint32_t comparisonEntry,
        const uint32_t startEntry,
        const uint32_t endEntry,
        const VertexList &vertices,
        const rwpmath::VecFloat &toleranceSquared);

    /*
    \brief Merges a range of spatial map entries which may have already been merged

    \TODO should be a method of SpatialMap

    \param spatialMap The SpatialMap containing the entries.
    \param vertexGroup The grouped vertex indices.
    \param comparisonEntry The comparison entry.
    \param startEntry the start of the group of entries to compare.
    \param endEntry the end of the group of entries to compare.
    \param vertices The collection of input vertices.
    \param toleranceSquared The vertex merge distance tolerance squared.
    **/
    static void MergeHigherRangeOfEntries(
        IDList &vertexGroup,
        const detail::SpatialMap &spatialMap,
        const uint32_t startEntry,
        const uint32_t endEntry,
        const VertexList &vertices,
        const rwpmath::VecFloat &toleranceSquared);
};


} // namespace meshbuilder
} // namespace collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU

#endif // defined PUBLIC_RW_COLLISION_MESHBUILDER_VERTEXMERGER_H
