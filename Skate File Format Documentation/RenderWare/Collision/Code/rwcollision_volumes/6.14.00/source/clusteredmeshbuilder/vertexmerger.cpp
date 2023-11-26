// (c) Electronic Arts. All Rights Reserved.


#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/meshbuilder/vertexmerger.h>


namespace rw
{
namespace collision
{
namespace meshbuilder
{


bool VertexMerger::MergeVertexGroups(
    IDList & vertexGroup,
    EA::Allocator::ICoreAllocator & spatialMapAllocator,
    const AABBoxType & aabbox,
    const rwpmath::VecFloat & vertexMergeDistanceTolerance,
    const VertexList & vertices)
{
    // The spatial map divides the physical space into cells. Each cell has
    // an ID which is determined by hashing its X, Y and Z coordinates. As
    // items are added to the map their X, Y and Z coordinates are used to
    // determine a hashed ID and they are added to the corresponding cell.

    // Determine the volume of the entire bbox
    const rwpmath::Vector3 boxsize(aabbox.Max() - aabbox.Min());
    const rwpmath::VecFloat vol(boxsize.GetX() * boxsize.GetY() * boxsize.GetZ());

    // Decide the best cell size. The cell count should be under 2^30 and
    // the cell size should be at least 5*tolerance. The cell size is the
    // Cubed root of the volume, of the bbox, divided by 2^30
    rwpmath::VecFloat cellSize = rwpmath::Pow(vol, rwpmath::VecFloat(1.0f / 3.0f)) / rwpmath::VecFloat(1024.0f);
    if (cellSize < rwpmath::VecFloat(5.0f) * vertexMergeDistanceTolerance)
    {
        cellSize = rwpmath::VecFloat(5.0f) * vertexMergeDistanceTolerance;
    }

    // The cell IDs are composed by hashing the cell X, Y and Z coordinates.
    // The hashing determines the granularity required to index the cells 
    // along each of the principal axis.
    rwpmath::VecFloat hashfactor = rwpmath::GetVecFloat_Half();
    // Avoid a divide by zero
    if (rwpmath::GetVecFloat_Zero() < cellSize)
    {
        hashfactor = rwpmath::VecFloat(1.0f) / cellSize;
    }

    // Determine how many bits are required for each dimension
    uint32_t xDimension = HowManyBits(static_cast<uint32_t>(static_cast<float>(boxsize.GetX()) * hashfactor));
    uint32_t yDimension = HowManyBits(static_cast<uint32_t>(static_cast<float>(boxsize.GetY()) * hashfactor));

#if defined EA_ASSERT_ENABLED
    uint32_t zDimension = HowManyBits(static_cast<uint32_t>(static_cast<float>(boxsize.GetZ()) * hashfactor));
    EA_ASSERT(xDimension + yDimension + zDimension <= 32);
#endif

    // NOTE: This implicitly uses MEM_TEMP, the temporary heap
    // Allocate the spatialmap
    detail::SpatialMap spatialMap(vertices.size(), xDimension, yDimension, &spatialMapAllocator);
    if (!spatialMap.IsValid())
    {
        return false;
    }

    const rwpmath::VecFloat ftemp = cellSize * rwpmath::VecFloat(0.2345f);
    const rwpmath::Vector3 hashbase = rwpmath::Vector3(aabbox.Min()) - rwpmath::Vector3(ftemp, ftemp, ftemp);
    uint32_t hx, hy, hz, hx1, hy1, hz1, hx0, hy0, hz0;

    // Insert the entry vertices into the spatialmap along with their hashed IDs.
    const uint32_t numVertices(vertices.size());
    for (uint32_t vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
    {
        rwpmath::Vector3 v(vertices[vertexIndex]);

        v -= hashbase;

        hx = static_cast<uint32_t>(static_cast<float>(v.GetX()) * hashfactor);
        hy = static_cast<uint32_t>(static_cast<float>(v.GetY()) * hashfactor);
        hz = static_cast<uint32_t>(static_cast<float>(v.GetZ()) * hashfactor);

        spatialMap.Insert(hx, hy, hz, vertexIndex);
    }

    // Close the Spatial Map to allow internal finalization its contents.
    spatialMap.Close();

    // Merge the vertices using the spatial map. Vertices which are within a
    // given tolerance of each other are merged to the vertex with the lower
    // index (the index used when adding the vertex to the ClusteredMeshBuilder).

    // NOTE: vertex merging occurs almost arbitrary, therefore the current
    // method cannot be relied upon to produce a specific result. The only
    // guarantee is that after the merge has taken place no two remaining
    // vertices will be within a given tolerance of each other.

    // Entries are merged in two steps. The first step involves comparing
    // all vertex pairs within a cell. The second step then compares each
    // vertex to entries in surround cells which are within a tolerance of
    // the vertex.

    // Determine the tolerance used to decide when an entry needs to be
    // compared to surrounding cells.
    const rwpmath::VecFloat toleranceSquared = vertexMergeDistanceTolerance * vertexMergeDistanceTolerance;

    // Obtain cell iterators
    detail::SpatialMap::CellIterator it = spatialMap.Begin();
    const detail::SpatialMap::CellIterator itEnd = spatialMap.End();

    // Iterate through each cell
    while (it != itEnd)
    {
        const uint32_t cellId = it->first;
        const uint32_t startEntry = spatialMap.GetFirstEntryInCell(cellId);
        const uint32_t endEntry = spatialMap.GetFirstEntryInNextCell(cellId);

        MergeRangeOfEntries(
            vertexGroup,
            spatialMap,
            startEntry,
            endEntry,
            vertices,
            toleranceSquared);

        // Increment the cell iterator
        ++it;
    }

    it = spatialMap.Begin();

    while (it != itEnd)
    {
        const uint32_t cellId = it->first;
        const uint32_t startEntry = spatialMap.GetFirstEntryInCell(cellId);
        const uint32_t endEntry = spatialMap.GetFirstEntryInNextCell(cellId);

        for (uint32_t entry = startEntry; entry < endEntry; ++entry)
        {
            // Compare the current higherVertex with all other vertices in cells within tolerance

            uint32_t vertexIndex = spatialMap.GetEntry(entry);
            rwpmath::Vector3 offsetV(vertices[vertexIndex]);
            offsetV -= hashbase;
            rwpmath::Vector3 v(vertices[vertexIndex]);

            hx = static_cast<uint32_t>(static_cast<float>(offsetV.GetX()) * hashfactor);
            hy = static_cast<uint32_t>(static_cast<float>(offsetV.GetY()) * hashfactor);
            hz = static_cast<uint32_t>(static_cast<float>(offsetV.GetZ()) * hashfactor);
            hx1 = static_cast<uint32_t>((static_cast<float>(offsetV.GetX()) + vertexMergeDistanceTolerance) * hashfactor);
            hy1 = static_cast<uint32_t>((static_cast<float>(offsetV.GetY()) + vertexMergeDistanceTolerance) * hashfactor);
            hz1 = static_cast<uint32_t>((static_cast<float>(offsetV.GetZ()) + vertexMergeDistanceTolerance) * hashfactor);
            hx0 = static_cast<uint32_t>((static_cast<float>(offsetV.GetX()) - vertexMergeDistanceTolerance) * hashfactor);
            hy0 = static_cast<uint32_t>((static_cast<float>(offsetV.GetY()) - vertexMergeDistanceTolerance) * hashfactor);
            hz0 = static_cast<uint32_t>((static_cast<float>(offsetV.GetZ()) - vertexMergeDistanceTolerance) * hashfactor);

            if (hx < hx1)
            {
                if (hy < hy1)
                {
                    if (hz < hz1)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx1,
                            hy1,
                            hz1,
                            entry,
                            toleranceSquared);
                    }
                    else if (hz > hz0)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx1,
                            hy1,
                            hz0,
                            entry,
                            toleranceSquared);
                    }

                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx1,
                        hy1,
                        hz,
                        entry,
                        toleranceSquared);
                }
                else if (hy > hy0)
                {
                    if (hz < hz1)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx1,
                            hy0,
                            hz1,
                            entry,
                            toleranceSquared);
                    }
                    else if (hz > hz0)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx1,
                            hy0,
                            hz0,
                            entry,
                            toleranceSquared);
                    }

                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx1,
                        hy0,
                        hz,
                        entry,
                        toleranceSquared);
                }

                if (hz < hz1)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx1,
                        hy,
                        hz1,
                        entry,
                        toleranceSquared);
                }
                else if (hz > hz0)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx1,
                        hy,
                        hz0,
                        entry,
                        toleranceSquared);
                }

                CompareEntryAgainstCell(
                    vertexGroup,
                    spatialMap,
                    vertices,
                    hx1,
                    hy,
                    hz,
                    entry,
                    toleranceSquared);
            }
            else if (hx > hx0)
            {
                if (hy < hy1)
                {
                    if (hz < hz1)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx0,
                            hy1,
                            hz1,
                            entry,
                            toleranceSquared);
                    }
                    else if (hz > hz0)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx0,
                            hy1,
                            hz0,
                            entry,
                            toleranceSquared);
                    }

                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx0,
                        hy1,
                        hz,
                        entry,
                        toleranceSquared);
                }
                else if (hy > hy0)
                {
                    if (hz < hz1)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx0,
                            hy0,
                            hz1,
                            entry,
                            toleranceSquared);
                    }
                    else if (hz > hz0)
                    {
                        CompareEntryAgainstCell(
                            vertexGroup,
                            spatialMap,
                            vertices,
                            hx0,
                            hy0,
                            hz0,
                            entry,
                            toleranceSquared);
                    }

                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx0,
                        hy0,
                        hz,
                        entry,
                        toleranceSquared);
                }

                if (hz < hz1)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx0,
                        hy,
                        hz1,
                        entry,
                        toleranceSquared);
                }
                else if (hz > hz0)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx0,
                        hy,
                        hz0,
                        entry,
                        toleranceSquared);
                }

                CompareEntryAgainstCell(
                    vertexGroup,
                    spatialMap,
                    vertices,
                    hx0,
                    hy,
                    hz,
                    entry,
                    toleranceSquared);
            }

            if (hy < hy1)
            {
                if (hz < hz1)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx,
                        hy1,
                        hz1,
                        entry,
                        toleranceSquared);
                }
                else if (hz > hz0)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx,
                        hy1,
                        hz0,
                        entry,
                        toleranceSquared);
                }

                CompareEntryAgainstCell(
                    vertexGroup,
                    spatialMap,
                    vertices,
                    hx,
                    hy1,
                    hz,
                    entry,
                    toleranceSquared);
            }
            else if (hy > hy0)
            {
                if (hz < hz1)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx,
                        hy0,
                        hz1,
                        entry,
                        toleranceSquared);
                }
                else if (hz > hz0)
                {
                    CompareEntryAgainstCell(
                        vertexGroup,
                        spatialMap,
                        vertices,
                        hx,
                        hy0,
                        hz0,
                        entry,
                        toleranceSquared);
                }

                CompareEntryAgainstCell(
                    vertexGroup,
                    spatialMap,
                    vertices,
                    hx,
                    hy0,
                    hz,
                    entry,
                    toleranceSquared);
            }

            if (hz < hz1)
            {
                CompareEntryAgainstCell(
                    vertexGroup,
                    spatialMap,
                    vertices,
                    hx,
                    hy,
                    hz1,
                    entry,
                    toleranceSquared);
            }
            else if (hz > hz0)
            {
                CompareEntryAgainstCell(
                    vertexGroup,
                    spatialMap,
                    vertices,
                    hx,
                    hy,
                    hz0,
                    entry,
                    toleranceSquared);
            }
        }

        // Increment the cell iterator
        ++it;
    }

    // spatialMap is now redundant so deallocate memory
    spatialMap.Release();

    // return success
    return true;
}


EA_FORCE_INLINE uint32_t VertexMerger::HowManyBits(uint32_t val)
{
    uint32_t i;
    for (i = 0; val; val >>= 1, ++i) {}
    return i;
}


void VertexMerger::MergeRangeOfEntries(
    IDList & vertexGroup,
    const detail::SpatialMap &spatialMap,
    const uint32_t startEntry,
    const uint32_t endEntry,
    const VertexList &vertices,
    const rwpmath::VecFloat & toleranceSquared)
{
    // Compare all pairs of entries with indices higher than the comparison vertex
    for (uint32_t higherEntry = startEntry ; higherEntry < endEntry ; ++higherEntry)
    {
        uint32_t higherVertexIndex = spatialMap.GetEntry(higherEntry);
        VectorType higherVertex = vertices[higherVertexIndex];

        // Compare the current higherVertex with all other vertices in its cell
        for (uint32_t lowerEntry = startEntry ; lowerEntry < higherEntry ; ++lowerEntry)
        {
            uint32_t lowerVertexIndex = spatialMap.GetEntry(lowerEntry);
            if (rwpmath::MagnitudeSquared(rwpmath::Vector3(higherVertex - vertices[lowerVertexIndex])) < (toleranceSquared))
            {
                if (lowerVertexIndex > vertexGroup[lowerVertexIndex])
                {
                    lowerVertexIndex = vertexGroup[lowerVertexIndex];
                }

                EA_ASSERT(lowerVertexIndex == vertexGroup[lowerVertexIndex]);
                vertexGroup[higherVertexIndex] = lowerVertexIndex;
                break;
            }
        }
    }
}


void VertexMerger::CompareEntryAgainstCell(
    IDList &vertexGroup,
    const detail::SpatialMap &spatialMap,
    const VertexList &vertices,
    const uint32_t x,
    const uint32_t y,
    const uint32_t z,
    const uint32_t comparisonEntry,
    const rwpmath::VecFloat & toleranceSquared)
{
    // Get the start and end entries for the cell
    const uint32_t startEntry = spatialMap.GetFirstEntryInCell(x, y, z);
    const uint32_t endEntry = spatialMap.GetFirstEntryInNextCell(x, y, z);

    // Check for an empty cell
    if (startEntry == endEntry)
    {
        return;
    }

    // Find the first entry with an index higher than the comparison vertex in the cell
    uint32_t higherEntry = spatialMap.FindHigherIndexEntry(
        x, y, z,
        comparisonEntry);

    // Merge the comparison entry with lower cell entries
    MergeEntryWithLowerEntries(
        vertexGroup,
        spatialMap,
        comparisonEntry,
        startEntry,
        higherEntry,
        vertices,
        toleranceSquared);

    // Merge the comparison entry with greater cell entries
    bool merged = MergeEntryWithHigherEntries(
        vertexGroup,
        spatialMap,
        comparisonEntry,
        higherEntry,
        endEntry,
        vertices,
        toleranceSquared);

    // If a merge has taken place with the higher entries then all
    // higher entries need to be re-merged
    if (merged)
    {
        MergeHigherRangeOfEntries(
            vertexGroup,
            spatialMap,
            higherEntry,
            endEntry,
            vertices,
            toleranceSquared);
    }
}


bool VertexMerger::MergeEntryWithHigherEntries(
    IDList & vertexGroup,
    const detail::SpatialMap &spatialMap,
    const uint32_t comparisonEntry,
    const uint32_t startEntry,
    const uint32_t endEntry,
    const VertexList &vertices,
    const rwpmath::VecFloat & toleranceSquared)
{
    uint32_t comparisonVertexIndex = spatialMap.GetEntry(comparisonEntry);
    const VectorType & comparisonVertex = vertices[comparisonVertexIndex];

    // Translate the comparison vertex index to its merged index.
    if (comparisonVertexIndex > vertexGroup[comparisonVertexIndex])
    {
        comparisonVertexIndex = vertexGroup[comparisonVertexIndex];
    }

    bool merged = false;

    // Compare the comparison vertex against those with a higher index
    // if they have not already been merged to a vertex with a lower index.
    for (uint32_t entry = startEntry ; entry < endEntry ; ++entry)
    {
        uint32_t entryIndex = spatialMap.GetEntry(entry);

        // Check the entry has not already been merged to a vertex with a lower index
        if (vertexGroup[entryIndex] > comparisonVertexIndex)
        {
            if (rwpmath::MagnitudeSquared(rwpmath::Vector3(comparisonVertex - vertices[entryIndex])) < (toleranceSquared))
            {
                vertexGroup[entryIndex] = comparisonVertexIndex;
                merged = true;
            }
        }
    }

    return merged;
}


void VertexMerger::MergeEntryWithLowerEntries(
    IDList &vertexGroup,
    const detail::SpatialMap &spatialMap,
    const uint32_t comparisonEntry,
    const uint32_t startEntry,
    const uint32_t endEntry,
    const VertexList &vertices,
    const rwpmath::VecFloat & toleranceSquared)
{
    const uint32_t comparisonVertexIndex = spatialMap.GetEntry(comparisonEntry);
    const VectorType & comparisonVertex = vertices[comparisonVertexIndex];

    uint32_t currentEntry = startEntry;

    // While there are entries to compare
    while ((currentEntry < endEntry))
    {
        // Get the index
        uint32_t entryIndex = spatialMap.GetEntry(currentEntry);

        // Check the merge distance
        if (rwpmath::MagnitudeSquared(rwpmath::Vector3(comparisonVertex - vertices[entryIndex])) < (toleranceSquared))
        {
            if (entryIndex > vertexGroup[entryIndex])
            {
                entryIndex = vertexGroup[entryIndex];
            }

            EA_ASSERT(entryIndex == vertexGroup[entryIndex]);

            // If the vertex has not already been merged to a lower index
            if (vertexGroup[comparisonVertexIndex] > entryIndex)
            {
                vertexGroup[comparisonVertexIndex] = entryIndex;
                return;
            }
        }

        // Advance the lower entry
        ++currentEntry;
    }
}


void VertexMerger::MergeHigherRangeOfEntries(
    IDList & vertexGroup,
    const detail::SpatialMap &spatialMap,
    const uint32_t startEntry,
    const uint32_t endEntry,
    const VertexList &vertices,
    const rwpmath::VecFloat & toleranceSquared)
{
    // Compare all pairs of entries with indices higher than the comparison vertex
    for (uint32_t higherEntry = startEntry ; higherEntry < endEntry ; ++higherEntry)
    {
        uint32_t higherVertexIndex = spatialMap.GetEntry(higherEntry);
        const VectorType higherVertex = vertices[higherVertexIndex];

        // Compare the current higherVertex with all other vertices in its cell
        for (uint32_t lowerEntry = startEntry ; lowerEntry < higherEntry ; ++lowerEntry)
        {
            uint32_t lowerVertexIndex = spatialMap.GetEntry(lowerEntry);
            if (rwpmath::MagnitudeSquared(rwpmath::Vector3(higherVertex - vertices[lowerVertexIndex])) < (toleranceSquared))
            {
                if (lowerVertexIndex > vertexGroup[lowerVertexIndex])
                {
                    lowerVertexIndex = vertexGroup[lowerVertexIndex];
                }

                EA_ASSERT(lowerVertexIndex == vertexGroup[lowerVertexIndex]);

                // If the vertex has not already been merged to a lower index
                if (vertexGroup[higherVertexIndex] > lowerVertexIndex)
                {
                    vertexGroup[higherVertexIndex] = lowerVertexIndex;
                    break;
                }
            }
        }
    }
}


void VertexMerger::UpdateTriangleVertexIndices(
    TriangleList & triangles,
    const IDList & vertexGroup)
{
    const uint32_t numTriangles(triangles.size());
    for (uint32_t triangleIndex = 0; triangleIndex < numTriangles; ++triangleIndex )
    {
        uint32_t *const indices = triangles[triangleIndex].vertices;
        indices[0] = vertexGroup[indices[0]];
        indices[1] = vertexGroup[indices[1]];
        indices[2] = vertexGroup[indices[2]];
    }
}


} // namespace meshbuilder
} // collision
} // namespace rw


#endif // !defined EA_PLATFORM_PS3_SPU


