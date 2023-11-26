// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_MAPPEDARRAY_H
#define PUBLIC_RW_COLLISION_MAPPEDARRAY_H

/*************************************************************************************************************

 File: rwcmapped.hpp

 Purpose: Derived aggregate class containing an array of volumes.
 */


#include "rw/collision/common.h"
#include "rw/collision/aggregate.h"

namespace rw
{
namespace collision
{

class MappedArray;
class Volume;


/**
\brief Aggregate data consisting of an actual array of volumes.

This is an abstract class where the child volume data is stored as
an array of volumes. Classes derived from this will define the type of spatial
map used in the line and BBox queries.

\see SimpleMappedArray Mapped array with no spatial map.
\see KDTreeMappedArray

\importlib rwccore
*/
class MappedArray : public Aggregate
{
private:
    /**
    Used by GetNextVolumeThis to access the user data in the Aggregate::VolumeWalkerData.

    \see Aggregate::VolumeWalker
    */
    struct VolumeWalkerData
    {
        uint32_t m_nextVolumeIndex; /// The next volume index in the mapped array.
    };

public:
    /**
    \brief Get a specific child volume.

    \param index The index identifying the child in the volume array.
    \return Ptr to the child volume.
    */
    Volume *
    GetVolume(uint16_t index) const
    {
        EA_ASSERT(index < m_numVolumes);
        return m_volumes + index;
    }

    /**
    \brief Get a pointer to the memory allocated for the storage of the child volumes.
    \return Ptr to the volume array memory.
    */
    Volume *
    GetVolumeArray()
    {
        return m_volumes;
    }

    /**
    \brief Get a pointer to the memory allocated for the storage of the child volumes.
    \return Ptr to the volume array memory.
    */
    const Volume *
    GetVolumeArray() const
    {
        return m_volumes;
    }

    /**
    \brief Gets the next volume from the mapped array and sets it in the VolumeWalker.

    \return TRUE if there is a volume else FALSE.

    \see Aggregate::VolumeWalker
    */
    RwpBool
    GetNextVolumeThis(VolumeWalker & volumeWalker) const
    {
        VolumeWalkerData *iterator(reinterpret_cast<VolumeWalkerData *>(volumeWalker.GetUserData()));

        // If the iterator is invalid initialize it.
        if (!volumeWalker.IsValid())
        {
            iterator->m_nextVolumeIndex = 0;
            volumeWalker.SetValid();
        }

        // Check to see if there are more volumes to process.
        if (iterator->m_nextVolumeIndex < m_numVolumes)
        {
            volumeWalker.SetVolumePointer(&m_volumes[iterator->m_nextVolumeIndex]);
            volumeWalker.SetChildIndex(iterator->m_nextVolumeIndex);
            ++iterator->m_nextVolumeIndex;
            return TRUE;
        }

        // No more volumes, set the walker to Finished and return FALSE.
        volumeWalker.SetFinished();
        return FALSE;
    }


    /**
    \brief Clears all volume processed flags
    */
    void ClearAllProcessedFlags()
    {
        // As we are overriding base function we need to ensure that we clear this volumes processed flag
        ClearProcessedFlag();

        for (unsigned int i = 0; i < m_numVolumes; ++i)
        {
            Volume &vol = m_volumes[i];
            vol.ClearAllProcessedFlags();
        }
    }


    /**
    \brief Applies uniform scaling to all volumes in aggregate

    \param scale The scale factor to apply to the mapped array
    \param useProcessedFlags Flag to specify whether to use or ignore processed flags
    */
    void ApplyUniformScale(float scale, bool useProcessedFlags = false)
    {
        EA_ASSERT(scale > 0.0f);

        if (!useProcessedFlags || !(m_flags & AGGREGATEFLAG_ISPROCESSED))
        {
            for (unsigned int i = 0; i < m_numVolumes; ++i)
            {
                Volume &vol = m_volumes[i];
                vol.ApplyUniformScale(scale, useProcessedFlags);
            }

            Update();

            if (useProcessedFlags)
            {
                SetProcessedFlag();
            }
        }
    }


    /**
    \internal

    \brief MappedArray Aggregate Constructor.

    \param numVolumes Number of child volumes in this aggregate.
    \param vTable Ptr to derived call function table.

    \note Do not call this directly. Use the Initialize function in the derived class
          to create a new object
    */
    MappedArray(uint32_t numVolumes, VTable *vTable)
    : Aggregate(numVolumes, vTable)
    {
    }

    /**
    \internal
    */
    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        // Serialize base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<Aggregate*>(this), "Aggregate");

        ar.TrackInternalPointer(m_volumes);

        // Serialize all the volumes
        ar & EA_SERIALIZATION_NAMED_STATIC_ARRAY(m_volumes, m_numVolumes);

    }

protected:
Volume *              m_volumes;   ///< Array of child volumes

    // This padding is here to avoid relying on compiler specific padding behaviour
#if 4 == EA_PLATFORM_PTR_SIZE
    uint32_t padkdtma[3];
#elif 8 == EA_PLATFORM_PTR_SIZE
    uint32_t padkdtma[2];
#endif
};

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_MAPPEDARRAY_H
