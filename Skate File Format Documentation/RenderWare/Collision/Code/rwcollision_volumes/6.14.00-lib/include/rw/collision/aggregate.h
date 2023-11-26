// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_AGGREGATE_H
#define PUBLIC_RW_COLLISION_AGGREGATE_H

/*************************************************************************************************************

 File: rwcaggregate.hpp

 Purpose: Base class for aggregates (arrays of volumes).
 */


#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "rw/collision/volumelinequery.h"
#include "rw/collision/volumebboxquery.h"

namespace rw
{
namespace collision
{

class Aggregate;


/**
Enumeration for aggregate flags.

*/
enum AggregateFlag
{
    AGGREGATEFLAG_ISPROCESSED        = 0x0001, ///< indicating that aggregate has been processed (i.e. scaled)

    AGGREGATEFLAG_FORCEENUMSIZEINT = EAPHYSICS_FORCEENUMSIZEINT
};


/**
\brief The base class for aggregate volume data.

An aggregate contains data for a collection of child volumes where the children can be
both primitive or aggregate volumes. An aggregate can be either
a  rw::collision::MappedArray or a \ref rw::collision::Procedural. A MappedArray stores the child data
as an actual array of volumes while a Procedural stores it in a format optimised for
the specific type of volume collection.

\see rw::collision::AggregateVolume

\importlib rwccore
 */
class Aggregate
{
public:

    /**
    \brief Get the axis aligned bounding box which encloses the entire Aggregate.
    \return Reference to the aggregates AABB.
    */
    const AABBox &
    GetBBox() const
    {
        return m_AABB ;
    }

    /**
    \brief Get the number of child volumes in an aggregate. 
    
    If any of the child volumes is itself a container volume (I.e. MappedArray or Procedural), 
    the count does not include children of the child volume.

    \return The volume count at this level in an aggregate hierarchy.
    */
    uint32_t
    GetVolumeCount() const
    {
        return m_numVolumes;
    }

    /**
    \brief Get the tag relevant to one level down in an aggregate hierarchy.

    A tag is composed of the set of child indices concatenated together into a bit field.
    This identifies a specific volume in an aggregate of arbitrary depth. At any level in the
    aggregate hierarchy, the tag contains the child index at this level and the tag relevant
    to one level lower. Use  MappedArray::GetChildIndexFromTag to recover the child index
    from the original tag before calling this function.

    \param tag The tag identifying the lowest child volume.

    \return The tag relevant to one level down or Zero if this is the lowest level referred
          to by the tag.
    */
    uint32_t
    GetChildTagFromTag(uint32_t tag) const
    {
        //Shift tag right by number of bits used by this aggregate level
        return (tag >> m_numTagBits);
    }

    /**
    \brief Get the child index for this level in the aggregate hierarchy from the unique tag
    identifying a volume at a lower level.


    A tag is composed of the set of child indices concatenated together into a bit field.
    This identifies a specific volume in an aggregate of arbitrary depth. At any level in the
    aggregate hierarchy, the child index is held in the right N bits of the tag where N is
    sufficient to store the maximum number of children at this level. When the child index is
    recovered, the tag is right shifted by N to give the tag relevant to the next level down
    in the hierarchy. Use  MappedArray::GetChildTagFromTag to shift perform this right shift
    operation. A resulting tag of Zero indicates that you don't decend the hierarchy any further.

    \param tag The tag identifying the lowest child volume.

    To decend an aggregate hierarchy using tags, use something like:
    \code
    childIndex = agg->GetChildIndexFromTag(tag);
    tag = agg->GetChildTagFromTag(tag);
    vol = agg->GetVolume(childIndex);

    //Descend while child tag is not zero
    while(tag && vol->GetType() == VOLUMETYPEAGGREGATE)
    {
        //Get the child container aggregate
        agg = ((AggregateVolume *)vol)->GetAggregate();

        //Tag must be > 0 at this point
        childIndex = agg->GetChildIndexFromTag(tag);
        tag = agg->GetChildTagFromTag(tag);
        if(!agg->IsProcedural())
        {
            vol = ((MappedArray*)agg)->GetVolume(childIndex);
        }
    }

    \endcode

    \return The index of the child volume at this level in the hierarchy.
    */
    uint32_t
    GetChildIndexFromTag(uint32_t tag) const
    {
        EA_ASSERT(tag > 0);
        //Get the child id from the right m_numIDBits bits
        return ((~(0xffffffff << m_numTagBits)) & tag)-1;
    }

    /**
    \brief Creates the relevant based on a tag from one level down in an aggregate hierarchy.

    A tag is composed of the set of child indices concatenated together into a bit field.
    This identifies a specific volume in an aggregate of arbitrary depth. At any level in the
    aggregate hierarchy, the tag contains the child index at this level and the tag relevant
    to one level lower. Use  MappedArray::GetChildIndexFromTag to recover the child index
    from the original tag before calling this function.

    \param index The index of the child volume at this level.
    \param childTag The tag from the volumes lower down.

    \return The concatinated tag.

    \see GetChildIndexFromTag, GetTagFromChildIndexAndChildTag, GetTagFromChildIndexAndParentTag
    */
    uint32_t
    GetTagFromChildIndexAndChildTag(uint32_t index, uint32_t childTag) const
    {
        // Make sure the index small enough to fit in m_numTagBits.
        EA_ASSERT(index < ~(0xffffffff << m_numTagBits));
        // Make sure we won't overflow after we shift it.
        EA_ASSERT((childTag >> (32 - m_numTagBits)) == 0);
        return (childTag << m_numTagBits) + index + 1;
    }

    /**
    \brief Creates a new tag based on the accumulated tag data and a child index

    A tag is composed of the set of child indices concatenated together into a bit field.
    This identifies a specific volume in an aggregate of arbitrary depth. At any level in the
    aggregate hierarchy, the tag contains the child index at this level and the tag relevant
    to one level lower. Use  MappedArray::GetChildIndexFromTag to recover the child index
    from the original tag before calling this function.

    \param tag The tag accumulated so far - this is updated to be a new tag.
    \param numTagBits The number of tag bits used so far - this is updated to have the new number of bits used
    \param index The index of the child volume at this level.

    To create a tag for an aggregate hierarchy , use something like:
    \code
    uint32_t tag = 0;
    uint32_t numBitsUsed = 0;

    Volume* vol = part.GetVolume();

    while (vol->GetType() == VOLUMETYPEAGGREGATE)
    {
        // Descend to child volume and return the index
        uint32_t index = DecideNextVolume(vol);

        UpdateTagWithChildIndex(tag, numBitsUsed, index);
    }
    \endcode

    Alternatively given a specific point in the hierarcy the following will iterate over a MappedArray
    \code
    Iterate(MappedArray* array, uint32_t parentTag, uint32_t parentBitsUsed)
    {
        for (uint32_t i = 0; i < array->GetVolumeCount(); ++i)
        {
            uint32_t tag = parentTag;
            uint32_t numBitsUsed = parentBitsUsed;
            UpdateTagWithChildIndex(tag, numBitsUsed, i);

            DoSomethingWithVolumeAndTag(array->GetVolume(i), tag, numBitsUsed);
        }
    }
    \endcode


    \see GetChildIndexFromTag
    */
    void
    UpdateTagWithChildIndex(uint32_t& tag, uint32_t& numTagBits, uint32_t index) const
    {
        // Make sure the index small enough to fit in m_numTagBits.
        EA_ASSERT(index < ~(0xffffffff << m_numTagBits));
        // Make sure we won't overflow after we shift it.
        EA_ASSERT(numTagBits + m_numTagBits < 32);

        tag |= ((index+1) << numTagBits);
        numTagBits += m_numTagBits;
    }

    /**
    \brief Get the type of the derived aggregate object.
    \return The derived object type.
    */
    rw::collision::ObjectType
    GetType() const
    {
        return (m_vTable->m_type);
    }

    /**
    \brief Get the resource requirements of the derived aggregate object.
    \return The EA::Physics::SizeAndAlignment
    */
    EA::Physics::SizeAndAlignment
    GetResourceDescriptor()
    {
        return EA::Physics::SizeAndAlignment((this->*(m_vTable->m_GetSize))(), m_vTable->m_alignment);
    }

    /**
    \brief Test whether the derived aggregate object is a procedural type. I.E. The child volume
    data is stored in a  rw::collision::Procedural format.

    \return TRUE if the derived class is a procedural type, FALSE otherwise.
    */
    RwpBool
    IsProcedural() const
    {
        return (m_vTable->m_isProcedural);
    }

    /**
    \brief Update the bounding box of the aggregate after child volume data
    has been created or or modified. 
    
    If the aggregate has a dynamic spatial map then this will be updated as well.
    */
    void
    Update()
    {
        (this->*(m_vTable->m_Update))();
    }


    /**
    \brief Get the aggregate flags.

    The flags could be used to control various properties of the aggregate.
    \see SetFlags
    */
    inline uint32_t
    GetFlags() const
    {
        return m_flags;
    }


    /**
    \brief Set the aggregate flags.

    The flags could be used to control various properties of the aggregate.
    \see GetFlags
    */
    inline void
    SetFlags(uint32_t newflags)
    {
        m_flags = newflags;
    }


    /**
    \brief This query will test the input line query against the collision data in the derived
    class. 
    
    The input line query structure contains the results buffer and this function
    can be called multiple times to retrieve all intersections in the case of the
    buffer becoming full.

    \param lineQuery Initialized line query structure.
    \param tm The transform of the aggregate in the query frame.

    \return TRUE if the query finished, FALSE if the results buffer overflowed and
            LineIntersectionQuery needs to be called again.
    */
    RwpBool
    LineIntersectionQuery(VolumeLineQuery *lineQuery,
                         const rwpmath::Matrix44Affine *tm)
    {
        return (this->*(m_vTable->m_LineIntersectionQuery))(lineQuery, tm);
    }


    /**
    \brief This query will test the input bounding box query against the collision data in the derived
    class. 
    
    The input bbox query structure contains the VolRef results buffer and this function
    can be called multiple times to retrieve all overlaps in the case of the
    buffer becoming full.

    \param bboxQuery Initialized bounding box query structure.
    \param tm The transform of the aggregate in the query frame.

    \return TRUE if the query finished, FALSE if the stack overflowed or the results buffer could
            not hold all the results and BBoxOverlapQuery needs to be called again.
    */
    RwpBool
    BBoxOverlapQuery(VolumeBBoxQuery *bboxQuery,
                    const rwpmath::Matrix44Affine *tm)
    {
        return (this->*(m_vTable->m_BBoxOverlapQuery))(bboxQuery, tm);
    }

    /**
    The VolumeWalker is used in conjunction with GetNextVolume to iterate all the Volumes of an Aggregate.
    Each aggregate type may implement GetNextVolume and set the m_GetNextVolume vTable entry.

    The VolumeWalker has 64 Bytes or 4 QuadWords of space that can be used by the aggregate to store any
    state that is required to implement the walker. The data can be accessed by calling GetUserData.

    When constructed the VolumeWalker is marked as invalid. On the first call to GetNextVolume the
    aggregates implementation can check the validity of the walker by calling IsValid. If the walker is
    invalid the implementation knows to initialize the walker and mark is as valid by calling SetValid.

    When the aggregate has no more volumes to iterate it should call SetFinished on the walker and return
    FALSE from GetNextVolume. The walker can be reused by calling Initialize.

    If an aggregate needs to instance a volume the walker has space to do this, this can be accessed by
    calling GetVolumeInstance.

    The volume pointer in the walker must be set by calling SetVolumePointer regardless of if the
    implementor has instanced a volume or is referencing an external volume.
    */
    class VolumeWalker
    {
    private:
        Volume              m_volumeInstance;   /// A Volume that can be instanced into.
        const Volume  *     m_volumePointer;    /// A Volume pointer that can be set to a external volume or the volume instance.
        const Aggregate *   m_aggregate;        /// A pointer to the Aggregate that is being walked.
        uint32_t            m_childIndex;       /// Index of the current child volume referenced, 0xffffffff represents the walker is uninitialized
        RwpBool              m_finished;         /// An RwpBool to identify if the iterator has finished.
        uint8_t             userData[64];       /// 4 Quad words of space for iterator implementations.

    public:

        // ---- User API ----

        enum
        {
            WALKER_UNINITIALIZED = 0xffffffff  ///< Value stored in m_childIndex to represent that the walker is uninitialized.
        };

        /**
        Construct a walker and mark it as invalid.

        \note The first call to GetNextVolume should detect the iterator as being invalid and initialize the
        user data as required then mark the iterator as valid.
        */
        inline VolumeWalker()
            : m_childIndex(static_cast<uint32_t>(WALKER_UNINITIALIZED)),
              m_finished(FALSE)
        {
            EA_COMPILETIME_ASSERT(0 == sizeof(VolumeWalker) % rwcVOLUMEALIGNMENT); // VolumeWalker_size_is_not_a_multiple_of_rwcVOLUMEALIGNMENT
        }

        /**
        Construct a walker and initialize it using the aggregate. Once the walker has been initialized the first Volume can
        be accessed if calling Finished returns FALSE.
        */
        inline VolumeWalker(const Aggregate * aggregate)
            : m_aggregate(aggregate),
              m_childIndex(static_cast<uint32_t>(WALKER_UNINITIALIZED)),
              m_finished(FALSE)
        {
            EA_COMPILETIME_ASSERT(0 == sizeof(VolumeWalker) % rwcVOLUMEALIGNMENT); // VolumeWalker_size_is_not_a_multiple_of_rwcVOLUMEALIGNMENT

            if (FALSE == m_aggregate->GetNextVolume(*this))
            {
                EA_ASSERT_MSG((!IsValid()) || Finished(), ("GetNextVolume returned FALSE with a valid walker but hasn't called VolumeWalker::SetFinished"));
            }
        }

        /**
        Initialize the walker using the aggregate. Once the walker has been initialized the first Volume can
        be accessed if calling Finished returns FALSE.
        */
        inline void Initialize(const Aggregate * aggregate)
        {
            m_aggregate = aggregate;
            m_childIndex = static_cast<uint32_t>(WALKER_UNINITIALIZED);
            m_finished = FALSE;

            if (FALSE == m_aggregate->GetNextVolume(*this))
            {
                EA_ASSERT_MSG((!IsValid()) || Finished(), ("GetNextVolume returned FALSE with a valid walker but hasn't called VolumeWalker::SetFinished"));
            }
        }

        /**
        Gets the address of the volume.

        \return The Volume pointer. The Volume pointer is only valid if the walker is valid.
        \see IsValid
        */
        inline const Volume * operator->()
        {
            EA_ASSERT(IsValid());
            EA_ASSERT(!Finished());
            return m_volumePointer;
        }

        /**
        Gets a reference to the volume.

        \return The Volume reference. The Volume reference is only valid if the walker is valid.
        \see IsValid
        */
        inline const Volume & operator*()
        {
            EA_ASSERT(IsValid());
            EA_ASSERT(!Finished());
            return *m_volumePointer;
        }

        /**
        Gets the next volume.
        */
        inline void operator++()
        {
            EA_ASSERT(IsValid());
            EA_ASSERT(!Finished());
            if (FALSE == m_aggregate->GetNextVolume(*this))
            {
                EA_ASSERT_MSG(Finished(), ("GetNextVolume returned FALSE but hasn't called VolumeWalker::SetFinished"));
            }
        }

        /**
        Check to see if there are any more volumes.
        \return TRUE if more volumes else FALSE.
        */
        inline RwpBool Finished()
        {
            EA_ASSERT(IsValid());
            return m_finished;
        }

        /**
        Tests to see if the walker is valid.
        \return TRUE if valid else FALSE.
        */
        inline RwpBool IsValid()
        {
            RwpBool result(FALSE);

            if (static_cast<uint32_t>(WALKER_UNINITIALIZED) != m_childIndex)
            {
                result = TRUE;
            }

            return result;
        }

        /**
        Tests to see if the volume is an instance volume.
        \return TRUE if the volume is instanced else FALSE.
        */
        inline RwpBool IsVolumeInstanced()
        {
            EA_ASSERT(IsValid());
            EA_ASSERT(!Finished());

            RwpBool result(FALSE);

            if (&m_volumeInstance == m_volumePointer)
            {
                result = TRUE;
            }

            return result;
        }

        /**
        Get the current child index
        */
        inline uint32_t GetChildIndex()
        {
            EA_ASSERT(IsValid());
            EA_ASSERT(!Finished());
            return m_childIndex;
        }

        // ---- Implementor API ----

        /**
        Get the address of the instance volume in the walker. This should be used when GetNextVolume needs
        to instance a volume.

        e.g.
        TriangleVolume triVolume;
        *vIt.GetVolumeInstance() = triVolume;
        */
        inline Volume *GetVolumeInstance()
        {
            return &m_volumeInstance;
        }

        /**
        Set the iterators volume pointer.

        If the GetNextVolume function has instanced a volume into the walker the volume pointer should set
        to the address of the instance.

        e.g.
        vIt.SetVolumePointer(vIt.GetVolumeInstance());
        */
        inline void SetVolumePointer(const Volume * volume)
        {
            m_volumePointer = volume;
        }

        /**
        Set the walker as valid
        */
        inline void SetValid()
        {
            EA_ASSERT(!IsValid());
            m_childIndex = 0;
        }

        /**
        Set the walker as invalid
        */
        inline void SetInvalid()
        {
            m_childIndex = static_cast<uint32_t>(WALKER_UNINITIALIZED);
        }

        /**
        Set the flag that there are no more volumes
        */
        inline void SetFinished()
        {
            m_finished = TRUE;
        }

        /**
        Set the current child index
        */
        inline void SetChildIndex(uint32_t childIndex)
        {
            EA_ASSERT_MSG(WALKER_UNINITIALIZED != childIndex, ("Child index of 0xffffffff is reserved"));
            m_childIndex = childIndex;
        }

        /**
        Get a pointer to the user data inside the walker
        \return A void * pointer.
        */
        void *GetUserData()
        {
            return userData;
        }
    };

    /**
    \brief Gets the next Volume from the Aggregate.

    This method is used to iterate all the Volumes in an aggregate. If the aggregate has implemented the
    GetNextVolume API and set the corresponding vTable entry the method will be called.

    \return TRUE if there is a volume else FALSE. FALSE if the vTable entry is NULL.
    */
    RwpBool GetNextVolume(VolumeWalker & volumeWalker) const
    {
        if (m_vTable->m_GetNextVolume)
        {
            return (this->*(m_vTable->m_GetNextVolume))(volumeWalker);
        }

        return FALSE;
    }

    typedef uint32_t (Aggregate::*GetSizeFn)(void);

    typedef uint32_t (Aggregate::*GetAlignmentFn)(void);

    typedef RwpBool   (Aggregate::*IsProceduralFn)(void);

    typedef void     (Aggregate::*UpdateFn)(void);

    /**
    \internal
    */
    typedef RwpBool (Aggregate::*LineIntersectionQueryFn)(VolumeLineQuery *lineQuery,
                                                         const rwpmath::Matrix44Affine *tm );

    /**
    \internal
    */
    typedef RwpBool (Aggregate::*BBoxOverlapQueryFn)(VolumeBBoxQuery *bboxQuery,
                                                    const rwpmath::Matrix44Affine *tm );

    typedef RwpBool (Aggregate::*GetNextVolumeFn)(VolumeWalker & volumeWalker) const;

    typedef void (Aggregate::*ClearAllProcessedFlagsFn)();

    typedef void (Aggregate::*ApplyUniformScaleFn)(float scaleFactor, bool useProcessedFlags);


    /**
    \internal
    Aggregate virtual functions
    */
    struct VTable
    {
        rw::collision::ObjectType   m_type;
        GetSizeFn                   m_GetSize;
        uint32_t                    m_alignment;
        RwpBool                     m_isProcedural;
        UpdateFn                    m_Update;
        LineIntersectionQueryFn     m_LineIntersectionQuery;
        BBoxOverlapQueryFn          m_BBoxOverlapQuery;
        GetNextVolumeFn             m_GetNextVolume;
        ClearAllProcessedFlagsFn    m_ClearAllProcessedFlags;
        ApplyUniformScaleFn         m_ApplyUniformScale;
    };

    /**

    \internal
    */
    Aggregate(uint32_t numVolumes, VTable *vTable)
    : m_vTable(vTable),
      m_numTagBits(0),
      m_numVolumes(numVolumes),
      m_flags(0)
    {
        uint32_t nv = numVolumes; //Add 1 because a tag is index + 1
        while(nv)
        {
            nv>>=1;  //Count the number of right shifts to find the number of
            m_numTagBits++;  //Tag bits used by this level of aggregate
        }
    }

    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
    void Serialize(Archive &ar, uint32_t version)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numTagBits);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_numVolumes);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_AABB);

        if (version > 1)
        {
            ar & EA_SERIALIZATION_NAMED_VALUE(m_flags);
        }
        else
        {
            EA_ASSERT(ar.IsLoading());
            if (ar.IsLoading())
            {
                m_flags = 0;
            }
        }
    }

    /**
    \brief Set the aggregate processed flag.
    \see AggregateFlags, ClearAllProcessedFlags, ClearProcessedFlag
    */
    void SetProcessedFlag()
    {
        // Set processed flag
        m_flags |= AGGREGATEFLAG_ISPROCESSED;
    }

    /**
    \internal
    \brief Clears processed flag of this volume only
    Note: This will not clear child volume processed flags
    \see ClearAllProcessedFlags, SetProcessedFlag
    */
    void ClearProcessedFlag()
    {
        // Clear aggregate processed flag
        m_flags &= ~AGGREGATEFLAG_ISPROCESSED;
    }

    /**
    \brief Clears the processed flag.

    Clears the aggregate processed flag. Forwards call for aggregate to handle 
    child flag clearing.
    \see VolumeFlags, Volume::SetEnabled, SetProcessedFlag
    */
    void ClearAllProcessedFlags()
    {
        ClearProcessedFlag();

        if (m_vTable->m_ClearAllProcessedFlags)
        {
            (this->*(m_vTable->m_ClearAllProcessedFlags))();
        }
    }

    /**
    \brief Applies uniform scaling to aggregate

    \param scale The scale factor to apply to the mapped array
    \param useProcessedFlags Flag to specify whether to use or ignore processed flags
    */
    void ApplyUniformScale(float scale, bool useProcessedFlags = false)
    {
        EA_ASSERT(scale > 0.0f);

        if (m_vTable->m_ApplyUniformScale)
        {
            (this->*(m_vTable->m_ApplyUniformScale))(scale, useProcessedFlags);
        }
    }

protected:
    AABBox             m_AABB;       ///< Axis aligned bounding box enclosing everything within the Aggregate.
    VTable *           m_vTable;     ///< Table of virtual functions specific to the derived class.
                                     ///  Implemented like this to support memory dumping of arena objects.

    uint32_t           m_numTagBits; ///< Number of bits required to hold the number of child volumes.
    uint32_t           m_numVolumes; ///< Number of child volumes. These may be stored in procedural format.
    uint32_t           m_flags;      ///< Aggregate flags

// Hard-coded padding to ensure 16 byte alignment for Unix64 builds
#if (EA_PLATFORM_PTR_SIZE == 8)
    uint32_t pad[3];
#endif
};

} // namespace collision
} // namespace rw


// Version 2 - Added aggregate flags
EA_SERIALIZATION_CLASS_VERSION(rw::collision::Aggregate, 2)

#endif // PUBLIC_RW_COLLISION_AGGREGATE_H
