// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcaggregatevolume.cpp

 Purpose: A short description of the file.

 */

// ***********************************************************************************************************
// Includes

#include <new>

#include "rw/collision/aggregate.h"
#include "rw/collision/aggregatevolume.h"

/* Use platform specific maths */
using namespace rwpmath;

namespace rw
{
namespace collision
{


// ***********************************************************************************************************
// Typedefs


// ***********************************************************************************************************
// Defines + Enums + Consts


// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions


// ***********************************************************************************************************
// Structs + Unions + Classes


// ***********************************************************************************************************
// Static Functions


/**
Initializes an AggregateVolume at the given memory location.
\param resource Memory resource
\param agg Aggregate of the volume.
\return a pointer to the new aggregate volume
*/
AggregateVolume *
AggregateVolume::Initialize(const EA::Physics::MemoryPtr& resource, Aggregate *agg)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) AggregateVolume(agg);
}


/**
Initializes an AggregateVolume at the given memory location. Used by serialization
where the aggregate pointer is set on load.
\param resource Memory resource
\param agg Aggregate of the volume.
\return a pointer to the new aggregate volume
\internal
*/
AggregateVolume *
AggregateVolume::Initialize(const EA::Physics::MemoryPtr& resource)
{
    rwcASSERTALIGN(resource.GetMemory(), rwcVOLUMEALIGNMENT);
    return new (resource.GetMemory()) AggregateVolume;
}


/**
Gets the bounding box of the aggregate volume.


Most aggregate objects cache the bounding box of the whole aggregate.  The bounding box of the aggregate is
transformed by the volume relative transform and by the input parent transform if it is not NULL.
The transformations may translate and/or rotate the bbox which may increase the size of the bbox.
\param tm the parent transformation to apply to the result bbox.  May be NULL.
\param tight if true, a slower more precise algorithm may be used to compute the bbox.  However, for the
aggregate volume type this parameter has no effect.
\param bBox reference to a bbox to store the result.
\return TRUE on success
*/
RwpBool
AggregateVolume::GetBBox(const rwpmath::Matrix44Affine *tm,
                         RwpBool /*tight*/, AABBox &bBox) const
{
    const Aggregate *agg = GetAggregate();

    AABBox bb = agg->GetBBox();
    if (tm)
    {
        Matrix44Affine mtx = Mult(transform, *tm);
        bBox = bb.Transform(&mtx);
    }
    else
    {
        bBox = bb.Transform(&transform);
    }


    return  TRUE ;
}

rwpmath::Vector3
AggregateVolume::GetBBoxDiag() const
{
    const Aggregate *agg = GetAggregate();

    AABBox bb = agg->GetBBox();
    bb = bb.Transform(&transform);

    return  bb.Max() - bb.Min() ;
}


/*
\toedit
\classinternal
Create the generalized primitive instance data.
\description 
Since the aggregate volume type is not enabled for GP, this function is not supported and it \b always
returns false.
\param instance output  generalized primitive instance data.
\param tm parent transformation to applied to the volume.  May be NULL.
\return TRUE on success.
*/
RwpBool
AggregateVolume::CreateGPInstance(GPInstance & /*instance*/,
                                  const Matrix44Affine * /*tm*/) const
{
    return  FALSE ;
}


/**
Calls Aggregate method for clearing all processed flags
*/
void AggregateVolume::ClearAllProcessedFlags()
{
    // As we are overriding base function we need to ensure that we clear this volumes processed flag
    ClearProcessedFlag();

    Aggregate *agg = GetAggregate();
    if (agg)
    {
        agg->ClearAllProcessedFlags();
    }
}


/**
\brief Applies a uniform scale factor to the dimensions of the aggregate volume

This function is used by the Volume class ApplyUniformScale function. If
useProcessedFlags is enabled then the volume processed flag will be respected
and scaling performed if flag is not set. The volume processed flag will be set
after the scaling operation.

\param scale uniform scale value to apply to volume
\param useProcessedFlags default false ignores volume processed flag
*/
void
AggregateVolume::ApplyUniformScale(float scale, bool useProcessedFlags)
{
    EA_ASSERT(scale > 0.0f);

    // Apply scale if processing is required OR if we are to ignore flags
    if (!useProcessedFlags || !(m_flags & VOLUMEFLAG_ISPROCESSED))
    {
        transform.Pos() *= scale;
        Aggregate *agg = GetAggregate();
        if (agg)
        {
            agg->ApplyUniformScale(scale, useProcessedFlags);
        }
    }

    // Set processed flag to identify that scaling has been performed on this volume
    if (useProcessedFlags)
    {
        SetProcessedFlag();
    }
}


// ***********************************************************************************************************
// External Functions

/**
This is the virtual function table that is shared by all aggregate volumes.
*/
Volume::VTable
globalAggregateVolumeVTable = {
    rw::collision::VOLUMETYPEAGGREGATE,
    (Volume::GetBBoxFn)(&AggregateVolume::GetBBox),
    (Volume::GetBBoxDiagFn)(&AggregateVolume::GetBBoxDiag),
    0,      // formerly GetInterval
    0,      // formerly GetMaximumFeature
    (Volume::CreateGPInstanceFn)(&AggregateVolume::CreateGPInstance),
    NULL,
    (Volume::ReleaseFn)(&AggregateVolume::Release),
    "AggregateVolume",
    0,
    0,
    0,
    (Volume::ClearAllProcessedFlagsFn)(&AggregateVolume::ClearAllProcessedFlags),
    (Volume::ApplyUniformScaleFn)(&AggregateVolume::ApplyUniformScale)
};

} // namespace collision
} // namespace rw
