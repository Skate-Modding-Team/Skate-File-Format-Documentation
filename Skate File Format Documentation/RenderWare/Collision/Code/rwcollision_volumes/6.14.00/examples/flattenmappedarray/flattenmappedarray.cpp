// (c) Electronic Arts. All Rights Reserved.

#include <EABase/eabase.h>
#include <rw/collision/common.h>


#if !defined EA_PLATFORM_PS3_SPU

#include <stdio.h>

#include <rw/collision/mappedarray.h>
#include <rw/collision/aggregatevolume.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/triangle.h>
#include <rw/collision/box.h>
#include <rw/collision/sphere.h>
#include <rw/collision/meshbuilder/detail/linearallocator.h>
#include <rw/collision/initialize.h>

#include "EAMain/EAEntryPointMain.inl" // For EAMain

/// Determine if the mapped array contains a child mapped array
/// @param mappedArray
/// @param returns true if any child of the MappedArray is another MappedArray (so mappedArray needs flattening), false otherwise
static bool ContainsMappedArray(const rw::collision::MappedArray & mappedArray)
{
    //loop over children
    for (uint16_t i = 0; i < static_cast<uint16_t>(mappedArray.GetVolumeCount()); ++i)
    {
        const rw::collision::Volume * child = mappedArray.GetVolume(i);
        const rw::collision::VolumeType volumeType = child->GetType();
        if (volumeType == rw::collision::VOLUMETYPEAGGREGATE)
        {
            const rw::collision::AggregateVolume * aggregateVolume = static_cast<const rw::collision::AggregateVolume *>(child);
            const rw::collision::Aggregate * aggregate = aggregateVolume->GetAggregate();
            EA_ASSERT(aggregate);

            if (!aggregate->IsProcedural())
            {
                //at least one child is a mapped array
                return true;
            }
        }
    }
    //none of the children are mapped arrays
    return false;
}

/// Recursively count number of primitive volumes in a mapped array and it's child mapped arrays.
/// @param mappedArray
/// @param numPrimitives Incremented by number of primitives found. Ignores any procedural children found.
static void CountPrimitives(const rw::collision::MappedArray & mappedArray, uint32_t & numPrimitives)
{    
    for (uint16_t i = 0; i < static_cast<uint16_t>(mappedArray.GetVolumeCount()); ++i)
    {
        const rw::collision::Volume * child = mappedArray.GetVolume(i);
        const rw::collision::VolumeType volumeType = child->GetType();
        if (volumeType == rw::collision::VOLUMETYPEAGGREGATE)
        {
            // Recursively count aggregate's children
            const rw::collision::AggregateVolume * aggregateVolume = static_cast<const rw::collision::AggregateVolume *>(child);
            const rw::collision::Aggregate * aggregate = aggregateVolume->GetAggregate();
            EA_ASSERT(aggregate);

            if (!aggregate->IsProcedural())
            {
                const rw::collision::MappedArray * mappedArrayChild = static_cast<const rw::collision::MappedArray *>(aggregate);
                CountPrimitives(*mappedArrayChild, numPrimitives);                 
            }
        }
        else
        {
            // Count 1 primitive
            ++numPrimitives;
        }
    }
}

/// Recursively copy primitive volumes in a mapped array into a new mapped array and it's child mapped arrays.
/// @param flattenedArray Simple mapped array to add volumes to.
/// @param nextChild Next child in flattenedArray to write into.
/// @param mappedArray Mapped array to read volumes from.
/// @param childTransform Transform to apply to children of ma
/// @returns number of primitives found. Ignores any procedural children found.
static void CopyPrimitives(rw::collision::SimpleMappedArray & flattenedArray, uint16_t& nextChild, 
                           const rw::collision::MappedArray & mappedArray, rwpmath::Matrix44Affine::InParam childTransform)
{    
    for (uint16_t i = 0; i < static_cast<uint16_t>(mappedArray.GetVolumeCount()); ++i)
    {
        const rw::collision::Volume * child = mappedArray.GetVolume(i);
        const rw::collision::VolumeType volumeType = child->GetType();
        if (volumeType == rw::collision::VOLUMETYPEAGGREGATE)
        {
            // Recursively process aggregate
            const rw::collision::AggregateVolume * aggregateVolume = static_cast<const rw::collision::AggregateVolume *>(child);
            const rw::collision::Aggregate * aggregate = aggregateVolume->GetAggregate();
            EA_ASSERT(aggregate);
            if (!aggregate->IsProcedural())
            {
                // Apply the relative transform from this aggregate to all children
                rwpmath::Matrix44Affine transform = (*aggregateVolume->GetRelativeTransform()) * childTransform;
                const rw::collision::MappedArray * mappedArrayChild = static_cast<const rw::collision::MappedArray *>(aggregate);              
                CopyPrimitives(flattenedArray, nextChild, *mappedArrayChild, transform);
            }
        }
        else
        {
            // Copy primitive
            rw::collision::Volume * newChild = flattenedArray.GetVolume(static_cast<uint16_t>(nextChild));
            * newChild = * child;
            if (volumeType == rw::collision::VOLUMETYPETRIANGLE)
            {
                rw::collision::TriangleVolume * triangle = static_cast<rw::collision::TriangleVolume *>(newChild);
                rwpmath::Vector3 p1, p2, p3;
                triangle->GetPoints(p1, p2, p3);
                p1 = rwpmath::TransformPoint(p1, childTransform); 
                p2 = rwpmath::TransformPoint(p2, childTransform); 
                p3 = rwpmath::TransformPoint(p3, childTransform); 
                triangle->SetPoints(p1, p2, p3);
            }
            else
            {
                *newChild->GetRelativeTransform() = childTransform * (*child->GetRelativeTransform());
            }
            ++nextChild;
        }
    }
}

/// Flatten a hierarchy of primitive volumes in MappedArrays into a single MappedArray.
/// @param mappedArray MappedArray aggregate to flatten primitives from.
/// @param alloc Allocator to use to allocate the returned MappedArray.
/// @returns Pointer to MappedArray allocated using single allocation from alloc, 
/// or NULL, if it contains nothing that needs flattening.
static rw::collision::MappedArray * Flatten(const rw::collision::MappedArray & mappedArray, EA::Allocator::ICoreAllocator & alloc)
{
    //Do we need to flatten anything
    if (!ContainsMappedArray(mappedArray))
    {
        return NULL;    // doesn't need flattening
    }

    // Count number of primitives required in final mapped array
    uint32_t numPrimitives = 0;
    CountPrimitives(mappedArray, numPrimitives);

    // Allocate memory for new mapped array
    EA::Physics::SizeAndAlignment sa = rw::collision::SimpleMappedArray::GetResourceDescriptor(numPrimitives);
    void * mem = alloc.Alloc(sa.GetSize(), "Flattened SimpleMappedArray", 0, sa.GetAlignment());
    EA_ASSERT(mem);
    
    // Initialize new mapped array
    rw::collision::SimpleMappedArray * flattenedArray = rw::collision::SimpleMappedArray::Initialize(mem, numPrimitives);
    flattenedArray->SetFlags(mappedArray.GetFlags());
    
    // Copy primitives into new mapped array and update bounding box
    uint16_t nextChild = 0;
    CopyPrimitives(*flattenedArray, nextChild, mappedArray, rwpmath::GetMatrix44Affine_Identity());
    flattenedArray->UpdateThis();
    return flattenedArray;
}


int EAMain(int /*argc*/, char ** /*argv*/)
{
    // allocator
    EA::Allocator::ICoreAllocator* alloc = EA::Allocator::ICoreAllocator::GetDefaultAllocator();

    // set up the vtable system for the volumes
    rw::collision::InitializeVTables();

    // make a simple mapped array containing a box and sphere
    rw::collision::SimpleMappedArray* mappedArray;
    {
        // get memory requirements and initialize
        EA::Physics::SizeAndAlignment sa = rw::collision::SimpleMappedArray::GetResourceDescriptor(2u);
        void* mem = alloc->Alloc(sa.GetSize(), "MappedArray", 0, sa.GetAlignment());
        EA_ASSERT(mem);
        mappedArray = rw::collision::SimpleMappedArray::Initialize(mem, 2u);

        // Set the volumes in the mapped array. The first volume will be a box, the second a sphere
        rw::collision::Volume * volume0 = mappedArray->GetVolume(0);
        rw::collision::Volume * volume1 = mappedArray->GetVolume(1);
        
        // Initialize a box using the space reserved in mapped array for 1st volume
        rw::collision::BoxVolume::Initialize(volume0, 5.0f, 5.0f, 5.0f);
        // Initialize a sphere using the space reserved in mapped array for 2nd volume
        rw::collision::SphereVolume::Initialize(volume1, 10.0f);
    }

    // now make a new mapped array containing a cylinder and the other mapped array
    rw::collision::SimpleMappedArray* nestedMappedArray;
    {
        // get memory requirements and initialize
        EA::Physics::SizeAndAlignment sa = rw::collision::SimpleMappedArray::GetResourceDescriptor(2u);
        void* mem = alloc->Alloc(sa.GetSize(), "NestedMappedArray", 0, sa.GetAlignment());
        EA_ASSERT(mem);
        nestedMappedArray = rw::collision::SimpleMappedArray::Initialize(mem, 2u);
    
        // Set the volumes in the nested mapped array. The first will be a cylinder, the second a mapped array.
        rw::collision::Volume * volume0 = nestedMappedArray->GetVolume(0);
        rw::collision::Volume * volume1 = nestedMappedArray->GetVolume(1);
        
        // Initialize a cylinder using the space reserved in nested mapped array for 1st volume
        rw::collision::CylinderVolume::Initialize(volume0, 0.2f, 5.0f);
        // Initialize an aggregate using the space reserved in nested mapped array for 2nd volume
        rw::collision::AggregateVolume::Initialize(volume1, mappedArray);
    }

    // make an aggregate volume which contains the nested mapped array. This would be stored by the part
    rw::collision::Volume * partVolume;
    {
        // Get memory requirements and initialize. 
        EA::Physics::SizeAndAlignment sa = rw::collision::AggregateVolume::GetResourceDescriptor(nestedMappedArray);
        void* mem = alloc->Alloc(sa.GetSize(), "partVolume", 0, sa.GetAlignment());
        EA_ASSERT(mem);
        partVolume = rw::collision::AggregateVolume::Initialize(mem, nestedMappedArray);
    }

    
    rw::collision::MappedArray* flatMappedArray = NULL;
    if(partVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE)
    {
        rw::collision::AggregateVolume* aggVol = static_cast<rw::collision::AggregateVolume *>(partVolume);
        rw::collision::Aggregate * aggregate = aggVol->GetAggregate();
        if(aggregate->GetType() == rw::collision::RWCOBJECTTYPE_SIMPLEMAPPEDARRAY)
        {
            rw::collision::SimpleMappedArray* partMappedArray = static_cast<rw::collision::SimpleMappedArray*>(aggregate);
            flatMappedArray = Flatten(*partMappedArray, *alloc);
        }

            
    }

    (void)(flatMappedArray);

    EA_ASSERT(flatMappedArray != NULL);

    EA_ASSERT(flatMappedArray->GetVolume(0)->GetType() == rw::collision::VOLUMETYPECYLINDER);   
    EA_ASSERT(flatMappedArray->GetVolume(1)->GetType() == rw::collision::VOLUMETYPEBOX);
    EA_ASSERT(flatMappedArray->GetVolume(2)->GetType() == rw::collision::VOLUMETYPESPHERE);  

    // free data
    alloc->Free(flatMappedArray);
    alloc->Free(partVolume);
    alloc->Free(nestedMappedArray);
    alloc->Free(mappedArray);

    return 0;
}

#endif // #if !defined EA_PLATFORM_PS3_SPU