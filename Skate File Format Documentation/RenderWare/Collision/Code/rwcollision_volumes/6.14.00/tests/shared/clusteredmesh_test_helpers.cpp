// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/clusteredmesh.h>

#include <serialization/serialization.h>
#include <serialization/binary_stream_oarchive.h>
#include <serialization/binary_stream_iarchive.h>
#include <eaphysics/hlserializable.h>

#include <eaphysics/unitframework/allocator.h> // For GetFilesysAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "clusteredmesh_test_helpers.hpp"
#include "SimpleStream.hpp"

using namespace rwpmath;
using namespace rw::collision;

//-----------------------------------------------------------------------------------------------------
//  This loads a serialized clustered mesh from a file

Volume *LoadSerializedClusteredMesh(const char * filename)
{
    ClusteredMesh * clusteredMesh = 0;

    SimpleStream strm(filename);
    if (strm.fail()) return 0;

    EA::Serialization::basic_binary_stream_iarchive<SimpleStream, 
        EA::Serialization::Endian::LittleEndianConverter> iArchive(strm);

    iArchive & EAPHYSICS_HL_SERIALIZABLE_WITH_ALLOCATOR(ClusteredMesh, clusteredMesh, *EA::Allocator::ICoreAllocator::GetDefaultAllocator());

    EA_ASSERT(clusteredMesh->IsValid());

    return static_cast<Volume*>(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(clusteredMesh));
}
