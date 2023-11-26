// (c) Electronic Arts. All Rights Reserved.
#ifndef CLUSTEREDMESH_TEST_HELPERS_HPP
#define CLUSTEREDMESH_TEST_HELPERS_HPP

#include "EABase/eabase.h"
#include "EAAssert/eaassert.h"
#include "rw/collision/volume.h"

// TODO: this function currently leaks the memory allocated for the clustered mesh and
// volume and needs fixing
rw::collision::Volume *LoadSerializedClusteredMesh(const char * filename);

#endif // !defined(CLUSTEREDMESH_TEST_HELPERS_HPP)
