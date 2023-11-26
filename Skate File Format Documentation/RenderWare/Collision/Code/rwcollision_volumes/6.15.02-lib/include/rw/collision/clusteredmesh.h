// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CLUSTEREDMESH_H
#define PUBLIC_RW_COLLISION_CLUSTEREDMESH_H

/*************************************************************************************************************/



#include "rw/collision/clusteredmeshcluster.h"

#include "rw/collision/common.h"

#if !defined(EA_PLATFORM_PS3_SPU)
#include "rw/collision/clusteredmeshunit.h"
#include "rw/collision/clusteredmeshbase.h"
#endif

#include "rw/collision/clusteredmeshcluster_methods.h"

#if !defined(EA_PLATFORM_PS3_SPU)
#include "rw/collision/clusteredmeshunit_methods.h"
#include "rw/collision/clusteredmeshbase_methods.h"
#endif

#endif //PUBLIC_RW_COLLISION_CLUSTEREDMESH_H
