// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: test-triangleunitwithedgecosines.cpp

Purpose: instantiate unit tests for TriangleUnitWithEdgeCosines class.

*/

// Intentionally include code under test first to check standalone header.
#include <rw/collision/triangleunit.h>   // for the TriangleUnitWithEdgeCosines<> we're testing

#include "test-clusterunit.h"           // for TestClusterUnit<> base class

#include <EABase/eabase.h>


// ***********************************************************************************************************

namespace EA 
{
    namespace Collision
    {
        namespace Tests
        {
            // We'll test the TriangleUnitWithEdgeCosines with dynamic compression.
            // Specific compression cases effectively get tested through this.
            typedef rw::collision::TriangleUnitWithEdgeCosines<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC> TestUnit;

            // We could add additional tests to this test suite if we wanted.
            class TestTriangleUnitWithEdgeCosines : public TestClusterUnit<TestUnit> 
            {
            public:

                static const bool CAN_USE_QUADS = false;
                static const bool ASSUMES_EDGECOSINES = true;
                static const bool HANDLES_IDS = true;

                TestTriangleUnitWithEdgeCosines() : 
                  TestClusterUnit<TestUnit>("TestTriangleUnitWithEdgeCosines", "test-triangleunitwithedgecosines.elf", 
                      CAN_USE_QUADS, ASSUMES_EDGECOSINES, HANDLES_IDS)
                {
                }

                  template <uint8_t GROUP_ID_BYTES, uint8_t SURFACE_ID_BYTES>
                  void CheckGetTriSizeWithID()
                  {
                      CreateTriUnitWithEdgeCosinesAndIDs(GROUP_ID_BYTES,SURFACE_ID_BYTES);
                      rw::collision::TriangleUnitWithEdgeCosinesAndIDs<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC,
                          GROUP_ID_BYTES, SURFACE_ID_BYTES> unit(*mCluster, mClusterParams);
                      EATESTAssert(unit.IsValid(), "Should be valid");
                      EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                      EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                      EATESTAssert(unit.GetSize() == (uint32_t) (7+SURFACE_ID_BYTES+GROUP_ID_BYTES), 
                          "Triangle unit with IDs should be 7 or more bytes");
                  }

                  virtual void CheckGetTriSize()
                  {
                      {
                          CreateTriUnitWithEdgeCosines();
                          TestUnit unit(*mCluster, mClusterParams);
                          EATESTAssert(unit.IsValid(), "Should be valid");
                          EATESTAssert(unit.GetVertexCount() == 3u, "Should hold three vertices");
                          EATESTAssert(unit.GetTriCount() == 1u, "Should hold one triangle");
                          EATESTAssert(unit.GetSize() == 7u, "Triangle unit with edge data should be 7 bytes");
                      }
                      CheckGetTriSizeWithID<0,0>();
                      CheckGetTriSizeWithID<0,1>();
                      CheckGetTriSizeWithID<0,2>();
                      CheckGetTriSizeWithID<1,0>();
                      CheckGetTriSizeWithID<1,1>();
                      CheckGetTriSizeWithID<1,2>();
                      CheckGetTriSizeWithID<2,0>();
                      CheckGetTriSizeWithID<2,1>();
                      CheckGetTriSizeWithID<2,2>();
                  }

                  template <uint8_t GROUP_ID_BYTES, uint8_t SURFACE_ID_BYTES>
                  void CheckGetIDsFromUnitGS(uint32_t numVertices, bool includeEdgeCosines = false)
                  {
                      uint32_t masks[3] = { 0x0, 0xff, 0xffff };
                      uint32_t mask = masks[GROUP_ID_BYTES] | (masks[SURFACE_ID_BYTES])<<16;

                      InitializeCluster(
                          rw::collision::ClusteredMeshCluster::VERTICES_UNCOMPRESSED,
                          GROUP_ID_BYTES,
                          SURFACE_ID_BYTES);

                      {
                          uint16_t groupID = 0x1234;
                          uint16_t surfaceID = 0xfdeb;
                          // Tri unit, no edge cosines
                          WriteUnit(mCluster->UnitData(), mClusterParams, numVertices, 4,1,3,9,
                              includeEdgeCosines, 0,0,0,0, groupID, surfaceID);
                          rw::collision::TriangleUnitWithEdgeCosinesAndIDs<rw::collision::ClusteredMeshCluster::COMPRESSION_DYNAMIC,
                              GROUP_ID_BYTES, SURFACE_ID_BYTES> unit(*mCluster, mClusterParams);

                          uint32_t id;
                          id = unit.GetID();
                          EATESTAssert((0xfdeb1234 & mask) == id, "Should combine 2 IDs");

                          id = unit.GetGroupID();
                          EATESTAssert((groupID & masks[GROUP_ID_BYTES]) == id, "Invalid group ID");

                          id = unit.GetSurfaceID();
                          EATESTAssert((surfaceID & masks[SURFACE_ID_BYTES]) == id, "Invalid surface ID");

                          // Double check its the same as GPTriangles use
                          rw::collision::GPTriangle tris[2];
                          rwpmath::Matrix44Affine identity = rwpmath::GetMatrix44Affine_Identity();
                          rw::collision::AABBox bbox(rwpmath::Vector3(-1000.0f,-1000.0f,-1000.0f), rwpmath::Vector3(1000.0f,1000.0f,1000.0f));
                          uint32_t numTris = 0;
                          mCluster->UnitGetOverlappingGPInstances(0, bbox, &identity, tris, numTris, mClusterParams);
                          EATESTAssert(numTris == numVertices-2, "Should get all tris");
                          EATESTAssert(tris[0].mUserTag == (0xfdeb1234 & mask), "Should be same as GP");
                      }
                  }

                  template<uint8_t GROUP_ID_BYTES>
                  void CheckGetIDsFromUnitG(uint8_t surfaceIdBytes, uint32_t numVertices, bool includeEdgeCosines = false)
                  {
                      switch (surfaceIdBytes)
                      {
                      case 0: 
                          CheckGetIDsFromUnitGS<GROUP_ID_BYTES, 0>(numVertices, includeEdgeCosines);
                          break;
                      case 1:
                          CheckGetIDsFromUnitGS<GROUP_ID_BYTES, 1>(numVertices, includeEdgeCosines);
                          break;
                      case 2:
                          CheckGetIDsFromUnitGS<GROUP_ID_BYTES, 2>(numVertices, includeEdgeCosines);
                          break;
                      }
                  }

                  virtual void CheckGetIDsFromUnit(uint8_t groupIdBytes, uint8_t surfaceIdBytes, uint32_t numVertices, bool includeEdgeCosines = false)
                  {
                      switch (groupIdBytes)
                      {
                      case 0: 
                          CheckGetIDsFromUnitG<0>(surfaceIdBytes, numVertices, includeEdgeCosines);
                          break;
                      case 1:
                          CheckGetIDsFromUnitG<1>(surfaceIdBytes, numVertices, includeEdgeCosines);
                          break;
                      case 2:
                          CheckGetIDsFromUnitG<2>(surfaceIdBytes, numVertices, includeEdgeCosines);
                          break;
                      }
                  }


            } gTriangleUnitWithEdgeCosinesTest;
        }
    }
}

// ***********************************************************************************************************

