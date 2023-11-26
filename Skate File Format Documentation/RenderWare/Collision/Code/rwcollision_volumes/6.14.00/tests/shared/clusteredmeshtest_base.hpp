// (c) Electronic Arts. All Rights Reserved.
#ifndef CLUSTEREDMESHTEST_BASE_HPP
#define CLUSTEREDMESHTEST_BASE_HPP

#include <unit/unit.h>
#include "EABase/eabase.h"
#include "EAAssert/eaassert.h"
#include "rw/collision/volume.h"
#include "rw/collision/triangle.h"

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>
#include "testsuitebase.h" // For TestSuiteBase

#if !defined(EA_PLATFORM_PS3_SPU)
#include <rw/collision/clusteredmeshofflinebuilder.h>
#endif // !defined(EA_PLATFORM_PS3_SPU)

#include "clusteredmesh_test_helpers.hpp"

class ClusteredMeshTest_Base: public rw::collision::tests::TestSuiteBase
{
protected:
    virtual void SetupSuite()
    {
        rw::collision::tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        rw::collision::Volume::InitializeVTable();

#if !defined(EA_PLATFORM_PS3_SPU)
        // Build a quad and triangle mesh
        quadMesh = BuildClusteredMesh(4u,4u,true);
        triangleMesh = BuildClusteredMesh(4u, 4u, false);
#endif //!defined(EA_PLATFORM_PS3_SPU)
    }

    virtual void TeardownSuite()
    {
#if !defined(EA_PLATFORM_PS3_SPU)
        if (NULL != quadMesh)
        {
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(quadMesh);
        }

        if (NULL != triangleMesh)
        {
            EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(triangleMesh);
        }

        EA::Physics::UnitFramework::CheckAllocator();
        EA::Physics::UnitFramework::ResetAllocator();
#endif // !defined(EA_PLATFORM_PS3_SPU)
        rw::collision::tests::TestSuiteBase::TeardownSuite();
    }
    inline void AssertTrianglesTheSame(const rw::collision::TriangleVolume * volumeA, const rw::collision::TriangleVolume * volumeB)
    {
        //Check volume A and volume B values are the same within floating point accuracy.
        EATESTAssert(rwpmath::IsSimilar(volumeA->GetLocalTransform(), volumeB->GetLocalTransform()), "Volume transform does not match");
        EATESTAssert(rwpmath::IsSimilar(volumeA->GetEdgeCosVector(), volumeB->GetEdgeCosVector()), "Edge cosine data does not match");
        EATESTAssert(rwpmath::IsSimilar(volumeA->GetRadius(), volumeB->GetRadius()), "Volume radius does not match");
        EATESTAssert(volumeA->GetGroup()   == volumeB->GetGroup(), "Volume group does not match");
        EATESTAssert(volumeA->GetSurface() == volumeB->GetSurface(), "Volume surface id does not match");
        EATESTAssert(volumeA->GetFlags()   == volumeB->GetFlags(), "Volume flags do not match");
    }

    bool AreTrianglesTheSame(const rw::collision::TriangleVolume * volumeA, const rw::collision::TriangleVolume * volumeB);

    void LineQueryTester(const rw::collision::Volume *meshVolume, 
                         const rw::collision::Volume *scaledMeshVolume, 
                         const rwpmath::Matrix44Affine* transformMatrix,
                         rw::collision::VolumeBBoxQuery* bboxQuery, 
                         rw::collision::VolumeLineQuery* triangleLineQuery, 
                         rw::collision::VolumeLineQuery* scaledMeshLineQuery, 
                         float scale = 1.0f,
                         float accuracy = 1.0e-5f);

    void RestartingLineQueryTester(const rw::collision::Volume* meshVolume, uint32_t fullBufferSize, uint32_t stackSize, uint32_t smallBufferSize);

    void RestartingBBoxQueryTester(const rw::collision::Volume* meshVolume, const rwpmath::Matrix44Affine* transformMatrix, uint32_t fullBufferSize, uint32_t stackSize, uint32_t smallBufferSize);

    void BBoxQueryInMappedArrayWithPrimitivesTester(const rw::collision::Volume* meshVolume);

#if !defined(EA_PLATFORM_PS3_SPU)
    rw::collision::ClusteredMesh * BuildClusteredMesh(const uint32_t triangleXCount,
        const uint32_t triangleZCount,
        const bool quads);

    void AddInputToBuilder(rw::collision::ClusteredMeshOfflineBuilder & offlineBuilder,
        const uint32_t triangleXCount,
        const uint32_t triangleZCount);
#endif // !defined(EA_PLATFORM_PS3_SPU)

    rw::collision::ClusteredMesh * triangleMesh;
    rw::collision::ClusteredMesh * quadMesh;
};
#endif // !defined(CLUSTEREDMESHTEST_BASE_HPP)
