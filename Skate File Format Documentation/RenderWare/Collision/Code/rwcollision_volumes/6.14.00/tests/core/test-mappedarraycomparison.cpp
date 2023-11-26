// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/libcore.h>
#include <rw/collision/kdtreebuilder.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator
#include <eaphysics/unitframework/serialization_test_helpers.hpp>

#include "testsuitebase.h" // For TestSuiteBase

#define VOLUMESPERAXIS 10u
#define NUMVOLUMES VOLUMESPERAXIS*VOLUMESPERAXIS*VOLUMESPERAXIS

using namespace rwpmath;
using namespace rw::collision;



class TestMappedArrayComparison: public tests::TestSuiteBase
{
public:

    uint8_t* m_arenaTypeBuffer;

    virtual void Initialize()
    {
        SuiteName("TestMappedArrayComparison");

        EATEST_REGISTER("TestBBoxQueryConsistency", "Test BBoxQuery Consistency between KDTree and Simple MappedArrays",
            TestMappedArrayComparison, TestBBoxQueryConsistency);
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();
    }

    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        tests::TestSuiteBase::TeardownSuite();
    }

    struct BoxInfo{
        Vector3         Dimensions;
        Matrix44Affine  Transform;
        AABBoxU         BoundingBox;
    };

    static BoxInfo* CreateBoxInfoArray()
    {
        //Create an array of volumes representing 10x10x10 cube of cubes
        //Boxes of of dimensions index/10 units - with 2 units seperating distance along each axis        
        BoxInfo *boxArray = (BoxInfo*)EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sizeof(BoxInfo)*NUMVOLUMES,0 ,0 , 16);
        for(uint32_t x = 0; x<VOLUMESPERAXIS; x++)
            for(uint32_t y = 0; y<VOLUMESPERAXIS; y++)
                for(uint32_t z = 0; z<VOLUMESPERAXIS; z++)
                {
                    uint32_t index=x*VOLUMESPERAXIS*VOLUMESPERAXIS+y*VOLUMESPERAXIS+z;
                    boxArray[index].Dimensions=Vector3(1.0f,1.0f,1.0f);//+Vector3((float)x,(float)y,(float)z))/VecFloat(10.0f);
                    Vector3 offset=Vector3((float)x,(float)y,(float)z)*VecFloat(3.0f);
                    boxArray[index].Transform= GetMatrix44Affine_Identity();
                    boxArray[index].Transform.SetW(offset);
                    Vector3 min= offset-boxArray[index].Dimensions;
                    Vector3 max= offset+boxArray[index].Dimensions;
                    boxArray[index].BoundingBox.m_min= rw::math::fpu::Vector3U_32(min.GetX(),min.GetY(),min.GetZ());
                    boxArray[index].BoundingBox.m_max= rw::math::fpu::Vector3U_32(max.GetX(),max.GetY(),max.GetZ());
                }
        return boxArray;
    }

    static SimpleMappedArray * CreateSimpleMappedArray(BoxInfo *boxArray)
    {
        SimpleMappedArray * mappedArray = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(NUMVOLUMES);
        Volume *m_VolumeArray = mappedArray->GetVolumeArray();
        for (uint32_t vol = 0; vol <NUMVOLUMES; ++vol)
        {
            BoxVolume::Initialize((void*)&m_VolumeArray[vol]);
            BoxVolume &currentBox = *(BoxVolume*)&m_VolumeArray[vol];
            currentBox.SetEnabled(true);
            currentBox.SetDimensions(boxArray[vol].Dimensions);
            currentBox.SetRadius(0.0f);
            ((BoxVolume*)&m_VolumeArray[vol])->SetLocalTransform(boxArray[vol].Transform);
        }
        mappedArray->UpdateThis();
        return mappedArray;
    }

    static KDTreeMappedArray * CreateKDTreeMappedArray(BoxInfo *boxArray)
    {
        uint32_t splitThreshold = 7;

        //Create array of Bboxes
        AABBoxU  bboxList[NUMVOLUMES];
        rw::collision::AABBox tot;
        tot.m_max=Vector3(-MAX_FLOAT,-MAX_FLOAT,-MAX_FLOAT);
        tot.m_min=Vector3(MAX_FLOAT,MAX_FLOAT,MAX_FLOAT);
        for(uint32_t i=0;i<NUMVOLUMES;i++)
        {            
            AABBoxU &current=bboxList[i]; 
            current=boxArray[i].BoundingBox;
            tot.Union(Vector3(current.m_min.GetX(),current.m_min.GetY(),current.m_min.GetZ()));
            tot.Union(Vector3(current.m_max.GetX(),current.m_max.GetY(),current.m_max.GetZ()));
        }

        rw::collision::KDTreeBuilder kdTreeBuilder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
        kdTreeBuilder.BuildTree(NUMVOLUMES, bboxList, splitThreshold);

        // Allocate a block of memory for our KDTreeMappedArray
        uint32_t numBranchNodes = kdTreeBuilder.GetNumBranchNodes();
        KDTreeMappedArray * kdtreeMappedArray = EA::Physics::UnitFramework::Creator<KDTreeMappedArray>().New(NUMVOLUMES, numBranchNodes, tot);

        // Create the child volumes using order of objects defined by kd tree
        const uint32_t * entryIndices = kdTreeBuilder.GetSortedEntryIndices();
        Volume *m_kdtmaVolumeArray = kdtreeMappedArray->GetVolumeArray();
        for (uint32_t vol = 0; vol < NUMVOLUMES; ++vol)
        {
            BoxVolume::Initialize((void*)&m_kdtmaVolumeArray[vol]);
            BoxVolume &currentBox = *(BoxVolume*)&m_kdtmaVolumeArray[vol];
            currentBox.SetEnabled(true);
            currentBox.SetDimensions(boxArray[entryIndices[vol]].Dimensions);
            currentBox.SetRadius(0.0f);
            currentBox.SetLocalTransform(boxArray[entryIndices[vol]].Transform);
        }

        // Initialize the kdtree
        kdTreeBuilder.InitializeRuntimeKDTree(kdtreeMappedArray->GetKDTreeMap());
        kdtreeMappedArray->Update();
        return kdtreeMappedArray;
    }

    static bool CompareBoxVolumes(const BoxVolume& original, const BoxVolume& copied)
    {
        Vector3 inDimensions;
        Vector3 outDimensions;
        original.GetDimensions(inDimensions);
        copied.GetDimensions(outDimensions);
        return (inDimensions == outDimensions);
    }

private:
    void TestBBoxQueryConsistency();

} TestMappedArrayComparisonSingleton;


void TestMappedArrayComparison::TestBBoxQueryConsistency()
{
    //Create MappedArrays
    BoxInfo *boxArray= CreateBoxInfoArray();
    KDTreeMappedArray * kdtreeMappedArray = CreateKDTreeMappedArray(boxArray);
    SimpleMappedArray * simpleMappedArray = CreateSimpleMappedArray(boxArray);

    EATESTAssert(kdtreeMappedArray->GetVolumeCount()==NUMVOLUMES, "KDTreeMappedArray contains an incorrect number of Volumes");
    EATESTAssert(simpleMappedArray->GetVolumeCount()==NUMVOLUMES, "SimpleMappedArray contains an incorrect number of Volumes");
    rw::collision::AABBox simpleBBox = simpleMappedArray->GetBBox();
    rw::collision::AABBox kdtreeBBox = kdtreeMappedArray->GetBBox();

    EATESTAssert(IsSimilar(simpleBBox.m_min,kdtreeBBox.m_min), "KDTreeMappedArray and Simple Mapped Array have differing BBoxes");
    EATESTAssert(IsSimilar(simpleBBox.m_max,kdtreeBBox.m_max), "KDTreeMappedArray and Simple Mapped Array have differing BBoxes");

    VolumeBBoxQuery * bboxQuery = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(2u,NUMVOLUMES);

    //Create AggregateVolumes from mapped arrays 
    Volume *kdtmaVolume=EA::Physics::UnitFramework::Creator<AggregateVolume>().New((Aggregate*)kdtreeMappedArray);
    kdtmaVolume->SetEnabled(true);
    Volume *simpleVolume=EA::Physics::UnitFramework::Creator<AggregateVolume>().New((Aggregate*)simpleMappedArray);
    simpleVolume->SetEnabled(true);

    //loop through 1000 Bboxes checking each one returns one and only one result for each aggregate.
    rw::collision::AABBox queryBBox;
    uint32_t numOverlaps;
    BoxVolume *boxVolume;
    Vector3 dimensions;
    for(uint32_t i = 0;i<NUMVOLUMES;i++)
    {
        queryBBox.m_min=Vector3(boxArray[i].BoundingBox.m_min.GetX(),boxArray[i].BoundingBox.m_min.GetY(),boxArray[i].BoundingBox.m_min.GetZ());
        queryBBox.m_max=Vector3(boxArray[i].BoundingBox.m_max.GetX(),boxArray[i].BoundingBox.m_max.GetY(),boxArray[i].BoundingBox.m_max.GetZ());
        
        bboxQuery->InitQuery((const Volume**)&kdtmaVolume,NULL,1,queryBBox);
        numOverlaps = bboxQuery->GetOverlaps();
        EATESTAssert(numOverlaps==1,"KDTreeMappedArray BBoxQuery returned incorrect number of volumes");
        boxVolume = (BoxVolume*)bboxQuery->GetOverlapResultsBuffer()->volume;
        boxVolume->GetDimensions(dimensions);
        EATESTAssert(IsSimilar(dimensions,boxArray[i].Dimensions),"KDTreeMappedArray returned incorrect volume");
        
        bboxQuery->InitQuery((const Volume**)&simpleVolume,NULL,1,queryBBox);
        numOverlaps = bboxQuery->GetOverlaps();
        EATESTAssert(numOverlaps==1,"SimpleMappedArray BBoxQuery returned incorrect number of volumes");
        boxVolume = (BoxVolume*)bboxQuery->GetOverlapResultsBuffer()->volume;
        boxVolume->GetDimensions(dimensions);
        EATESTAssert(IsSimilar(dimensions,boxArray[i].Dimensions),"SimpleMappedArray returned incorrect volume");
    }

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(boxArray);
}

//ToDo: LineTestConsistency
//ToDo: Aggregate of aggregates 


