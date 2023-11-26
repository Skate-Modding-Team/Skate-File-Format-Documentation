// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <eaphysics/sizeandalignment.h>
#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

//*********************DEFINES*********************

#define CUBE_HALFLENGTH 0.5f
#define SPHERE_RADIUS 0.5f

//*********************NAMESPACES*********************
using namespace rw::collision;
using namespace rwpmath;

// ***********************************************************************************************************
// Test suite

class TestVolumeBBox : public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestVolumeBBox");
    
        // Test individual Methods
        EATEST_REGISTER("TestInitialize", "TestInitialize", TestVolumeBBox, TestInitialize);
        EATEST_REGISTER("TestInitQuery", "TestInitQuery", TestVolumeBBox, TestInitQuery);
        EATEST_REGISTER("TestAddPrimitiveRef", "TestAddPrimitiveRef", TestVolumeBBox, TestAddPrimitiveRef);
        EATEST_REGISTER("TestAddVolumeRef", "TestAddVolumeRef", TestVolumeBBox, TestAddVolumeRef);
        EATEST_REGISTER("GetResourceDescriptorNonZeroArgs", "GetResourceDescriptorNonZeroArgs", TestVolumeBBox, GetResourceDescriptorNonZeroArgs);
        EATEST_REGISTER("GetResourceDescriptorZeroArgs", "GetResourceDescriptorZeroArgs", TestVolumeBBox, GetResourceDescriptorZeroArgs);
        EATEST_REGISTER("TestFinished", "TestFinished", TestVolumeBBox, TestFinished);
        EATEST_REGISTER("TestGetOverlapResultsBufferCount", "TestGetOverlapResultsBufferCount", TestVolumeBBox, TestGetOverlapResultsBufferCount);
        EATEST_REGISTER("TestGetOverlapResultsBuffer", "TestGetOverlapResultsBuffer", TestVolumeBBox, TestGetOverlapResultsBuffer);

        // Test the GetOverlaps Method
        EATEST_REGISTER("GetOverlapsReturnValuePrimitive", "GetOverlapsReturnValuePrimitive", TestVolumeBBox, GetOverlapsReturnValuePrimitive);
        EATEST_REGISTER("GetOverlapsReturnValueAggregate", "GetOverlapsReturnValueAggregate", TestVolumeBBox, GetOverlapsReturnValueAggregate);
        EATEST_REGISTER("GetOverlapsZeroInputs", "GetOverlapsZeroInputs", TestVolumeBBox, GetOverlapsZeroInputs);
        EATEST_REGISTER("GetOverlapsSinglePrimInterResultsBuffer", "GetOverlapsSinglePrimInterResultsBuffer", TestVolumeBBox, GetOverlapsSinglePrimInterResultsBuffer);
        EATEST_REGISTER("GetOverlapsZeroPrimInterResultsBufferCounter", "GetOverlapsZeroPrimInterResultsBufferCounter(", TestVolumeBBox, GetOverlapsZeroPrimInterResultsBufferCounter);
        EATEST_REGISTER("GetOverlapsSinglePrimInterNoMatrix", "GetOverlapsSinglePrimInterNoMatrix", TestVolumeBBox, GetOverlapsSinglePrimInterNoMatrix);
        EATEST_REGISTER("GetOverlapsNullInputArrays", "GetOverlapsNullInputArrays", TestVolumeBBox, GetOverlapsNullInputArrays);
        EATEST_REGISTER("GetOverlapsCorrectPrimInterResults", "GetOverlapsCorrectPrimInterResults", TestVolumeBBox, GetOverlapsCorrectPrimInterResults);
        EATEST_REGISTER("GetOverlapsOverflowTwoPrimInter", "GetOverlapsOverflowTwoPrimInter", TestVolumeBBox, GetOverlapsOverflowTwoPrimInter);
        EATEST_REGISTER("GetOverlapsAggInterNoPrimInter", "GetOverlapsAggInterNoPrimInter", TestVolumeBBox, GetOverlapsAggInterNoPrimInter);
        EATEST_REGISTER("GetOverlapsSingleAggSinglePrimInterResultsBuffer", "GetOverlapsSingleAggSinglePrimInterResultsBuffer", TestVolumeBBox, GetOverlapsSingleAggSinglePrimInterResultsBuffer);
        EATEST_REGISTER("GetOverlapsPrimitivesOverflowTwoAggInter", "GetOverlapsPrimitivesOverflowTwoAggInter", TestVolumeBBox, GetOverlapsPrimitivesOverflowTwoAggInter);
        EATEST_REGISTER("GetOverlapsStackOverflowTwoAggInter", "GetOverlapsStackOverflowTwoAggInter", TestVolumeBBox, GetOverlapsStackOverflowTwoAggInter);
        EATEST_REGISTER("GetOverlapsZeroAggInterZeroPrimInter", "GetOverlapsZeroAggInterZeroPrimInter", TestVolumeBBox, GetOverlapsZeroAggInterZeroPrimInter);
        EATEST_REGISTER("GetOverlapsDisabledPrimInter", "GetOverlapsDisabledPrimInter", TestVolumeBBox, GetOverlapsDisabledPrimInter);
        EATEST_REGISTER("GetOverlapsDisabledAggInter", "GetOverlapsDisabledAggInter", TestVolumeBBox, GetOverlapsDisabledAggInter);
        EATEST_REGISTER("GetOverlapsPrimVolToAggStack", "GetOverlapsPrimVolToAggStack", TestVolumeBBox, GetOverlapsPrimVolToAggStack);
        EATEST_REGISTER("GetOverlapsPrimVolToAggStackFullBuffer", "GetOverlapsPrimVolToAggStackFullBuffer", TestVolumeBBox, GetOverlapsPrimVolToAggStackFullBuffer);
    }

    void SetupSuite()
    {
        tests::TestSuiteBase::SetupSuite();

        // Initialise the collision system
        Volume::InitializeVTable();

        // AABBox - unit cube centered at origin
        aabb = rw::collision::AABBox( -CUBE_HALFLENGTH, -CUBE_HALFLENGTH, -CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
        // but scale the box up a bit to ensure it contains the volumes we create (which tend to also be unit cubes)
        aabb.m_min *= 1.05f;
        aabb.m_max *= 1.05f;
    }

    void TeardownSuite()
    {
        EA::Physics::UnitFramework::ResetAllocator();
        Volume::ReleaseVTable();
        tests::TestSuiteBase::TeardownSuite();
    }

private:
    // Test individual Methods
    void TestInitialize();
    void TestInitQuery();
    void TestAddPrimitiveRef();
    void TestAddVolumeRef();
    void GetResourceDescriptorNonZeroArgs();
    void GetResourceDescriptorZeroArgs();
    void TestFinished();
    void TestGetOverlapResultsBufferCount();
    void TestGetOverlapResultsBuffer();

    // Test the GetOverlaps Method
    void GetOverlapsReturnValuePrimitive();
    void GetOverlapsReturnValueAggregate();
    void GetOverlapsZeroInputs();
    void GetOverlapsSinglePrimInterResultsBuffer();
    void GetOverlapsZeroPrimInterResultsBufferCounter();
    void GetOverlapsSinglePrimInterNoMatrix();
    void GetOverlapsNullInputArrays();
    void GetOverlapsCorrectPrimInterResults();
    void GetOverlapsOverflowTwoPrimInter();
    void GetOverlapsAggInterNoPrimInter();
    void GetOverlapsSingleAggSinglePrimInterResultsBuffer();
    void GetOverlapsSingleAggInterNoMatrix();
    void GetOverlapsPrimitivesOverflowTwoAggInter();
    void GetOverlapsStackOverflowTwoAggInter();
    void GetOverlapsZeroAggInterZeroPrimInter();
    void GetOverlapsDisabledPrimInter();
    void GetOverlapsDisabledAggInter();
    void GetOverlapsPrimVolToAggStack();
    void GetOverlapsPrimVolToAggStackFullBuffer();

    // Members
    rw::collision::AABBox aabb;
} TestVolumeBBoxSingleton;

// This function runs the following tests on the VolumeBBoxQuery::Initialize method
// 
// TEST CREATING AN OBJECT USING A VALID DESCRIPTOR
//

void TestVolumeBBox::TestInitialize()
{
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 2;

    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    EATESTAssert( p_VBBQ->m_stackMax == STACKSIZE,          "Testing Initialize sets m_stackMax correctly");
    EATESTAssert( p_VBBQ->m_primBufferSize == RESBUFFERSIZE,    "Testing Initialize sets m_primBufferSize correctly");
    EATESTAssert( p_VBBQ->m_instVolMax == RESBUFFERSIZE,        "Testing Initialize sets m_instVolMax correctly");
}



// This function runs the following tests on the VolumeBBoxQuery::InitQuery
//
// TEST VOLUME ARRAY MEMBER ASSIGNMENT USING NON-NULL ARRAYS
// TEST MATRIX ARRAY MEMBER ASSIGNMENT USING NULL ARRAYS
// TEST NUMINPUTS ASSIGNMENT USING NON-ZERO INTEGER
// TEST NUMINPUTS ASSIGNMENT USING ZERO INTEGER
// TEST VOLUME MEMBER ASSIGNMENT USING NULL ARRAYS
// TEST MATRIX MEMBER ASSIGNMENT USING NON-NULL ARRAYS
//
void TestVolumeBBox::TestInitQuery()
{
    const uint32_t NUMINTERSECTIONS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    const Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        0.0f, 0.0f, 0.0f
        );

    // Matrix array.
    const Matrix44Affine *matArray[ 1 ] = { &mat };

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume* volArray[ NUMINTERSECTIONS ];
    for( int i = 0 ; i < (int)NUMINTERSECTIONS ; i++ )
    {
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );
    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINTERSECTIONS , aabb );

    EATESTAssert( p_VBBQ->m_inputVols == (const Volume**)volArray,"Test InitQuery sets correct input volumes for non-zero array");
    EATESTAssert( p_VBBQ->m_inputMats == NULL,"Test InitQuery sets correct input matrices for NULL array");
    EATESTAssert( p_VBBQ->m_numInputs == NUMINTERSECTIONS,"Test InitQuery sets correct number of intersections for non-zero integer");

    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ2 = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );
    // Initialize query
    p_VBBQ2->InitQuery( NULL , matArray , 0 , aabb );

    EATESTAssert( p_VBBQ2->m_inputVols == NULL, "Test InitQuery sets correct input volumes for NULL arrays");
    EATESTAssert( p_VBBQ2->m_inputMats == matArray, "Test InitQuery sets correct input matrices for non-zero arrays");
    EATESTAssert( p_VBBQ2->m_numInputs == 0, "Test InitQuery sets correct number of intersections for zero integer");
}



// This function runs the following tests on the VolumeBBoxQuery::GetOverlapResultsBuffer
//
// TEST RETURN VALUE FOR VALID ARRAY ADDRESS
//
void TestVolumeBBox::TestGetOverlapResultsBuffer()
{
    const uint32_t NUMINTERSECTIONS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume* volArray[ NUMINTERSECTIONS ];
    for( int i = 0 ; i < (int)NUMINTERSECTIONS ; i++ )
    {
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }
    
    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );
    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[0] ) , NULL , NUMINTERSECTIONS , aabb );

    // Perform GetOverlaps()
    p_VBBQ->GetOverlaps();

    // Obtain the results array
    VolRef *testResultsArray = p_VBBQ->GetOverlapResultsBuffer();
    
    EATESTAssert( testResultsArray, "Test GetOverlapResultsBuffer returns a valid pointer");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlapResultsBufferCount
//
// TEST RETURN VALUE AGAINST ACTUAL VALUE
//
void TestVolumeBBox::TestGetOverlapResultsBufferCount()
{
    const uint32_t NUMINTERSECTIONS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume *volArray[ NUMINTERSECTIONS ];
    for( int i = 0 ; i < (int)NUMINTERSECTIONS ; i++ )
    {
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINTERSECTIONS , aabb );

    // Perform GetOverlaps()
    p_VBBQ->GetOverlaps();

    // Test the results buffer count 
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 1, "Testing the GetOverlapsResultsBufferCount function");
}

// This function runs the following tests on the VolumeBBoxQuery::Finished
//
// TEST FOR TRUE WHEN FINISHED
// TEST FOR FALSE WHEN NOT FINISHED
//
void TestVolumeBBox::TestFinished()
{
    const uint32_t NUMINTERSECTIONS = 2;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume *volArray[ NUMINTERSECTIONS ];
    for( int i = 0 ; i < (int)NUMINTERSECTIONS ; i++ )
    {
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINTERSECTIONS , aabb );

    // This is a fake test to complete code coverage. Because Finished() always returns true, this test 
    // is simply to test the other code path. It can be removed once this function is fixed
    p_VBBQ->m_currInput = 0;
    p_VBBQ->m_numInputs = 1;
    p_VBBQ->m_currVRef.volume = 0;
    p_VBBQ->m_stackNext = 0;
    EATESTAssert( p_VBBQ->Finished() == FALSE, "Test the contrived setup for code coverage");
        // This is where the contrived test ends

    // Run query and test return value and see if the function finishes
    EATESTAssert( p_VBBQ->GetOverlaps() == 1, "Test results buffer only big enough to hold 1 primitive" );
    
    // Comment this line back in once the code has been fixed!
    // EATESTAssert( p_VBBQ->Finished() == FALSE, "Test the function hasn't finished");

    // Run the query again, should process the second volume
    p_VBBQ->GetOverlaps();

    EATESTAssert( p_VBBQ->Finished() == 1, "Test the function has now finished");
}

// This function runs the following tests on the VolumeBBoxQuery::AddPrimitiveRef
//
// TEST ADDING A PRIMITIVE VOLUME TO THE RESULTS BUFFER 
// TEST ADDING A PRIMITIVE VOLUME, WITH NO MATRIX, TO THE RESULTS BUFFER  
// TEST ADDING A PRIMITIVE VOLUME, WITH ZERO SIZE VOLUME, TO THE RESULTS BUFFER  
// TEST RESULTS BUFFER FOR CORRECT VALUES, I.E. ONLY INTERSECTING VOLUMES  
// TEST RESULTS BUFFER OVERFLOW.
//
void TestVolumeBBox::TestAddPrimitiveRef()
{
    const uint32_t NUMINTERSECTIONS = 8;
    const uint32_t STACKSIZE = 10;
    const uint32_t RESBUFFERSIZE = 10;

    const Matrix44Affine mat
        (   
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        0.0f, 0.0f, 0.0f 
        );

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume *volArray[ NUMINTERSECTIONS ];
    for( int i = 0 ; i < (int)NUMINTERSECTIONS - 1 ; i++ )
    {
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // Create another box collision volume, with zero volume, and move it away from the origin so it doesn't intersect with the aabbox
    Volume *Box = EA::Physics::UnitFramework::Creator<BoxVolume>().New( 0.0f, 0.0f, 0.0f );
    Matrix44Affine mtx(GetMatrix44Affine_Identity());
    mtx.SetW(Vector3(-10.0f, 10.0f, 0.0f));
    Box->SetLocalTransform(mtx);
    // Add the box to the array
    volArray[ NUMINTERSECTIONS - 1 ] = Box;

    // VolumeBBoxQuery object
    VolumeBBoxQuery *p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINTERSECTIONS , aabb );

    // Run query and test return value
    EATESTAssert( p_VBBQ->GetOverlaps() == 7, "Test only 7 of the 8 primitives intersect");
    // Check that the non-intersecting volume is not part of the results array
    for( int i = 0 ; i < (int)NUMINTERSECTIONS - 1 ; i++ )
    {
            EATESTAssert( p_VBBQ->m_primVRefBuffer[ i ].volume == volArray[ i ], "Test only intersecting primitives added to results list");
    }

    // Add a primitive to the results list with a NULL matrix
    const Volume *Sphere = EA::Physics::UnitFramework::Creator<SphereVolume>().New( SPHERE_RADIUS );
    p_VBBQ->AddPrimitiveRef(Sphere, NULL, aabb, 1, 1);
    EATESTAssert( p_VBBQ->m_primNext == 8, "Test adding primitive to results list with a NULL matrix");


    // Add a primitive to the results list with a zero volume
    p_VBBQ->AddPrimitiveRef(Box, NULL, aabb, 1, 1);
    EATESTAssert( p_VBBQ->m_primNext == 9, "Test adding primitive to results list with zero volume" );


    // Add another primitive to the results list
    const Volume *Cylinder = EA::Physics::UnitFramework::Creator<CylinderVolume> ().New( CUBE_HALFLENGTH, SPHERE_RADIUS);
    p_VBBQ->AddPrimitiveRef(Cylinder, &mat, aabb, 1, 1);
    EATESTAssert( p_VBBQ->m_primNext == 10, "Test adding a primitive to the results list" );



    // Test overflowing the results list
    const Volume *Capsule = EA::Physics::UnitFramework::Creator<CapsuleVolume> ().New( CUBE_HALFLENGTH, SPHERE_RADIUS);
    EATESTAssert( p_VBBQ->AddPrimitiveRef(Capsule, &mat, aabb, 1, 1) == FALSE,  "Test overflowing the results list" );
}



// This function runs the following tests on the VolumeBBoxQuery::AddVolumeRef
//
// TEST AGGREGATE VOLUME STACK IS EMPTY WHEN INPUT VOLUMES IS EMPTY
// TEST ADDING AN AGGREGATE VOLUME TO THE STACK
// TEST ADDING A PRIMITIVE TO THE AGGREGATE STACK
// TEST ADDING AN AGGREGATE VOLUME, WITH NO MATRIX, TO THE AGGREGATE STACK
// TEST AGGREGATE STACK OVERFLOW 
//
void TestVolumeBBox::TestAddVolumeRef()
{
    const uint32_t NUMINPUTS = 0;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 10;

    
    // Create Translation Matrix - Identity Matrix
    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        0.0f, 0.0f, 0.0f 
        );

    // Create  Matrix array.
    const Matrix44Affine* matArray[ 1 ] = { &mat };

    // Create an array of pointers to volumes
    const Volume* volArray[1];

    // Create VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize Query
    p_VBBQ->InitQuery( &( volArray[ 0 ] ) , &( matArray[ 0 ] ), NUMINPUTS, aabb );

    // Run Query, checking no overlaps yet as there are no volumes initialized (numInputs = 0)
    EATESTAssert( p_VBBQ->GetOverlaps() == 0, "Test there are no overlaps yet");
    EATESTAssert( p_VBBQ->m_stackNext == 0, "Test the stack of volumes to evaluate is empty");



    // Create Aggregate / Aggregate Volume holding single intersecting Primitive Volume
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    // BoxVolume set as unit cube.
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();
    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );

    // Add the new aggregate to the stack
    p_VBBQ->AddVolumeRef(aggVol, matArray[0], aabb, 1, 1);

    EATESTAssert( p_VBBQ->m_stackNext == 1, "Test adding an aggregate volume reference");



    // Try adding a primitive to the stack list - should get added to the results buffer
    const Volume* Cylinder = EA::Physics::UnitFramework::Creator<CylinderVolume> ().New( CUBE_HALFLENGTH, SPHERE_RADIUS);
    p_VBBQ->AddVolumeRef(Cylinder, matArray[0], aabb, 1, 1);

    EATESTAssert( p_VBBQ->m_primNext == 1, "Test adding a primitive using AddVolumeRef adds it to results stack");
    EATESTAssert( p_VBBQ->m_stackNext == 1, "Test adding a primitive using AddVolumeRef doesn not add it to the aggregate stack");



    // Create Aggregate / Aggregate Volume holding single intersecting Primitive Volume
    SimpleMappedArray* p_SMAggregate2 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    // BoxVolume set as unit cube.
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate2->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate2->UpdateThis();
    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol2 = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate2 );

    // Add the new aggregate to the stack without a transformation matrix
    p_VBBQ->AddVolumeRef(aggVol2, NULL, aabb, 1, 1);

    EATESTAssert( p_VBBQ->m_stackNext == 2, "Test adding an aggregate volume with no transformation matrix");


    // Create Aggregate / Aggregate Volume holding single intersecting Primitive Volume
    SimpleMappedArray* p_SMAggregate3 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    // BoxVolume set as unit cube.
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate3->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate3->UpdateThis();
    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol3 = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate2 );

    
    // Add the new aggregate to the stack 
    EATESTAssert( p_VBBQ->AddVolumeRef(aggVol3, matArray[0], aabb, 1, 1) == FALSE, "Test AddVolumeRef returns false when stack is full");
    EATESTAssert( p_VBBQ->m_stackNext == 2, "Test over flowing the stack");
}

// This function runs the following tests on the VolumeBBoxQuery::GetResourceDescriptor
//
// TEST CREATING A DESCRIPTOR WITH ZERO ARGUMENTS
//
void TestVolumeBBox::GetResourceDescriptorZeroArgs()
{
    
    const uint32_t STACKMAX = 0;
    const uint32_t RESBUFFERSIZE = 0;

    // Create a resource descriptor
    EA::Physics::SizeAndAlignment resDesc = VolumeBBoxQuery::GetResourceDescriptor( STACKMAX, RESBUFFERSIZE );


    // Determine the size of object - this uses the same method used in the GetResourceDescriptor method.
    uintptr_t size = 0;

    size += EA::Physics::SizeAlign<uintptr_t>(sizeof(VolumeBBoxQuery), RWMATH_VECTOR3_ALIGNMENT);

    size += Max(sizeof(KDTree::BBoxQuery), sizeof(Octree::BBoxQuery));

    EATESTAssert( resDesc.GetSize() == size,  "GetResourceDescriptorZeroArgs - Size");
    EATESTAssert( resDesc.GetAlignment() == RWMATH_VECTOR3_ALIGNMENT,  "GetResourceDescriptorZeroArgs - Alignment ");
}

// This function runs the following tests on the VolumeBBoxQuery::GetResourceDescriptor
//
// TEST CREATING A DESCRIPTOR WITH NON-ZERO ARGUMENTS
//
void TestVolumeBBox::GetResourceDescriptorNonZeroArgs()
{
    const uint32_t STACKMAX = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Create a resource descriptor
    EA::Physics::SizeAndAlignment resDesc = VolumeBBoxQuery::GetResourceDescriptor( STACKMAX, RESBUFFERSIZE );

    // Determine the size of object - this uses the same method used in the GetResourceDescriptor method.
    uintptr_t size = 0;

    size += EA::Physics::SizeAlign<uintptr_t>(sizeof(VolumeBBoxQuery), RWMATH_VECTOR3_ALIGNMENT);

    size += sizeof(VolRef) * STACKMAX;

    size += sizeof(VolRef) * RESBUFFERSIZE;

    size += sizeof(Volume) * RESBUFFERSIZE;
    
    size += Max(sizeof(KDTree::BBoxQuery), sizeof(Octree::BBoxQuery));

    EATESTAssert( resDesc.GetSize() == size,  "GetResourceDescriptorNonZeroArgs - Size");
    EATESTAssert( resDesc.GetAlignment() == RWMATH_VECTOR3_ALIGNMENT,  "GetResourceDescriptorNonZeroArgs - Alignment");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST DEFAULT SWITCH CASE BY 'MANUALLY' ADDING A PRIMITIVE VOLUME TO THE AGGREGATE STACK AND USING A FULL RESULTS BUFFER
//  
void TestVolumeBBox::GetOverlapsPrimVolToAggStackFullBuffer()
{
    
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 0;

    // Array of Volume(s)
    Volume* volArray[ NUMINPUTS ];
    for( int i = 0 ; i < (int)NUMINPUTS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }
    
    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );

    // This test is used to access the default case of the switch statement in the GetOverlaps method.
    // It is used to increase code coverage.
    // THIS SECTION OF CODE IS TAKEN FROM THE "AddVolumeRef" METHOD.
    // Manually add a primitive volume to the stack buffer
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].volume = volArray[ 0 ];
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].tm = NULL;
    volArray[ 0 ]->GetBBox( NULL, 0, p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].bBox ) ;
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].tag  = 0;
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].numTagBits = 0;
    p_VBBQ->m_stackNext++;

    // Run query 
    p_VBBQ->GetOverlaps();

    // Check the results count, in this case it should be zero since the stack is of length zero.
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 0, "GetOverlapsPrimVolToAggStackFullBuffer");
}


// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST DEFAULT SWITCH CASE BY 'MANUALLY' ADDING A PRIMITIVE VOLUME TO THE AGGREGATE STACK AND USING A NON-FULL RESULTS BUFFER 
//  
void TestVolumeBBox::GetOverlapsPrimVolToAggStack()
{

    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 2;

    // Volume array holding one intersecting BoxVolume
    Volume* volArray[ NUMINPUTS ];
    for( int i = 0 ; i < (int)NUMINPUTS ; i++){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );

    // This test is used to access the default case of the switch statement in the GetOverlaps method.
    // It is used to increase code coverage.
    // THIS SECTION OF CODE IS TAKEN FROM THE "AddVolumeRef" METHOD.
    // Manually add a primitive volume to the stack buffer
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].volume = volArray[ 0 ];
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].tm = NULL;
    volArray[ 0 ]->GetBBox( NULL, 0, p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].bBox ) ;
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].tag  = 0;
    p_VBBQ->m_stackVRefBuffer[p_VBBQ->m_stackNext].numTagBits = 0;
    p_VBBQ->m_stackNext++;

    // Run query 
    p_VBBQ->GetOverlaps();

    // Check the results count, in this case it should be Two since the single volume is added twice.
    EATESTAssert( p_VBBQ->m_primNext == 2,  "GetOverlapsPrimVolToAggStack");
}


// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST DISABLED VOLUME USING A DISABLED AGGREGATE VOLUME 
//  
void TestVolumeBBox::GetOverlapsDisabledAggInter()
{
    
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Aggregate / Aggregate Volume holding single intersecting Primitive Volume
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    // BoxVolume set as unit cube.
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();
    // Create aggregate volume using aggregate, then disable the volume
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );
    aggVol->SetEnabled( FALSE );

    // Create AggregateVolume array.
    const Volume* volArray[ NUMINPUTS ] = { aggVol };

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize Query
    p_VBBQ->InitQuery( &( volArray[ 0 ] ) , NULL, NUMINPUTS, aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // Test the number of intersections, in this case it should be zero
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 0,  "GetOverlapsDisabledAggInter");
}



// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST DISABLED VOLUME USING A DISABLED PRIMITIVE VOLUME 
//  
void TestVolumeBBox::GetOverlapsDisabledPrimInter()
{

    // number of intersecting primitive volumes
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume* volArray[ NUMINPUTS ];
    for( int i = 0 ; i < (int)NUMINPUTS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
        volArray[ i ]->SetEnabled( FALSE ); 
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );

    p_VBBQ->GetOverlaps();

    // Test the number of intersections, in this case it should be zero
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 0,  "GetOverlapsDisabledPrimInter");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST STACK BUFFER FOR SINGLE AGGREGATE VOLUME INTERSECTION WITHOUT INPUT MATRIX 
//  
void TestVolumeBBox::GetOverlapsSingleAggInterNoMatrix()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Aggregate / Aggregate Volume holding single intersecting Primitive Volume
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( NUMINPUTS );
    // BoxVolume set as unit cube.
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();
    // aggregate volume using aggregate.
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );
    // AggregateVolume array.
    const Volume* volArray[ NUMINPUTS ] = { aggVol };

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize Query
    p_VBBQ->InitQuery( &( volArray[ 0 ] ) , NULL , NUMINPUTS, aabb );

    // Run Query
    p_VBBQ->GetOverlaps();

    // Obtain the results array
    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();

    // Test value of matrix held in results array, in this case it should be NULL.
    EATESTAssert( results[ 0 ].tm == NULL,  "GetOverlapsSingleAggInterNoMatrix");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST ZERO AGGREGATE VOLUME INTERSECTIONS AND ZERO PRIMITIVE VOLUME INTERSECTIONS
//  
void TestVolumeBBox::GetOverlapsZeroAggInterZeroPrimInter()
{

    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    BoxVolume::Initialize( EA::Physics::MemoryPtr( p_SMAggregate->GetVolume( 0 )), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    p_SMAggregate->UpdateThis();
    AggregateVolume* aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );
    const Volume* volArray[ NUMINPUTS ] = { aggVol };

    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        10.0f, 0.0f, 0.0f 
        );

    Matrix44Affine* matArray[ NUMINPUTS ] = { &mat };

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // Check for number of collisions, in this case it should be zero
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 0 ,  "GetOverlapsZeroAggInterZeroPrimInter");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST PRIMITIVES BUFFER OVERFLOW FUNCTIONALITY USING 2 INTERSECTING AGGREGATE VOLUMES AND A STACK BUFFER OF SIZE 1
//  
void TestVolumeBBox::GetOverlapsPrimitivesOverflowTwoAggInter()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    SimpleMappedArray* smaArray[ NUMINPUTS ];
    AggregateVolume* aggVol[ NUMINPUTS ];
    Volume* volArray[ NUMINPUTS ];

    for( int i = 0 ; i < (int)NUMINPUTS ; i++ ){
        // Aggregate holding a single intersecting primitive volume
        smaArray[ i ] = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
        BoxVolume::Initialize( EA::Physics::MemoryPtr( smaArray[ i ]->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
        smaArray[ i ]->UpdateThis();
        aggVol[ i ] = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( smaArray[ i ] );
        volArray[ i ] = aggVol[ i ];
    }
    
    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // obtain results
    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();

    // Check that the first Volume has been added to the results array.
    EATESTAssert( results[ 0 ].volume == smaArray[ 0 ]->GetVolume( 0 ),  "GetOverlapsPrimitivesOverflowTwoAggInter - 1st Volume");

    // Run query a second time
    p_VBBQ->GetOverlaps();

    // obtain results
    results = p_VBBQ->GetOverlapResultsBuffer();

    // Check that the second Volume has been added to the results array.
    EATESTAssert( results[ 0 ].volume == smaArray[ 1 ]->GetVolume( 0 ),  "GetOverlapsPrimitivesOverflowTwoAggInter - 2nd Volume");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST STACK BUFFER OVERFLOW FUNCTIONALITY USING 2 INTERSECTING AGGREGATE VOLUMES AND A STACK BUFFER OF SIZE 1
//  
void TestVolumeBBox::GetOverlapsStackOverflowTwoAggInter()
{
    const uint32_t NUMPRIMS = 2;
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    Volume* volArray[ NUMINPUTS ];
    Volume* resultsArray[ NUMPRIMS ];

    for( int i = 0 ; i < (int)NUMINPUTS ; i++ )
    {
        // Aggregate holding a single intersecting primitive volume
        SimpleMappedArray * smaArray0 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( NUMPRIMS );
        for (uint16_t j=0; j<NUMPRIMS; ++j)
        {
            SimpleMappedArray * smaArray1 = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
            AggregateVolume::Initialize( EA::Physics::MemoryPtr( smaArray0->GetVolume( j ) ), smaArray1 );
            {
                BoxVolume::Initialize( EA::Physics::MemoryPtr( smaArray1->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
                smaArray1->UpdateThis();
                resultsArray[j] = smaArray1->GetVolume( 0 );
            }
        }
        smaArray0->UpdateThis();
        volArray[ i ] = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( smaArray0 );
    }
    
    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , aabb );

    // Run query
    uint32_t numResults0 = p_VBBQ->GetOverlaps();

    // First GetOverlaps is supposed to hit stack overflow and return no results
    EATESTAssert( numResults0 == 0, "GetOverlapsStackOverflowTwoAggInter GetOverlaps Failed");

    // Run query a second time
    uint32_t numResults1 = p_VBBQ->GetOverlaps();

    EATESTAssert( numResults1 == 1, "GetOverlapsStackOverflowTwoAggInter GetOverlaps Failed");

    // obtain results
    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();

    // Second GetOverlaps is supposed to proceed to the last remaining volume and return a valid result
    EATESTAssert( results[ 0 ].volume == resultsArray[ 1 ],  "GetOverlapsStackOverflowTwoAggInter - 2nd Volume");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST SINGLE AGGREGATE VOLUME INTERSECTION WITH NO PRIMITIVE VOLUME INTERSECTIONS
//
void TestVolumeBBox::GetOverlapsAggInterNoPrimInter()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t NUMPRIMS = 2;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 2; 

    // Aggregate holding two primitive volumes
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( NUMPRIMS );
    // BoxVolume set as unit cube.
    for( int i = 0 ; i < (int)NUMPRIMS ; i++ ){
        BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( (uint16_t) i ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // Displace the two primitives so they don't intersect with the AABB
    p_SMAggregate->GetVolume( 0 )->SetLocalTransform(Matrix44Affine(1.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f,
                                                                    0.0f, 0.0f, 1.0f,
                                                                    10.0f, 0.0f, 0.0f 
                                                                    ));

    p_SMAggregate->GetVolume( 1 )->SetLocalTransform(Matrix44Affine(1.0f, 0.0f, 0.0f,
                                                                    0.0f, 1.0f, 0.0f,
                                                                    0.0f, 0.0f, 1.0f,
                                                                    -10.0f, 0.0f, 0.0f 
                                                                    ));

    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();

    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );

    Volume* volArray[ 1 ] = { aggVol };

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();
    
    // Check the primitive volume result counter, should be zero in this case. 
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 0,  "GetOverlapsAggInterNoPrimInter");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER OVERFLOW FUNCTIONALITY USING 2 INTERSECTING PRIMITIVE VOLUMES AND A RESULTS ARRAY OF SIZE 1 
//
void TestVolumeBBox::GetOverlapsOverflowTwoPrimInter()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    Volume* volArray[ NUMINPUTS ];
    for( int i  = 0 ; i < (int)NUMINPUTS ;  i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&(volArray[ 0 ]), NULL , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // obtain results
    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();

    // Check that the first Volume has been added to the results array.
    EATESTAssert( results[ 0 ].volume == volArray[ 0 ],  "GetOverlapsOverflowTwoPrimInter - 1st Volume");

    // Run query a second time
    p_VBBQ->GetOverlaps();

    // obtain results
    results = p_VBBQ->GetOverlapResultsBuffer();
    
    // Check that the second Volume has been added to the results array.
    EATESTAssert( results[ 0 ].volume == volArray[ 1 ],  "GetOverlapsOverflowTwoPrimInter - 2nd Volume");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER FOR CORRECT INTERSECTION VALUES WHEN USING 2 PRIMITIVE VOLUMES, THE SECOND OF WHICH INTERSECTS
//
void TestVolumeBBox::GetOverlapsCorrectPrimInterResults()
{
    const uint32_t NUMINPUTS = 2;
    const uint32_t STACKSIZE = 2;
    const uint32_t RESBUFFERSIZE = 2;

    Volume* volArray[ NUMINPUTS ];
    for( int i = 0 ; i < (int)NUMINPUTS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    Matrix44Affine* matArray[ NUMINPUTS ];

    // Setup first volume for non-intersection
    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        10.0f, 0.0f, 0.0f 
        );

    matArray[ 0 ] = &mat;

    // Setup second volume for non-intersection
    matArray[ 1 ] = NULL;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&(volArray[ 0 ]), (const Matrix44Affine** )&(matArray[ 0 ]) , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // Get results
    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();
    
    // Compare the value of the volume held in the results buffer, 
    // should be equal to the address of the second volume in the input array
    EATESTAssert( results[ 0 ].volume == volArray[ 1 ],  "GetOverlapsCorrectPrimInterResults");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER FOR FOR NULL INPUT VOLUMES AND ARRAYS 
//
void TestVolumeBBox::GetOverlapsNullInputArrays()
{
    const uint32_t NUMINPUTS = 0;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( NULL , NULL , NUMINPUTS , aabb );

    // Run query. Call should complete without error. 
    p_VBBQ->GetOverlaps();
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER FOR SINGLE PRIMITIVE VOLUME INTERSECTION WITHOUT INPUT MATRIX 
//
void TestVolumeBBox::GetOverlapsSinglePrimInterNoMatrix()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    //Box Volume
    Volume* volArray[ NUMINPUTS ];
    volArray[ 0 ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // Obtain results array
    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();

    // Check the value of the matrix held in the results buffer, should be NULL. 
    EATESTAssert( results[0].tm == NULL ,  "GetOverlapsSinglePrimInterNoMatrix");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER AND COUNTER FOR ZERO PRIMITIVE VOLUME INTERSECTIONS  
//
void TestVolumeBBox::GetOverlapsZeroPrimInterResultsBufferCounter()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;
    
    // Box Volume and corresponding matrix
    Volume* volArray[ NUMINPUTS ];

    volArray[ 0 ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );

    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        10.0f, 0.0f, 0.0f 
        );

    Matrix44Affine* matArray[ NUMINPUTS ] = { &mat };

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , (const Matrix44Affine**)&( matArray[0] ) , NUMINPUTS , aabb );

    // Run query
    p_VBBQ->GetOverlaps();

    // Check the volume has not been added by examining the result counter. 
    EATESTAssert( p_VBBQ->GetOverlapResultsBufferCount() == 0,  "GetOverlapsZeroPrimInterResultsBufferCounter");
}


// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER AND COUNTER FOR SINGLE PRIMITIVE VOLUME INTERSECTION  
//
void TestVolumeBBox::GetOverlapsSinglePrimInterResultsBuffer()
{
    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;

    // Single BoxVolume
    Volume* volArray[ NUMINPUTS ];

    volArray[ 0 ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );
    
    p_VBBQ->GetOverlaps();

    VolRef* results = p_VBBQ->GetOverlapResultsBuffer();

    // Check the volume has been added by comparing pointers to the volume. 
    EATESTAssert( results[ 0 ].volume == volArray[ 0 ],  "GetOverlapsSinglePrimInterResultsBuffer");
}



// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RESULT BUFFER AND COUNTER FOR NUMINPUT = ZERO   
//
void TestVolumeBBox::GetOverlapsZeroInputs()
{
    const uint32_t NUMINPUTS = 0;
    const uint32_t STACKSIZE = 1;
    const uint32_t RESBUFFERSIZE = 1;
    
    // Single BoxVolume
    Volume* volArray[ 1 ];
    
    volArray[ 0 ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINPUTS , aabb );

    // Run query and test return value
    EATESTAssert( p_VBBQ->GetOverlaps() == NUMINPUTS,  "GetOverlapsZeroInputs");
}



// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RETURN VALUE FOR X NUMBER OF AGGREGATE INTERSECTIONS 
//
void TestVolumeBBox::GetOverlapsReturnValueAggregate()
{
    const uint32_t NUMINTERSECTIONS = 10;
    const uint32_t STACKSIZE = 10;
    const uint32_t RESBUFFERSIZE = 10;

    // Single Aggregate holding an intersecting primitive volume.
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();

    Volume* volArray[ NUMINTERSECTIONS ];
    
    for( int i = 0 ; i < (int)NUMINTERSECTIONS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINTERSECTIONS , aabb );

    // Run query and test return value
    EATESTAssert( p_VBBQ->GetOverlaps() == NUMINTERSECTIONS,  "GetOverlapsReturnValueAggregate");
}


// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST RETURN VALUE FOR X NUMBER OF PRIMITIVE INTERSECTIONS
//
void TestVolumeBBox::GetOverlapsReturnValuePrimitive()
{
    // number of intersecting primitive volumes
    const uint32_t NUMINTERSECTIONS = 10;
    const uint32_t STACKSIZE = 10;
    const uint32_t RESBUFFERSIZE = 10;

    // Array of NUMINTERSECTION intersecting primitive volumes
    Volume* volArray[ NUMINTERSECTIONS ];
    
    for( int i = 0 ; i < (int)NUMINTERSECTIONS ; i++ ){
        volArray[ i ] = EA::Physics::UnitFramework::Creator<BoxVolume>().New( CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    }

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    // Initialize query
    p_VBBQ->InitQuery( (const Volume**)&( volArray[ 0 ] ) , NULL , NUMINTERSECTIONS , aabb );

    // Run query and test return value
    EATESTAssert( p_VBBQ->GetOverlaps() == NUMINTERSECTIONS,  "GetOverlapsReturnValuePrimitive");
}

// This function runs the following tests on the VolumeBBoxQuery::GetOverlaps
//
// TEST SINGLE AGGREGATE VOLUME INTERSECTION WITH SINGLE PRIMITIVE VOLUME INTERSECTION.
//
void TestVolumeBBox::GetOverlapsSingleAggSinglePrimInterResultsBuffer()
{

    const uint32_t NUMINPUTS = 1;
    const uint32_t STACKSIZE = 10;
    const uint32_t RESBUFFERSIZE = 10;

    // Create Aggregate / Aggregate Volume holding single intersecting Primitive Volume
    SimpleMappedArray* p_SMAggregate = EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New( 1u );
    // BoxVolume set as unit cube.
    BoxVolume::Initialize( EA::Physics::MemoryPtr(p_SMAggregate->GetVolume( 0 ) ), CUBE_HALFLENGTH, CUBE_HALFLENGTH, CUBE_HALFLENGTH );
    // Update aggregate collision volume.
    p_SMAggregate->UpdateThis();
    // Create aggregate volume using aggregate.
    AggregateVolume *aggVol = EA::Physics::UnitFramework::Creator<AggregateVolume>().New( p_SMAggregate );
    // Create AggregateVolume array.
    const Volume* volArray[ NUMINPUTS ] = { aggVol };
    
    // Create Translation Matrix - Identity Matrix
    Matrix44Affine mat
        (
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 
        0.0f, 0.0f, 0.0f 
        );

    // Matrix array.
    const Matrix44Affine* matArray[ NUMINPUTS ] = { &mat };

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );
    
    // Initialize Query
    p_VBBQ->InitQuery( &( volArray[ 0 ] ) , &( matArray[ 0 ] ), NUMINPUTS, aabb );

    // Run Query
    p_VBBQ->GetOverlaps();

    // Obtain the results array
    VolRef* testResultsArray = p_VBBQ->GetOverlapResultsBuffer();

    // Test volume held in the results array by comparing tag to input primitive volume tag
    // Tag generation occurs when aggregate volumes are processed by the derived aggregate object. In this case
    // the simplemappedarray object generates the tag, for the single primitive volume, as 1.
    // Another method with which to test the volume would be to compare the address' of the volume
    // in the input array and the volume in the results array, which should match.

    EATESTAssert( testResultsArray[ 0 ].tag == 1,  "GetOverlapsSingleAggSinglePrimInterResultsBuffer");
}

