// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>
#include <eaphysics/base.h>
#include <rw/collision/libcore.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#define DISABLE_KNOWN_FAILING_UNITTESTS

using namespace rwpmath;
using namespace rw::collision;


// EPSILON is largest FP error
static const float SQRT_THIRD = 0.577350269f;
static const float LENGTH_SCALE = 5.0f;
static const float LINE_PARAM_TOL = 20.0f * EPSILON;
static const float POSITION_TOL = 20.0f * LENGTH_SCALE * EPSILON;
static const float NORMAL_TOL = 20.0f * EPSILON; 


class TestBoxLineQuery: public tests::TestSuiteBase
{
public:
    virtual void Initialize()
    {
        SuiteName("TestBoxLineQuery");
        EATEST_REGISTER("BoxVolumeLineSegIntersect", "Test box volume line seg intersect" , TestBoxLineQuery, BoxVolumeLineSegIntersect);   
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

private:
    void BoxVolumeLineSegIntersect();

} TestBoxLineQuerySingleton;


/*
Currently tests specific region crossing cases for a bug. Other tests yet to be integrated from 
    //EAOS/Physics/Work/collision_opt/rwphysics_base/work_collisionopt
*/
void TestBoxLineQuery::BoxVolumeLineSegIntersect()
{
    // Unit box with unit fatness
    VecFloat fatness(1.0f);
    Vector3 halfDims(1.0f, 1.0f, 1.0f);
    BoxVolume* box = EA::Physics::UnitFramework::Creator<BoxVolume>().New(halfDims);  
    box->SetRadius(fatness);

    // Construct points on fat face, edge, and corner
    Vector3 facePoint(0.0f, 0.0f, float(halfDims.Z() + fatness));
    Vector3 faceNormal(0.0f, 0.0f, 1.0f);
    Vector3 edgePoint(float(halfDims.X() + SQRT_HALF*fatness), 0.0f, float(halfDims.Z() + SQRT_HALF*fatness));
    Vector3 edgeNormal(SQRT_HALF,  0.0f, SQRT_HALF);
    Vector3 cornerPoint = halfDims + SQRT_THIRD * fatness;
    Vector3 cornerNormal(SQRT_THIRD, SQRT_THIRD, SQRT_THIRD);

    // Make sure we can see corner and edge points (start.normal > point.normal)
    Vector3 startInFaceRegion = facePoint 
        + GetVecFloat_Two() * faceNormal * (Dot(cornerPoint - facePoint, cornerNormal) / Dot(faceNormal, cornerNormal));
    Vector3 startInEdgeRegion = edgePoint 
        + GetVecFloat_Two() * edgeNormal * (Dot(cornerPoint - edgePoint, cornerNormal) / Dot(edgeNormal, cornerNormal));
    Vector3 startInCornerRegion = halfDims + GetVecFloat_Two() * fatness;

    // Define 6 line tests crossing region boundaries
    struct
    {
        Vector3     start;
        Vector3     hitPoint;
        Vector3     hitNormal;
        const char *descrip;
    } 
    segments[6] = 
    {
        {startInFaceRegion, edgePoint, edgeNormal,       "face -> edge region" },
        {startInEdgeRegion, facePoint, faceNormal,       "edge -> face region" },
        {startInFaceRegion, cornerPoint, cornerNormal,   "face -> corner region" },
        {startInCornerRegion, facePoint, faceNormal,     "corner -> face region" },
        {startInEdgeRegion, cornerPoint, cornerNormal,   "edge -> corner region" },
        {startInCornerRegion, edgePoint, edgeNormal,     "corner -> edge region" }
    };

    for (uint32_t i=0; i<6; i++)
    {
        // Make line intersect half way
        Vector3 end = segments[i].start + VecFloat(2.0f) * (segments[i].hitPoint - segments[i].start);
        VolumeLineSegIntersectResult result;
        RwpBool hit = box->LineSegIntersect(segments[i].start, end, 0 /* transform */, result, 0.0f /* lineFatness */);
        EATESTAssert(hit, "fails to return hit" );
        EATESTAssert(IsSimilar(result.lineParam, 0.5f, LINE_PARAM_TOL), "unexpected lineParam" );
        EATESTAssert(IsSimilar(result.position, segments[i].hitPoint, POSITION_TOL), "unexpected intersection position" );
        EATESTAssert(IsSimilar(result.normal, segments[i].hitNormal, NORMAL_TOL), "unexpected intersection normal" );
    }
}
