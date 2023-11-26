// (c) Electronic Arts. All Rights Reserved.

#include <unit/unit.h>

#include <EABase/eabase.h>

#include <rw/collision/libcore.h>
#include <rw/collision/detail/fpu/clusteredmesh.h>

#include <serialization/serialization.h>
#include <serialization/binary_stream_oarchive.h>
#include <serialization/binary_stream_iarchive.h>
#include <eaphysics/hlserializable.h>


#include "clusteredmeshtest_base.hpp"
#include "SimpleStream.hpp"

using namespace rwpmath;
using namespace rw::collision;

bool ClusteredMeshTest_Base::AreTrianglesTheSame(const TriangleVolume * volumeA, const TriangleVolume * volumeB)
{

    Vector3 triangleAVertices[3];
    volumeA->GetPoints(triangleAVertices[0], triangleAVertices[1], triangleAVertices[2], NULL);
    Vector3 triangleANormal;
    volumeA->GetNormal(triangleANormal, NULL);

    Vector3 triangleBVertices[3];
    volumeB->GetPoints(triangleBVertices[0], triangleBVertices[1], triangleBVertices[2], NULL);
    Vector3 triangleBNormal;
    volumeB->GetNormal(triangleBNormal, NULL);

    bool similar = true;

    similar &= IsSimilar(triangleAVertices[0], triangleBVertices[0]);
    similar &= IsSimilar(triangleAVertices[1], triangleBVertices[1]);
    similar &= IsSimilar(triangleAVertices[2], triangleBVertices[2]);
    similar &= IsSimilar(triangleANormal, triangleBNormal);

    if (!similar)
    {
        return false;
    }

    if (volumeA->GetFlags() != volumeB->GetFlags())
    {
        return false;
    }

    rwpmath::Mask3 useEdgeCosines(
        (volumeA->GetFlags() & rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX) != 0,
        (volumeA->GetFlags() & rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX) != 0,
        (volumeA->GetFlags() & rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX) != 0);
    rwpmath::Vector3 ecA = rwpmath::Select(useEdgeCosines, volumeA->GetEdgeCosVector(), rwpmath::GetVector3_One());
    rwpmath::Vector3 ecB = rwpmath::Select(useEdgeCosines, volumeB->GetEdgeCosVector(), rwpmath::GetVector3_One());

    if (!IsSimilar(ecA, ecB))
    {
        return false;
    }

    if (!IsSimilar(volumeA->GetRadius(), volumeB->GetRadius()))
    {
        return false;
    }

    if (volumeA->GetGroup() != volumeB->GetGroup())
    {
        return false;
    }

    if (volumeA->GetSurface() != volumeB->GetSurface())
    {
        return false;
    }

    return true;
}

#if !defined(EA_PLATFORM_PS3_SPU)
void ClusteredMeshTest_Base::AddInputToBuilder(
    rw::collision::ClusteredMeshOfflineBuilder & offlineBuilder,
    const uint32_t triangleXCount,
    const uint32_t triangleZCount)
{
    typedef rw::collision::meshbuilder::VectorType VectorType;

    uint32_t vertexIndex = 0;
    uint32_t triangleIndex = 0;

    for (uint32_t triangleXIndex = 0 ; triangleXIndex < triangleXCount ; ++triangleXIndex)
    {
        for (uint32_t triangleZIndex = 0 ; triangleZIndex < triangleZCount ; ++triangleZIndex)
        {
            VectorType v0(static_cast<float>(triangleXIndex),     0.0f, static_cast<float>(triangleZIndex));
            VectorType v1(static_cast<float>(triangleXIndex),     0.0f, static_cast<float>(triangleZIndex + 1));
            VectorType v2(static_cast<float>(triangleXIndex + 1), 0.0f, static_cast<float>(triangleZIndex));
            VectorType v3(static_cast<float>(triangleXIndex + 1), 0.0f, static_cast<float>(triangleZIndex + 1));

            offlineBuilder.SetVertex(vertexIndex, v0);
            offlineBuilder.SetVertex(vertexIndex + 1u, v1);
            offlineBuilder.SetVertex(vertexIndex + 2u, v2);

            offlineBuilder.SetTriangle(triangleIndex,
                vertexIndex,
                vertexIndex + 1u,
                vertexIndex + 2u);

            vertexIndex += 3;
            ++triangleIndex;

            offlineBuilder.SetVertex(vertexIndex, v1);
            offlineBuilder.SetVertex(vertexIndex + 1u, v3);
            offlineBuilder.SetVertex(vertexIndex + 2u, v2);

            offlineBuilder.SetTriangle(triangleIndex,
                vertexIndex,
                vertexIndex + 1u,
                vertexIndex + 2u);

            vertexIndex += 3;
            ++triangleIndex;
        }
    }
}
rw::collision::ClusteredMesh *
ClusteredMeshTest_Base::BuildClusteredMesh(
    const uint32_t xCount,
    const uint32_t zCount,
    const bool quads)
{
    const uint32_t triangleCount = xCount * zCount * 2;
    const uint32_t vertexCount = triangleCount * 3;

    // Create mesh builder parameters
    rw::collision::ClusteredMeshOfflineBuilder::Parameters params;
    params.quads_Enable = quads;

    // Create mesh builder
    rw::collision::ClusteredMeshOfflineBuilder offlineBuilder(
        triangleCount,
        vertexCount,
        0u,
        params,
        EA::Allocator::ICoreAllocator::GetDefaultAllocator());

    AddInputToBuilder(
        offlineBuilder,
        xCount,
        zCount);

    return offlineBuilder.BuildClusteredMesh();
}
#endif //!defined(EA_PLATFORM_PS3_SPU)

void ClusteredMeshTest_Base::LineQueryTester(const Volume* meshVolume, const Volume *scaledMeshVolume, const rwpmath::Matrix44Affine* transformMatrix, VolumeBBoxQuery* bboxQuery, VolumeLineQuery* triangleLineQuery, VolumeLineQuery* scaledMeshLineQuery, float scale, float accuracy)
{   
    uint32_t numResults = 0;
    
    //Set up the box query
    rw::collision::AABBox volBBox;
    meshVolume->GetBBox(0, TRUE, volBBox);
    bboxQuery->InitQuery(&meshVolume, &transformMatrix, 1, volBBox);

    // We carry out a single line test for each triangle in the mesh, cycling the line fatness through these
    // values as as we iterate over the triangles.
    const uint32_t NUM_LINE_FATNESSES = 4u;
    float lineFatnesses[NUM_LINE_FATNESSES];
    lineFatnesses[0] = 0.0f;
    lineFatnesses[1] = .33f * scale;
    lineFatnesses[2] = .66f * scale;
    lineFatnesses[3] = .99f * scale;

    //Perform the bbox query, obtaining all triangles
    while (bboxQuery->GetOverlaps())
    {
        const uint32_t numTriangles = bboxQuery->GetOverlapResultsBufferCount();
        VolRef * triangleBuffer = bboxQuery->GetOverlapResultsBuffer();

        //Loop over every triangle in results buffer
        for (uint32_t i = 0; i < numTriangles; ++i)
        {
            TriangleVolume *testTriangle = const_cast<TriangleVolume*>(static_cast<const TriangleVolume*>(triangleBuffer[i].volume));

            rwpmath::Vector3 p0;
            rwpmath::Vector3 p1;
            rwpmath::Vector3 p2;
                
            //scale the triangle
            testTriangle->ApplyUniformScale(scale);

            testTriangle->GetPoints(p0, p1, p2, triangleBuffer[i].tm);         

            rwpmath::Vector3 triCentre = (p0 + p1 + p2) / 3.0f;
            triCentre += (p1 - p0) * 0.1f; // Offset slightly so the barycentric coordinate are not the same
            triCentre += (p2 - p0) * 0.2f; // Offset slightly so the barycentric coordinate are not the same
            const rwpmath::Vector3 triNormal = rwpmath::Normalize(rwpmath::Cross(p1 - p0, p2 - p0));

            //create a point which passes through the triangle and scale line
            const float lineHalfLength = 1.0f;
            const rwpmath::Vector3 lineStart = (triCentre +  lineHalfLength * triNormal);
            const rwpmath::Vector3 lineEnd = (triCentre - lineHalfLength * triNormal);

            // Select a line fatness for this triangle
            const float lineFatness = lineFatnesses[i % NUM_LINE_FATNESSES];

            // Perform triangle line test, should find one result (as we query one triangle at a time)
            triangleLineQuery->InitQuery(&triangleBuffer[i].volume, const_cast<const rwpmath::Matrix44Affine**>(&triangleBuffer[i].tm), 1, lineStart, lineEnd, lineFatness);
            const uint32_t numTriangleResults = triangleLineQuery->GetAllIntersections();                
            EATESTAssert(1 == numTriangleResults, "Incorrect number of triangles found, should have found exactly one");
            VolumeLineSegIntersectResult const *triLineQueryResults = triangleLineQuery->GetIntersectionResultsBuffer();

            // ClusteredMesh line test
            scaledMeshLineQuery->InitQuery(&scaledMeshVolume, &transformMatrix, 1, lineStart, lineEnd, lineFatness);
            VolumeLineSegIntersectResult const *meshLineQueryResults = 0;
            bool triFound = false;

            // Query the mesh and check that we find the triangle and that the parameters match
            while (!triFound && !scaledMeshLineQuery->Finished())
            {
                const uint32_t numMeshResults = scaledMeshLineQuery->GetAllIntersections();
                EATESTAssert(numMeshResults > 0, "Did not find any intersections with mesh");
                meshLineQueryResults = scaledMeshLineQuery->GetIntersectionResultsBuffer();

                for (uint32_t numQResults = 0; numQResults < numMeshResults; ++numQResults, ++meshLineQueryResults)
                {
                    TriangleVolume const * resultTri = static_cast<const TriangleVolume*>(meshLineQueryResults->vRef.volume);

                    if (AreTrianglesTheSame(testTriangle, resultTri))
                    {    
                        EATESTAssert(triangleBuffer[i].numTagBits == meshLineQueryResults->vRef.numTagBits, "'numTagBits' does not match.");                            
                        EATESTAssert(triangleBuffer[i].tag == meshLineQueryResults->vRef.tag, "'tag' does not match.");

                        triFound = true;
                        break;
                    }
                }
            }
             
            EATESTAssert(triFound, "No matching triangle found.");

            // Check the triangle and clustered mesh line query results match
            EATESTAssert(rwpmath::IsSimilar(triLineQueryResults->lineParam, meshLineQueryResults->lineParam, accuracy), "'lineParam' does not match.");
            EATESTAssert(rwpmath::IsSimilar(triLineQueryResults->normal,    meshLineQueryResults->normal, accuracy), "'normal' does not match.");
            EATESTAssert(rwpmath::IsSimilar(triLineQueryResults->position,  meshLineQueryResults->position, accuracy*scale), "'position' does not match."); 
            //Get larger discrepancies in the volume parameter. Therefore use a larger tolerance. We use the max of scale and 1 as one component is scale
            //dependent and two components are scale independent.
            //Also only attempt to test triangles whose sides have a ratio of less than 5.
            float length1 = Magnitude(p0 - p1);
            float length2 = Magnitude(p0 - p2);
            float length3 = Magnitude(p1 - p2);
            float ratio1 = length1/length2;
            float ratio2 = length1/length3;
            float ratio3 = length2/length3;               
            if((ratio1 < 5.0f  && ratio1 > 0.2f) && (ratio2 < 5.0f && ratio2 > 0.2f) && (ratio3 < 5.0f && ratio3 > 0.2f))
            {
                EATESTAssert(rwpmath::IsSimilar(triLineQueryResults->volParam,  meshLineQueryResults->volParam, 1.0e-1f*rwpmath::Max(scale,1.0f)), "'volParam' does not match.");                
            }
        }

        numResults += bboxQuery->GetOverlapResultsBufferCount();
    }
}

void ClusteredMeshTest_Base::RestartingLineQueryTester(const rw::collision::Volume* meshVolume, uint32_t fullBufferSize, uint32_t stackSize, uint32_t smallBufferSize)
{
    const Volume * meshLineTestVolumeArray[] = { meshVolume };

    rw::collision::AABBox volBBox;
    meshVolume->GetBBox(0, TRUE, volBBox);
    const Vector3 middle = (volBBox.Min() + volBBox.Max()) * rwpmath::GetVecFloat_Half();
    const rwpmath::Vector3 lineStart(middle.X(), volBBox.Max().Y(), middle.Z());
    const rwpmath::Vector3 lineEnd(middle.X(), volBBox.Min().Y(), middle.Z());
    
    //Obtain all results of the line with the mesh
    VolumeLineQuery* meshLineQueryAll = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(stackSize, fullBufferSize);
    meshLineQueryAll->InitQuery(meshLineTestVolumeArray, 0, 1, lineStart, lineEnd, 1.0f);

    const uint32_t numExpectedResults = meshLineQueryAll->GetAllIntersections();
    const VolumeLineSegIntersectResult * expectedResults = meshLineQueryAll->GetIntersectionResultsBuffer();

    EATESTAssert(numExpectedResults, "No results found.");
    EATESTAssert(meshLineQueryAll->Finished(), "Query has not finished try increasing the result buffer size.");

    for (uint32_t bufferSize = 1; bufferSize < smallBufferSize; ++bufferSize)
    {
        uint32_t totalNumResults = 0;

        // Create line query to use against clustered mesh which can hold fewer results
        VolumeLineQuery* meshLineQuery = EA::Physics::UnitFramework::Creator<VolumeLineQuery>().New(stackSize, bufferSize);
        EATESTAssert(meshLineQuery, "Failed to create clustered mesh line query.");

        meshLineQuery->InitQuery(meshLineTestVolumeArray, 0, 1, lineStart, lineEnd, 1.0f);

        //Perform query until all results are found (query will be restarted)
        while (!meshLineQuery->Finished())
        {
            const uint32_t numResults = meshLineQuery->GetAllIntersections();
            const VolumeLineSegIntersectResult * resultsBuffer = meshLineQuery->GetIntersectionResultsBuffer();

            //For each results in buffer check it exists in the expected results buffer
            for (uint32_t i = 0; i < numResults; ++i)
            {
                EATESTAssert(expectedResults[totalNumResults + i].vRef.numTagBits ==resultsBuffer[i].vRef.numTagBits, "'numTagBits' does not match.");
                EATESTAssert(expectedResults[totalNumResults + i].vRef.tag == resultsBuffer[i].vRef.tag, "'tag' does not match.");

                AssertTrianglesTheSame(
                    static_cast<const TriangleVolume *>(expectedResults[totalNumResults + i].v),
                    static_cast<const TriangleVolume *>(resultsBuffer[i].v));
            }

            totalNumResults += numResults;
        }

        //Check we found all results
        EATESTAssert(totalNumResults == numExpectedResults, "Number of results returned from line query is incorrect.");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshLineQuery);
    }

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(meshLineQueryAll);   

}

void ClusteredMeshTest_Base::RestartingBBoxQueryTester(const rw::collision::Volume* meshVolume, const rwpmath::Matrix44Affine* transformMatrix, uint32_t fullBufferSize, uint32_t stackSize, uint32_t smallBufferSize)
{
    float scale = 1.0f;
    const float tolerance = 1e-6f;
    const rw::collision::Aggregate * unscaledMesh = NULL;

    EA_ASSERT(meshVolume->GetType() == rw::collision::VOLUMETYPEAGGREGATE);
    const rw::collision::Aggregate * agg = static_cast<const rw::collision::AggregateVolume *>(meshVolume)->GetAggregate();
    if (agg->GetType() == rw::collision::RWCOBJECTTYPE_SCALEDCLUSTEREDMESH)
    {
        const rw::collision::ScaledClusteredMesh * scaledCM = static_cast<const rw::collision::ScaledClusteredMesh *>(agg);
        scale = scaledCM->GetScale();
        unscaledMesh = scaledCM->GetClusteredMesh();
    }
    else
    {
        EA_ASSERT(agg->GetType() ==  rw::collision::RWCOBJECTTYPE_CLUSTEREDMESH);
        unscaledMesh = static_cast<const rw::collision::ClusteredMesh *>(agg);
    }
    rw::collision::Volume unscaledVolume;
    rw::collision::AggregateVolume::Initialize(&unscaledVolume, const_cast<rw::collision::Aggregate *>(unscaledMesh));
    const rw::collision::Volume * volumePtr = &unscaledVolume;

    rw::collision::AABBox volBBox;
    unscaledVolume.GetBBox(transformMatrix, TRUE, volBBox);
    
    // VolumeBBoxQuery object for all results against unscaled mesh
    VolumeBBoxQuery* queryAll = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(stackSize, fullBufferSize);
    queryAll->InitQuery(&volumePtr, &transformMatrix, 1, volBBox);

    const uint32_t numExpectedResults = queryAll->GetOverlaps();
    EATESTAssert(numExpectedResults, "No results found.");
    EATESTAssert(queryAll->Finished(), "Query has not finished try increasing the result buffer size.");

    const VolRef *allVolRef = queryAll->GetOverlapResultsBuffer();

    for (uint32_t resSize = 1u; resSize <= smallBufferSize; ++resSize)
    {
        // VolumeBBoxQuery object
        VolumeBBoxQuery* query = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New(stackSize, resSize);
        rw::collision::AABBox scaledVolBBox;
        meshVolume->GetBBox(transformMatrix, TRUE, scaledVolBBox);
        query->InitQuery(&meshVolume, &transformMatrix, 1, scaledVolBBox);

        uint32_t totalNumResults = 0;
        while (query->GetOverlaps())
        {
            const VolRef *volRef      = query->GetOverlapResultsBuffer();
            const uint32_t numResults = query->GetOverlapResultsBufferCount();

            EATESTAssert(totalNumResults + numResults <= numExpectedResults, "Too many results.");

            for (uint32_t i = 0; i < numResults; ++i)
            {
                EATESTAssert(allVolRef[totalNumResults + i].numTagBits == volRef[i].numTagBits, "'numTagBits' does not match.");
                EATESTAssert(allVolRef[totalNumResults + i].tag == volRef[i].tag, "'tag' does not match.");

                const TriangleVolume * volumeRef = static_cast<const TriangleVolume *>(allVolRef[totalNumResults + i].volume);
                const TriangleVolume * volumeCheck = static_cast<const TriangleVolume *>(volRef[i].volume);

                // The triangle is in volume space (ie scaled clustered mesh space, not including transformMatrix)
                rwpmath::Vector3 e0, e1, e2, p0, p1, p2;
                volumeRef->GetPoints(e0, e1, e2);
                volumeCheck->GetPoints(p0, p1, p2);
                EATESTAssert(IsSimilar(e0*scale, p0, tolerance * scale * Magnitude(e0)), "Vertex 0 should be scaled");
                EATESTAssert(IsSimilar(e1*scale, p1, tolerance * scale * Magnitude(e1)), "Vertex 1 should be scaled");
                EATESTAssert(IsSimilar(e2*scale, p2, tolerance * scale * Magnitude(e2)), "Vertex 2 should be scaled");

                EATESTAssert(IsSimilar(volumeRef->GetRadius()*scale, volumeCheck->GetRadius()), "Volume radius does not match");
                EATESTAssert(volumeRef->GetGroup() ==  volumeCheck->GetGroup(), "Volume group does not match");
                EATESTAssert(volumeRef->GetSurface() == volumeCheck->GetSurface(), "Volume surface id does not match");
                EATESTAssert(volumeRef->GetFlags() == volumeCheck->GetFlags(), "Volume flags do not match");

                rwpmath::Mask3 useEdgeCosines(
                    (volumeRef->GetFlags() & rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX) != 0,
                    (volumeRef->GetFlags() & rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX) != 0,
                    (volumeRef->GetFlags() & rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX) != 0);               

                rwpmath::Vector3 ecRef = rwpmath::Select(useEdgeCosines, volumeRef->GetEdgeCosVector(), rwpmath::GetVector3_One());
                rwpmath::Vector3 ecCheck = rwpmath::Select(useEdgeCosines, volumeCheck->GetEdgeCosVector(), rwpmath::GetVector3_One());
                EATESTAssert(IsSimilar(ecRef, ecCheck), "Edge cosine data does not match");

                //transform the bounding box into world space after scaling it in mesh space

                rwpmath::Vector3 translation = transformMatrix->GetW();

                rw::collision::AABBox bBoxW = allVolRef[totalNumResults + i].bBox;
                //the following code is equivalent to e.g. (bBoxW.Min() - translation) * scale + translation
                bBoxW.Set( bBoxW.Min() * scale + ( 1.0f - scale ) * translation,
                           bBoxW.Max() * scale + ( 1.0f - scale ) * translation );
                
                // The bounding box should be in world space (ie including transformMatrix)
                EATESTAssert(IsSimilar(bBoxW.Min(), volRef[i].bBox.Min(),0.001f), "BBox.Min() should be scaled and properly transformed into world space");
                EATESTAssert(IsSimilar(bBoxW.Max(), volRef[i].bBox.Max(),0.001f), "BBox.Max() should be scaled and properly transformed into world space");
            }

            totalNumResults += numResults;
        }

        EATESTAssert(totalNumResults == numExpectedResults, "Number of results returned from bbox query is incorrect.");

        EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(query);
    }
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(queryAll);
}

void ClusteredMeshTest_Base::BBoxQueryInMappedArrayWithPrimitivesTester(const rw::collision::Volume* meshVolume)
{
    // Create a simple mapped array with some spheres and the clustered mesh
    const uint32_t numVolumes(8);
    SimpleMappedArray* simpleMappedArray(EA::Physics::UnitFramework::Creator<SimpleMappedArray>().New(numVolumes));

    // Initialize the volumes in the simple mapped array
    for (uint32_t i(0); i < simpleMappedArray->GetVolumeCount()-1; ++i)
    {
        rw::collision::SphereVolume::Initialize(&simpleMappedArray->GetVolumeArray()[i], static_cast<float>(i));
    }
    simpleMappedArray->GetVolumeArray()[numVolumes-1] = *meshVolume;

    simpleMappedArray->Update();

    AggregateVolume* volume(EA::Physics::UnitFramework::Creator<AggregateVolume>().New(&*simpleMappedArray));

    const uint32_t STACKSIZE = 5;
    const uint32_t RESBUFFERSIZE = 32;

    // VolumeBBoxQuery object
    VolumeBBoxQuery* p_VBBQ = EA::Physics::UnitFramework::Creator<VolumeBBoxQuery>().New( STACKSIZE, RESBUFFERSIZE );

    rwpmath::Matrix44Affine identityMatrix = rwpmath::GetMatrix44Affine_Identity();
    rw::collision::AABBox volBBox;
    volume->GetBBox(&identityMatrix, TRUE, volBBox);

    const Volume *volArray[1] = { &*volume };
    p_VBBQ->InitQuery(&volArray[0], NULL, 1, volBBox);

    uint32_t numResults = 0;
    while (p_VBBQ->GetOverlaps())
    {
        numResults += p_VBBQ->GetOverlapResultsBufferCount();
    }

    const rw::collision::AggregateVolume *aggVol = static_cast<const rw::collision::AggregateVolume *>(meshVolume);
    rw::collision::ClusteredMesh *mesh = static_cast<rw::collision::ClusteredMesh *>(aggVol->GetAggregate());
    // Check we have all the triangles and each of the primitives
    EATESTAssert(numResults == mesh->GetVolumeCount() + numVolumes - 1, "Number of results returned from bbox query incorrect.");

    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(p_VBBQ);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volume);
    EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(simpleMappedArray);
}
