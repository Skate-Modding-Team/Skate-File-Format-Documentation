// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

File: test-kdtreebuilder.cpp

Purpose: Unit test suite for KDTree building.

*/

// ***********************************************************************************************************
// Includes


#include <rw/math/math.h>
#include <EABase/eabase.h>
#include <eaphysics/base.h>

#include <rw/collision/kdtreebuilder.h>
#include <rw/collision/kdtree.h>

#include <eaphysics/unitframework/allocator.h> // For ResetAllocator
#include <eaphysics/unitframework/creator.h> // For Creator

#include "testsuitebase.h" // For TestSuiteBase

#include "unit/unit.h"
#include "random.hpp"

#include <EASTL/vector.h>

#if !defined(RWP_DISABLE_FILESYSTEM)
#include "unittest_datafile_utilities.hpp"
#include "SimpleStream.hpp"
#include "stdio.h"
#include <string.h>
#endif

#define UNITTEST_MESH_DATAFILE UNITTEST_DATA_FILE("meshdata/")

#define UNITTEST_MESH_GROUPIDS_DATA_FILE ".matIdx"
#define UNITTEST_MESH_POINTINDICES_DATA_FILE ".pointIdx"
#define UNITTEST_MESH_POINTS_DATA_FILE ".points"

namespace test
{

    // ***********************************************************************************************************
    // Test suite


    /// Test suite.
    class TestKDTreeBuilder : public rw::collision::tests::TestSuiteBase
    {
    private:

        struct ConsistencyTolerances{

            ConsistencyTolerances()
                : nonEmptyLeafCount(0)
                , minNonEmptyLeafNodeDepth(0)
                , maxNonEmptyLeafNodeDepth(0)
                , averageNonEmptyLeafNodeDepth(0)
                , emptyLeafCount(0)
                , minEmptyLeafNodeDepth(0)
                , maxEmptyLeafNodeDepth(0)
                , averageEmptyLeafNodeDepth(0)
            {
            }

            uint32_t nonEmptyLeafCount;
            uint32_t minNonEmptyLeafNodeDepth;
            uint32_t maxNonEmptyLeafNodeDepth;
            uint32_t averageNonEmptyLeafNodeDepth;

            uint32_t emptyLeafCount;
            uint32_t minEmptyLeafNodeDepth;
            uint32_t maxEmptyLeafNodeDepth;
            uint32_t averageEmptyLeafNodeDepth;
        };

        virtual void SetupSuite()
        {
            rw::collision::tests::TestSuiteBase::SetupSuite();
        }

        virtual void TeardownSuite()
        {
            EA::Physics::UnitFramework::ResetAllocator();
            rw::collision::tests::TestSuiteBase::TeardownSuite();
        }

        bool IsSimilarValues(uint32_t a, uint32_t b, uint32_t tolerance)
        {
            if (a >= b)
            {
                return (a - b) <= tolerance;
            }
            else
            {
                return (b - a) <= tolerance;
            }
        }

        // Type we're using for vector3's in the tests.
        typedef rw::math::fpu::Vector3U_32 VectorType;

        static const uint32_t MAX_STACK_DEPTH = 100;
        static const uint32_t maxBBoxesPerNode = 63;

        class LineReader
        {
        public:
            LineReader(char * source,
                       uint32_t sourceSize)
                       : m_sourceBuffer(source)
                       , m_sourceSize(sourceSize)
                       , m_current(0)
            {
            }

            LineReader()
                : m_sourceBuffer(NULL)
                , m_sourceSize(0)
                , m_current(0)
            {
            }

            uint32_t ReadLine(char * output, const uint32_t outputSize)
            {
                uint32_t outputCurrent = 0;
                while (m_current < m_sourceSize &&
                       m_sourceBuffer[m_current] != '\n' &&
                       outputCurrent < (outputSize - 1))
                {
                    output[outputCurrent++] = m_sourceBuffer[m_current++];
                }

                if (outputCurrent == (outputSize - 1))
                {
                    output[outputCurrent] = '\n';
                }

                if (m_sourceBuffer[m_current] == '\n')
                {
                    output[outputCurrent] = '\n';
                }

                

                ++m_current;
                return outputCurrent;
            }

            char * m_sourceBuffer;
            uint32_t m_sourceSize;
            uint32_t m_current;
        };

        void InspectKDTree(
            const rw::collision::AABBoxU * /*bboxList*/,
            const uint32_t /*numVolumes*/,
            rw::collision::KDTreeBuilder & builder,
            uint32_t &numNodes,
            uint32_t &numNonEmptyLeafNodes,
            uint32_t &minNonEmptyLeafNodeDepth,
            uint32_t &maxNonEmptyLeafNodeDepth,
            uint32_t &averageNonEmptyLeafNodeDepth,
            uint32_t &numEmptyLeafNodes,
            uint32_t &minEmptyLeafNodeDepth,
            uint32_t &maxEmptyLeafNodeDepth,
            uint32_t &averageEmptyLeafNodeDepth,
            uint32_t &maxLeafNodeSize,
            float &leafNodeSurfaceArea,
            int32_t &balance)
        {
            numNodes = 0;

            numNonEmptyLeafNodes = 0;
            minNonEmptyLeafNodeDepth = rwcKDTREE_MAX_DEPTH;
            maxNonEmptyLeafNodeDepth = 0;
            averageNonEmptyLeafNodeDepth = 0;

            numEmptyLeafNodes = 0;
            minEmptyLeafNodeDepth = rwcKDTREE_MAX_DEPTH;
            maxEmptyLeafNodeDepth = 0;
            averageEmptyLeafNodeDepth = 0;

            maxLeafNodeSize = 0;

            uint32_t sumNonEmptyLeafNodeDepth = 0;
            uint32_t sumEmptyLeadNodeDepth = 0;

            struct NodeStack{
                const rw::collision::KDTreeBuilder::BuildNode * node;
                int32_t balance;
                uint32_t depth;
            };

            NodeStack stack[MAX_STACK_DEPTH];
            uint32_t stackDepth = 0;

            stack[stackDepth].node = builder.GetRootNode();
            stack[stackDepth].balance = 0;
            stack[stackDepth++].depth = 0;

            while (stackDepth > 0)
            {
                EATESTAssert(stackDepth < MAX_STACK_DEPTH-2, "Should have room to add two more nodes");
                // Get this node
                const rw::collision::KDTreeBuilder::BuildNode *const node = stack[--stackDepth].node;
                uint32_t currentDepth = stack[stackDepth].depth;
                int32_t currentBalance = stack[stackDepth].balance;

                const bool leaf = (node->m_left == 0 && node->m_right == 0);

                // Update stats
                ++numNodes;
                if (leaf)
                {
                    rwpmath::Vector3 diagonal(node->m_bbox.Max() - node->m_bbox.Min());
                    leafNodeSurfaceArea += 2.0f *(diagonal.GetX() * diagonal.GetY() + diagonal.GetX() * diagonal.GetZ() + diagonal.GetY() * diagonal.GetZ());

                    if (node->m_numEntries > 0)
                    {
                        minNonEmptyLeafNodeDepth = (minNonEmptyLeafNodeDepth > currentDepth) ? currentDepth : minNonEmptyLeafNodeDepth;
                        maxNonEmptyLeafNodeDepth = (maxNonEmptyLeafNodeDepth < currentDepth) ? currentDepth : maxNonEmptyLeafNodeDepth;
                        sumNonEmptyLeafNodeDepth += currentDepth;
                        ++numNonEmptyLeafNodes;
                        if (currentDepth > maxNonEmptyLeafNodeDepth) maxNonEmptyLeafNodeDepth = currentDepth;
                    }
                    else
                    {
                        minEmptyLeafNodeDepth = (minEmptyLeafNodeDepth > currentDepth) ? currentDepth : minEmptyLeafNodeDepth;
                        maxEmptyLeafNodeDepth = (maxEmptyLeafNodeDepth < currentDepth) ? currentDepth : maxEmptyLeafNodeDepth;
                        sumEmptyLeadNodeDepth += currentDepth;
                        ++numEmptyLeafNodes;
                    }

                    if (node->m_numEntries > maxLeafNodeSize)
                    {
                        maxLeafNodeSize = node->m_numEntries;
                    }
                    balance += currentBalance;
                }

                if (node->m_left)
                {
                    stack[stackDepth].node = node->m_left;
                    stack[stackDepth].balance = currentBalance - 1;
                    stack[stackDepth++].depth = currentDepth + 1;
                }
                if (node->m_right)
                {
                    stack[stackDepth].node = node->m_right;
                    stack[stackDepth].balance = currentBalance + 1;
                    stack[stackDepth++].depth = currentDepth + 1;
                }
            }

            averageNonEmptyLeafNodeDepth = (numNonEmptyLeafNodes) ? sumNonEmptyLeafNodeDepth / numNonEmptyLeafNodes : 0;
            averageEmptyLeafNodeDepth = (numEmptyLeafNodes) ? sumEmptyLeadNodeDepth / numEmptyLeafNodes : 0 ;
            leafNodeSurfaceArea = (numNonEmptyLeafNodes) ? leafNodeSurfaceArea / numNonEmptyLeafNodes : 0 ;
        }

        static const uint32_t MAX_TRIANGLE_COLLECTION_COUNT = 5000u;
        static const uint32_t MAX_POINT_COLLECTION_COUNT = 5000u;

        typedef eastl::vector<rw::math::fpu::Vector3> PointCollection;

        struct TrianglePointIndices
        {
            TrianglePointIndices(){}

            TrianglePointIndices(uint32_t point0,
                uint32_t point1,
                uint32_t point2)
                : p0(point0)
                , p1(point1)
                , p2(point2)
            {

            }
            uint32_t p0;
            uint32_t p1;
            uint32_t p2;
        };

        typedef eastl::vector<TrianglePointIndices> TriangleCollection;

        bool LoadMeshPoints(const char * filename, PointCollection & points)
        {
            char completePath[256];
            sprintf(completePath, UNITTEST_MESH_DATAFILE);
            strcat(completePath, filename);
            strcat(completePath, UNITTEST_MESH_POINTS_DATA_FILE);        

            benchmarkenvironment::FileStream stream;
            if (!stream.Open(completePath))
            {
                return false; // failed to read file
            }

            uint32_t size = stream.GetSize();
            void *data = malloc(size);

            if (size != stream.Read(data, size))
            {
                free(data);
                return false;
            }

            const uint32_t MAX_LINE_CHARS = 128;
            char line[MAX_LINE_CHARS];

            LineReader lineReader(static_cast<char*>(data), size);

            while (lineReader.ReadLine(line, MAX_LINE_CHARS))
            {
                uint32_t vertexIndex(0);
                float x(0);
                float y(0);
                float z(0);

                if (sscanf(line, "%d, %f, %f, %f", &vertexIndex, &x, &y, &z) == 4)
                {
                    points.push_back(rw::math::fpu::Vector3(x, y, z));
                }
                else
                {
                    printf("Failed to scan line.\n");
                    free(data);
                    return false;
                }
            }

            free(data);
            return true;
        }

        bool LoadMeshTriangles(const char *filename, TriangleCollection & triangles, uint32_t & numVolumes)
        {
            char completePath[256];
            sprintf(completePath, UNITTEST_MESH_DATAFILE);
            strcat(completePath, filename);
            strcat(completePath, UNITTEST_MESH_POINTINDICES_DATA_FILE);

            benchmarkenvironment::FileStream stream;
            if (!stream.Open(completePath))
            {
                return false; // failed to read file
            }

            uint32_t size = stream.GetSize();
            void *data = malloc(size);

            if (size != stream.Read(data, size))
            {
                free(data);
                return false;
            }

            const uint32_t MAX_LINE_CHARS = 128;
            char line[MAX_LINE_CHARS];

            LineReader lineReader(static_cast<char*>(data), size);

            while (lineReader.ReadLine(line, MAX_LINE_CHARS))
            {
                uint32_t triangleIndex(0);
                uint32_t p0(0);
                uint32_t p1(0);
                uint32_t p2(0);

                if (sscanf(line, "%d, %d, %d, %d", &triangleIndex, &p0, &p1, &p2) == 4)
                {
                    triangles.push_back(TrianglePointIndices(p0, p1, p2));
                    ++numVolumes;
                }
                else
                {
                    printf("Failed to scan line.\n");
                    free(data);
                    return false;
                }
            }

            free(data);
            return true;
        }

        bool LoadMeshFile(const char * filename, VectorType ** volumeExtents, uint32_t & numVolumes)
        {
            PointCollection points;
            TriangleCollection triangles;                        

            if (!LoadMeshPoints(filename, points))
            {
                return false;
            }            

            // Read the mesh triangles
            if (!LoadMeshTriangles(filename, triangles, numVolumes))
            {
                return false;
            }

            // Allocate the volumeExtents
            *volumeExtents = static_cast<VectorType*>(EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Alloc(sizeof(VectorType) * 2 * numVolumes, NULL, 0, 16));

            GenerateMeshEntryExtents(points, triangles, volumeExtents);

            return true;
        }

        void GenerateMeshEntryExtents(PointCollection & points, TriangleCollection & triangles, VectorType **volumeExtents)
        {
            for (uint32_t triangleIndex = 0 ; triangleIndex < triangles.size() ; ++triangleIndex)
            {
                TrianglePointIndices & triangle = triangles[triangleIndex];
                rw::math::fpu::Vector3 min = rw::math::fpu::Min(points[triangle.p0], points[triangle.p1]);
                min = rw::math::fpu::Min(min, points[triangle.p2]);
                rw::math::fpu::Vector3 max = rw::math::fpu::Max(points[triangle.p0], points[triangle.p1]);
                max = rw::math::fpu::Max(max, points[triangle.p2]);

                (*volumeExtents)[(triangleIndex * 2)] = min;
                (*volumeExtents)[(triangleIndex * 2) + 1] = max;
            }
        }

        void RunTest(
            const uint32_t numVolumes,
            const VectorType volumeExtents[],
            uint32_t splitThreshold,
            float largeItemThreshold,
            const uint32_t expectedLeafNodeSize,
            const uint32_t expectedNonEmptyLeafCount = 0,
            const uint32_t expectedMinNonEmptyLeafNodeDepth = 0,
            const uint32_t expectedMaxNonEmptyLeafNodeDepth = 0,
            const uint32_t expectedAverageNonEmptyLeafNodeDepth = 0,
            const uint32_t expectedEmptyLeafCount = 0,
            const uint32_t expectedMinEmptyLeafNodeDepth = 0,
            const uint32_t expectedMaxEmptyLeafNodeDepth = 0,
            const uint32_t expectedAverageEmptyLeafNodeDepth = 0,
            const ConsistencyTolerances& tolerances = ConsistencyTolerances())
        {
            // Create the input volumes and accumulate the root bbox.
            rw::collision::AABBoxU * bboxList = new rw::collision::AABBoxU[numVolumes];
            EATESTAssert((NULL != bboxList), "Failed to allocate entryAABBox list");

            rw::collision::AABBoxU rootBBox(volumeExtents[0], volumeExtents[1]);

            for (uint32_t i = 0; i < numVolumes; i++)
            {
                bboxList[i] = rw::collision::AABBoxU(volumeExtents[i * 2], volumeExtents[i * 2 + 1]);
                rootBBox = Union(rootBBox, bboxList[i]);
            }

            // Build the KD tree

            rw::collision::KDTreeBuilder builder(*EA::Allocator::ICoreAllocator::GetDefaultAllocator());
            builder.BuildTree(numVolumes, bboxList, splitThreshold, largeItemThreshold);

            EATESTAssert((true == builder.SuccessfulBuild()), "KDTree Build process failed due to memory allocation failure");

            // Gather stats about the built tree.
            uint32_t numNodes = 0;

            uint32_t numNonEmptyLeafNodes = 0;
            uint32_t minNonEmptyLeafNodeDepth = 0;
            uint32_t maxNonEmptyLeafNodeDepth = 0;
            uint32_t averageNonEmptyLeafNodeDepth = 0;

            uint32_t numEmptyLeafNodes = 0;
            uint32_t minEmptyLeafNodeDepth = 0;
            uint32_t maxEmptyLeafNodeDepth = 0;
            uint32_t averageEmptyLeafNodeDepth = 0;

            uint32_t maxLeafNodeSize = 0;

            float leafNodeSurfaceArea = 0;
            int32_t balance = 0;

            InspectKDTree(
                bboxList,
                numVolumes,
                builder,
                numNodes,
                numNonEmptyLeafNodes,
                minNonEmptyLeafNodeDepth,
                maxNonEmptyLeafNodeDepth,
                averageNonEmptyLeafNodeDepth,
                numEmptyLeafNodes,
                minEmptyLeafNodeDepth,
                maxEmptyLeafNodeDepth,
                averageEmptyLeafNodeDepth,
                maxLeafNodeSize,
                leafNodeSurfaceArea,
                balance);

            // For debug purposes
            printf("Num Nodes = %d\n",         numNodes);
            printf("Num Non Empty = %d\n",      numNonEmptyLeafNodes);
            printf("Min Non Empty = %d\n",    minNonEmptyLeafNodeDepth);
            printf("Max Non Empty= %d\n",    maxNonEmptyLeafNodeDepth);
            printf("Ave Non Empty= %d\n",    averageNonEmptyLeafNodeDepth);
            printf("Num Empty = %d\n",      numEmptyLeafNodes);
            printf("Min Empty = %d\n",    minEmptyLeafNodeDepth);
            printf("Max Empty= %d\n",    maxEmptyLeafNodeDepth);
            printf("Ave Empty= %d\n",    averageEmptyLeafNodeDepth);
            printf("total Leaf Node Surface Area = %f\n", leafNodeSurfaceArea);
            printf("balance = %d\n", balance);


            EATESTAssert((maxLeafNodeSize <= expectedLeafNodeSize), "Maximum size of KDTree leaf nodes exceeded expected maximum");
            EATESTAssert((maxLeafNodeSize <= maxBBoxesPerNode), "Maximum BBoxes per node limit breached");

            EATESTAssert((IsSimilarValues(numNonEmptyLeafNodes, expectedNonEmptyLeafCount, tolerances.nonEmptyLeafCount)), "Count of non-empty leaf nodes is not within tolerance of expected count");
            if(expectedNonEmptyLeafCount > 0)
            {
                EATESTAssert((IsSimilarValues(minNonEmptyLeafNodeDepth, expectedMinNonEmptyLeafNodeDepth, tolerances.minNonEmptyLeafNodeDepth)), "Minimum depth of non empty leaf node is not within tolerance of expected depth");
                EATESTAssert((IsSimilarValues(maxNonEmptyLeafNodeDepth, expectedMaxNonEmptyLeafNodeDepth, tolerances.maxNonEmptyLeafNodeDepth)), "Maximum depth of non empty leaf node is not within tolerance of expected depth");
                EATESTAssert((IsSimilarValues(averageNonEmptyLeafNodeDepth, expectedAverageNonEmptyLeafNodeDepth, tolerances.averageNonEmptyLeafNodeDepth)), "Average depth of non empty leaf nodes is not within tolerance of expected depth");
            }

            EATESTAssert((IsSimilarValues(numEmptyLeafNodes, expectedEmptyLeafCount, tolerances.emptyLeafCount)), "Count of empty leaf nodes is not within tolerance of expected count");
            if (expectedEmptyLeafCount > 0)
            {
                EATESTAssert((IsSimilarValues(minEmptyLeafNodeDepth, expectedMinEmptyLeafNodeDepth, tolerances.minEmptyLeafNodeDepth)), "Minimum depth of empty leaf node is not within tolerance of expected depth");
                EATESTAssert((IsSimilarValues(maxEmptyLeafNodeDepth, expectedMaxEmptyLeafNodeDepth, tolerances.maxEmptyLeafNodeDepth)), "Maximum depth of empty leaf node is not within tolerance of expected depth");
                EATESTAssert((IsSimilarValues(averageEmptyLeafNodeDepth, expectedAverageEmptyLeafNodeDepth, tolerances.averageEmptyLeafNodeDepth)), "Average depth of empty leaf nodes is not within tolerance of expected depth");
            }

            // Create a KDTree and validate it
            uint32_t numBranchNodes = builder.GetNumBranchNodes();

            rw::collision::AABBox bbox = builder.GetRootBBox();
            rw::collision::AABBox extents(rwpmath::Vector3(rootBBox.Min()), rwpmath::Vector3(rootBBox.Max()));
            EATESTAssert(bbox.Contains(extents),"KDTree BBox should not be smaller than its contents");

            rw::collision::KDTree * kdtree = EA::Physics::UnitFramework::Creator<rw::collision::KDTree>().New(numBranchNodes, numVolumes, bbox);
            EATESTAssert((NULL != kdtree), "Failed to allocate memory for KDTree");

            builder.InitializeRuntimeKDTree(kdtree);

            EATESTAssert(kdtree->IsValid(), "KDTree produced should be valid");

            delete [] bboxList;
        }

        void Test00()
        {
            const uint32_t numVolumes = 1;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
            };

            ConsistencyTolerances tolerances;

            // Single volume should be fine, producing a "trivial" KDTree with no branch nodes
            RunTest(numVolumes, volumeExtents, 1, 1.0f, 1, 1, 0, 0, 0, 0, 0, 0, 0, tolerances);
        }

        void Test01()
        {
            const uint32_t numVolumes = 2;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                VectorType(0.0f, 0.0f, 0.0f), VectorType(0.25f, 0.25f, 0.25f),
                VectorType(0.75f, 0.75f, 0.75f), VectorType(1.0f, 1.0f, 1.0f),
            };

            ConsistencyTolerances tolerances;

            RunTest(numVolumes, volumeExtents, 1, 1.0f, 1, 2, 1, 1, 1, 0, 0, 0, 0, tolerances);
        }

        void Test02()
        {
            const uint32_t numVolumes = 2;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
                VectorType(0.75f, 0.75f, 0.75f), VectorType(1.0f, 1.0f, 1.0f),
            };

            ConsistencyTolerances tolerances;

            RunTest(numVolumes, volumeExtents, 1, 1.0f, 1, 2, 1, 1, 1, 0, 0, 0, 0, tolerances);
        }

        void Test03()
        {
            // This puppy tests the pathological case where a single large item spans a bunch of smaller
            // ones, preventing them from being broken up. The expectation is that the "alternative"
            // algorithm will come to the rescue and split the items into groups by size instead, with
            // the offending "big" items in one box and the rest in the other.
            const uint32_t numVolumes = 11;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                // big items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),

                // small items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(0.1f, 0.1f, 0.1f),
                VectorType(0.1f, 0.1f, 0.1f), VectorType(0.2f, 0.2f, 0.2f),
                VectorType(0.2f, 0.2f, 0.2f), VectorType(0.3f, 0.3f, 0.3f),
                VectorType(0.3f, 0.3f, 0.3f), VectorType(0.4f, 0.4f, 0.4f),
                VectorType(0.4f, 0.4f, 0.4f), VectorType(0.5f, 0.5f, 0.5f),
                VectorType(0.5f, 0.5f, 0.5f), VectorType(0.6f, 0.6f, 0.6f),
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.7f, 0.7f, 0.7f),
                VectorType(0.7f, 0.7f, 0.7f), VectorType(0.8f, 0.8f, 0.8f),
                VectorType(0.8f, 0.8f, 0.8f), VectorType(0.9f, 0.9f, 0.9f),
                VectorType(0.9f, 0.9f, 0.9f), VectorType(1.0f, 1.0f, 1.0f),
            };

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            RunTest(numVolumes, volumeExtents, 1, 1.0f, 6, 11, 2, 9, 6, 13, 3, 8, 5, tolerances);
            // With a largItemThreshold defined, we expect fewer objects per leaf
            RunTest(numVolumes, volumeExtents, 1, 0.5f, 1, 11, 2, 8, 6, 12, 3, 7, 4, tolerances);
        }

        void Test04()
        {
            // This one tests a pathological case where there are a bunch of "big" items and no "small ones".
            // The fear is that the "alternative" solution will recursively keep trying to break them up into
            // two boxes, one with all the items and the other empty. But this doesn't happen because the
            // proposed solution has cost 1.0 and so is unattractive.
            const uint32_t numVolumes = 4;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                // big items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
            };

            ConsistencyTolerances tolerances;

            RunTest(numVolumes, volumeExtents, 1, 1.0f, 4, 1, 0, 0, 0, 0, 0, 0, 0, tolerances);
            // With a largItemThreshold defined, we expect fewer objects per leaf, but it doesn't deliver
            RunTest(numVolumes, volumeExtents, 1, 0.5f, 4, 1, 0, 0, 0, 0, 0, 0, 0, tolerances);
        }

        void Test05()
        {
            // This tests the case where there are several "big" items, not just one.
            const uint32_t numVolumes = 12;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                // big items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),

                // small items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(0.1f, 0.1f, 0.1f),
                VectorType(0.1f, 0.1f, 0.1f), VectorType(0.2f, 0.2f, 0.2f),
                VectorType(0.2f, 0.2f, 0.2f), VectorType(0.3f, 0.3f, 0.3f),
                VectorType(0.3f, 0.3f, 0.3f), VectorType(0.4f, 0.4f, 0.4f),
                VectorType(0.4f, 0.4f, 0.4f), VectorType(0.5f, 0.5f, 0.5f),
                VectorType(0.5f, 0.5f, 0.5f), VectorType(0.6f, 0.6f, 0.6f),
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.7f, 0.7f, 0.7f),
                VectorType(0.7f, 0.7f, 0.7f), VectorType(0.8f, 0.8f, 0.8f),
                VectorType(0.8f, 0.8f, 0.8f), VectorType(0.9f, 0.9f, 0.9f),
                VectorType(0.9f, 0.9f, 0.9f), VectorType(1.0f, 1.0f, 1.0f),
            };

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            RunTest(numVolumes, volumeExtents, 1, 1.0f, 7, 11, 2, 9, 6, 13, 3, 8, 5, tolerances);
            // With a largItemThreshold defined, we expect fewer objects per leaf
            RunTest(numVolumes, volumeExtents, 1, 0.5f, 2, 11, 2, 8, 6, 12, 3, 7, 4, tolerances);
        }

        void Test06()
        {
            // This tests the case where there is a "big" item and two distinct groups of "small" items
            // that will, after breakup, themselves become sets of only "big" items. These can't be broken
            // up and will remain.
            const uint32_t numVolumes = 7;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                // big items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),

                // one group of small items, all the same size
                VectorType(0.2f, 0.2f, 0.2f), VectorType(0.4f, 0.4f, 0.4f),
                VectorType(0.2f, 0.2f, 0.2f), VectorType(0.4f, 0.4f, 0.4f),
                VectorType(0.2f, 0.2f, 0.2f), VectorType(0.4f, 0.4f, 0.4f),

                // a second group of small items, all the same size
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.8f, 0.8f, 0.8f),
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.8f, 0.8f, 0.8f),
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.8f, 0.8f, 0.8f),
            };

            ConsistencyTolerances tolerances;

            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            RunTest(numVolumes, volumeExtents, 1, 1.0f, 4, 7, 2, 7, 5, 5, 2, 5, 3, tolerances);
            // With a largItemThreshold defined, we expect fewer objects per leaf
            RunTest(numVolumes, volumeExtents, 1, 0.5f, 3, 7, 2, 6, 4, 4, 2, 4, 3, tolerances);
        }

        void Test07()
        {
            /// This tests the consistency of the number of leafNodes in the KDTree.
            /// At the time of writing the test values were generated, by the
            /// code under testing.

            /// A regular cube grid of unit cubes, 30x30x30.

            const uint32_t xCount = 30;
            const uint32_t yCount = 30;
            const uint32_t zCount = 30;
            const uint32_t numVolumes = xCount * yCount * zCount;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            struct volumeExtentPairs{
                VectorType min;
                VectorType max;
            };

            volumeExtentPairs* currentPair = reinterpret_cast<volumeExtentPairs*>(&volumeExtents[0]);
            for (uint32_t z = 0 ; z < zCount ; ++z)
            {
                for (uint32_t y = 0 ; y < yCount ; ++y)
                {
                    for (uint32_t x = 0 ; x < xCount ; ++x)
                    {
                        currentPair->min = VectorType(x * 1.0f, y * 1.0f, z * 1.0f);
                        currentPair->max = VectorType((x + 1) * 1.0f, (y + 1) * 1.0f, (z + 1) * 1.0f);
                        currentPair++;
                    }
                }
            }

            ConsistencyTolerances tolerances;

            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 1;
            tolerances.maxNonEmptyLeafNodeDepth = 1;
            tolerances.averageNonEmptyLeafNodeDepth = 1;

            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                4,              // Expected leaf node size
                6832,           // Expected non empty leaf count
                11,             // Expected min non empty leaf node depth
                13,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            delete [] volumeExtents;
        }

        void Test08()
        {
            /// This tests the consistency of the number of leafNodes in the KDTree.
            /// At the time of writing the test values were generated, by the
            // code under testing.

            /// A regular cube grid of unit cubes, 32x32x32. Since this test uses a
            /// regular grid of 32 we can expect an exact result each time, with or 
            /// without largeitem threshold feature enabled. Therefore tolerances are
            /// set to zero.


            const uint32_t xCount = 32;
            const uint32_t yCount = 32;
            const uint32_t zCount = 32;
            const uint32_t numVolumes = xCount * yCount * zCount;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            struct volumeExtentPairs{
                VectorType min;
                VectorType max;
            };

            volumeExtentPairs* currentPair = reinterpret_cast<volumeExtentPairs*>(&volumeExtents[0]);
            for (uint32_t z = 0 ; z < zCount ; ++z)
            {
                for (uint32_t y = 0 ; y < yCount ; ++y)
                {
                    for (uint32_t x = 0 ; x < xCount ; ++x)
                    {
                        currentPair->min = VectorType(x * 1.0f, y * 1.0f, z * 1.0f);
                        currentPair->max = VectorType((x + 1) * 1.0f, (y + 1) * 1.0f, (z + 1) * 1.0f);
                        currentPair++;
                    }
                }
            }

            ConsistencyTolerances tolerances;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                8,              // Split threshold
                1.0f,           // Large item threshold
                8,              // Expected leaf node size
                4096,           // Expected non empty leaf count
                12,             // Expected min non empty leaf node depth
                12,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                8,              // Split threshold
                0.8f,           // Large item threshold
                8,              // Expected leaf node size
                4096,           // Expected non empty leaf count
                12,             // Expected min non empty leaf node depth
                12,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            delete [] volumeExtents;
        }

        void Test09()
        {
            /// This tests the consistency of the number of leafNodes in the KDTree.
            /// At the time of writing the test values were generated, by the
            // code under testing.

            /// A group of randomly sized and distributed boxes.

            rw::math::SeedRandom(9u);

            const float length = 1.0f;
            const uint32_t numVolumes = 10000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[volumeIndex * 2] = VectorType(RandomVector3(100.0f));
                volumeExtents[(volumeIndex * 2) + 1] = volumeExtents[volumeIndex * 2];

                volumeExtents[volumeIndex * 2]-= VectorType(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
                volumeExtents[(volumeIndex * 2) + 1]+= VectorType(Random(length / 2.0f, length), Random(length / 2.0f, length), Random(length / 2.0f, length));
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 1;
            tolerances.maxNonEmptyLeafNodeDepth = 1;
            tolerances.averageNonEmptyLeafNodeDepth = 1;
            tolerances.emptyLeafCount = 1;
            tolerances.minEmptyLeafNodeDepth = 1;
            tolerances.maxEmptyLeafNodeDepth = 1;
            tolerances.averageEmptyLeafNodeDepth = 1;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                8,              // Split threshold
                1.0f,           // Large item threshold
                8,              // Expected leaf node size
                1768,           // Expected non empty leaf count
                10,             // Expected min non empty leaf node depth
                13,             // Expected max non empty leaf node depth
                10,             // Expected average non empty leaf node depth
                2,              // Expected empty leaf count
                11,              // Expected min empty leaf node depth
                12,              // Expected max empty leaf node depth
                11,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                8,              // Split threshold
                0.8f,           // Large item threshold
                8,              // Expected leaf node size
                1768,           // Expected non empty leaf count
                10,             // Expected min non empty leaf node depth
                13,             // Expected max non empty leaf node depth
                10,             // Expected average non empty leaf node depth
                2,              // Expected empty leaf count
                11,              // Expected min empty leaf node depth
                12,              // Expected max empty leaf node depth
                11,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            delete [] volumeExtents;
        }


        // New silly crazy pathological tests

        void TestPath01()
        {
            // This tests the case where there is a bunch of large item and only a couple small item
            // Ideally the smaller objects should get picked out eventually, old should just give up
            const uint32_t numVolumes = 11;
            const VectorType volumeExtents[numVolumes * 2] =
            {
                // big items
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),            
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),  
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),            
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),  
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),            
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),  
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),            
                VectorType(0.0f, 0.0f, 0.0f), VectorType(1.0f, 1.0f, 1.0f),  
                // a second group of small items
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.8f, 0.8f, 0.8f),
                VectorType(0.2f, 0.2f, 0.2f), VectorType(0.3f, 0.3f, 0.3f),
                VectorType(0.6f, 0.6f, 0.6f), VectorType(0.7f, 0.7f, 0.7f),
            };

            ConsistencyTolerances tolerances;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                1,              // Split threshold
                1.0f,           // Large item threshold
                9,              // Expected leaf node size
                5,           // Expected non empty leaf count
                2,             // Expected min non empty leaf node depth
                4,             // Expected max non empty leaf node depth
                3,             // Expected average non empty leaf node depth
                2,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                3,              // Expected max empty leaf node depth
                2,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                1,              // Split threshold
                0.8f,           // Large item threshold
                8,              // Expected leaf node size
                4,           // Expected non empty leaf count
                2,             // Expected min non empty leaf node depth
                4,             // Expected max non empty leaf node depth
                3,             // Expected average non empty leaf node depth
                2,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                3,              // Expected max empty leaf node depth
                2,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath02()
        {
            // This tests the case where there are 3000 identical items in the same position
            // Just to check that the vertex limit per node isn't overflowed

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            for (uint32_t i(0); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[(i * 2) + 1] = VectorType(1.0f, 1.0f, 1.0f);
            }    

            ConsistencyTolerances tolerances;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                84,              // Expected leaf node size
                84,           // Expected non empty leaf count
                4,             // Expected min non empty leaf node depth
                11,             // Expected max non empty leaf node depth
                7,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                3000,              // Expected leaf node size
                84,           // Expected non empty leaf count
                4,             // Expected min non empty leaf node depth
                11,             // Expected max non empty leaf node depth
                7,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }
        void TestPath02b()
        {
            // This tests the case where there are 500 items all same size in the same position
            // Just to check that the vertex limit per node isn't overflowed

            const uint32_t numVolumes = 500;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            for (uint32_t i(0); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[(i * 2) + 1] = VectorType(1.0f, 1.0f, 1.0f);
            }    

            ConsistencyTolerances tolerances;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                500,              // Expected leaf node size
                13,           // Expected non empty leaf count
                2,             // Expected min non empty leaf node depth
                6,             // Expected max non empty leaf node depth
                4,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                500,              // Expected leaf node size
                13,           // Expected non empty leaf count
                2,             // Expected min non empty leaf node depth
                6,             // Expected max non empty leaf node depth
                4,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath02c()
        {
            // This tests the case where there are 3000 items of same size in a similar position as there is
            // a random variance of the min extent
            // Just to check that the vertex limit per node isn't overflowed

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.8f;
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 80;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                404,              // Expected leaf node size
                910,           // Expected non empty leaf count
                8,             // Expected min non empty leaf node depth
                14,             // Expected max non empty leaf node depth
                10,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                404,              // Expected leaf node size
                910,           // Expected non empty leaf count
                8,             // Expected min non empty leaf node depth
                14,             // Expected max non empty leaf node depth
                10,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath02d()
        {
            // This tests the case where there are 3000 items of random size in a similar position as there is
            // a random variance of the min extent
            // Just to check that the vertex limit per node isn't overflowed

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + Random(0.3f, 0.8f);
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 80;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 7;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;


            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                24,              // Expected leaf node size
                1122,           // Expected non empty leaf count
                5,             // Expected min non empty leaf node depth
                25,             // Expected max non empty leaf node depth
                15,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                12,              // Expected leaf node size
                1118,           // Expected non empty leaf count
                5,             // Expected min non empty leaf node depth
                25,             // Expected max non empty leaf node depth
                15,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath03()
        {
            // This tests the case where there are 1500 identical large items in the same position and
            // 1500 identical small items (1/10 size of large item) all in the same pos too
            // Just to check that the vertex limit per node isn't overflowed and the large items are separated
            // from the small ones efficiently

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes/2); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[(i * 2) + 1] = VectorType(1.0f, 1.0f, 1.0f);
            }
            // small items
            for (uint32_t i(numVolumes/2); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[(i * 2) + 1] = VectorType(0.1f, 0.1f, 0.1f);
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 9;
            tolerances.averageEmptyLeafNodeDepth = 9;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                3000,              // Expected leaf node size
                82,           // Expected non empty leaf count
                4,             // Expected min non empty leaf node depth
                13,             // Expected max non empty leaf node depth
                8,             // Expected average non empty leaf node depth
                3,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                4,              // Expected max empty leaf node depth
                3,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                1500,              // Expected leaf node size
                82,           // Expected non empty leaf count
                4,             // Expected min non empty leaf node depth
                12,             // Expected max non empty leaf node depth
                7,             // Expected average non empty leaf node depth
                2,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                3,              // Expected max empty leaf node depth
                2,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath03b()
        {
            // This tests the case where there are 1500 large items in the same position and
            // 1500 small items (1/10 size of large item) all in the same pos too but
            // there is a small variance in the size of all the objects
            // Just to check that the vertex limit per node isn't overflowed and the large items are separated
            // from the small ones efficiently

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes/2); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[(i * 2) + 1] = VectorType(0.8f, 0.8f, 0.8f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
            }
            // small items
            for (uint32_t i(numVolumes/2); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[(i * 2) + 1] = VectorType(0.3f, 0.3f, 0.3f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 80;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 9;
            tolerances.averageEmptyLeafNodeDepth = 9;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                3000,              // Expected leaf node size
                1079,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                25,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                3,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                4,              // Expected max empty leaf node depth
                3,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                3000,              // Expected leaf node size
                1072,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                25,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                3,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                4,              // Expected max empty leaf node depth
                3,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath03c()
        {
            // This tests the case where there are 1500 large items and 1500 small items (1/10 size of large item) but
            // there is a small variance in the position of the objects
            // Just to check that the vertex limit per node isn't overflowed and the large items are separated
            // from the small ones efficiently

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes/2); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.8f;
            }
            // small items
            for (uint32_t i(numVolumes/2); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.2f;

            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 80;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 15;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                1681,              // Expected leaf node size
                969,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                17,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                6,              // Expected empty leaf count
                5,              // Expected min empty leaf node depth
                8,              // Expected max empty leaf node depth
                6,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                202,              // Expected leaf node size
                970,           // Expected non empty leaf count
                7,             // Expected min non empty leaf node depth
                17,             // Expected max non empty leaf node depth
                12,             // Expected average non empty leaf node depth
                2,              // Expected empty leaf count
                5,              // Expected min empty leaf node depth
                6,              // Expected max empty leaf node depth
                5,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath03d()
        {
            // This tests the case where there are 1500 large items and 1500 small items (1/10 size of large item) but
            // there is a small variance in the size and position of the objects
            // Just to check that the vertex limit per node isn't overflowed and the large items are separated
            // from the small ones efficiently

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes/2); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.6f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
            }
            // small items
            for (uint32_t i(numVolumes/2); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.1f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                2091,              // Expected leaf node size
                1072,           // Expected non empty leaf count
                5,             // Expected min non empty leaf node depth
                26,             // Expected max non empty leaf node depth
                13,             // Expected average non empty leaf node depth
                9,              // Expected empty leaf count
                6,              // Expected min empty leaf node depth
                17,              // Expected max empty leaf node depth
                10,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                2091,              // Expected leaf node size
                1074,           // Expected non empty leaf count
                5,             // Expected min non empty leaf node depth
                26,             // Expected max non empty leaf node depth
                13,             // Expected average non empty leaf node depth
                9,              // Expected empty leaf count
                6,              // Expected min empty leaf node depth
                17,              // Expected max empty leaf node depth
                11,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath03e()
        {
            // This tests the case where there are 1500 large items and 1500 small items (1/10 size of large item) but
            // there is a small variance in the size and position of the large objects and a small variation
            // of the size of the small objects but more of a random variation in the position
            // Just to check that the vertex limit per node isn't overflowed and the large items are separated
            // from the small ones efficiently

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes/2); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.6f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
            }
            // small items
            for (uint32_t i(numVolumes/2); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[i * 2]+= VectorType(Random(0.0f, 0.8f), Random(0.0f, 0.8f), Random(0.0f, 0.8f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.1f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                798,              // Expected leaf node size
                1061,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                18,             // Expected max non empty leaf node depth
                11,             // Expected average non empty leaf node depth
                71,              // Expected empty leaf count
                9,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                12,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                798,              // Expected leaf node size
                1066,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                18,             // Expected max non empty leaf node depth
                11,             // Expected average non empty leaf node depth
                70,              // Expected empty leaf count
                9,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                12,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath04()
        {
            // Creates a cube of overlapping BBoxes
            const uint32_t numVolumes = 729;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            uint32_t i(0);
            for (uint32_t x(0); x < 9; x++)
            {
                for (uint32_t y(0); y < 9; y++)
                {
                    for (uint32_t z(0); z < 9; z++, i++)
                    {
                        volumeExtents[i * 2] = VectorType(0.1f*x, 0.1f*y, 0.1f*z);
                        volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.2f;
                    }
                }
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                40,              // Expected leaf node size
                188,           // Expected non empty leaf count
                7,             // Expected min non empty leaf node depth
                10,             // Expected max non empty leaf node depth
                7,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                40,              // Expected leaf node size
                188,           // Expected non empty leaf count
                7,             // Expected min non empty leaf node depth
                10,             // Expected max non empty leaf node depth
                7,             // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath05()
        {
            // This tests the case where there are 1000 large items, 1000 medium items (1/3 size of large item) and
            // 1000 small items (1/3 size of medium item) but there
            // there is a small variance in the size and position of the objects
            // Just to check that the vertex limit per node isn't overflowed and the large items are separated
            // from the small ones efficiently

            rw::math::SeedRandom(9u);

            const uint32_t numVolumes = 3000;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];
            // large items
            for (uint32_t i(0); i < (numVolumes/3); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.2f, 0.2f, 0.2f);
                volumeExtents[i * 2]-= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.6f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.2f), Random(0.0f, 0.2f), Random(0.0f, 0.2f));
            }
            // medium items
            for (uint32_t i(numVolumes/3); i < ((2*numVolumes)/3); i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[i * 2]+= VectorType(Random(0.0f, 0.6f), Random(0.0f, 0.6f), Random(0.0f, 0.6f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.3f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
            }
            // small items
            for (uint32_t i((2*numVolumes)/3); i < numVolumes; i++)
            {         
                volumeExtents[i * 2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[i * 2]+= VectorType(Random(0.0f, 0.8f), Random(0.0f, 0.8f), Random(0.0f, 0.8f));
                volumeExtents[(i * 2) + 1] = volumeExtents[i * 2] + 0.1f;
                volumeExtents[(i * 2) + 1]+= VectorType(Random(0.0f, 0.1f), Random(0.0f, 0.1f), Random(0.0f, 0.1f));
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                680,              // Expected leaf node size
                1075,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                17,             // Expected max non empty leaf node depth
                11,             // Expected average non empty leaf node depth
                7,              // Expected empty leaf count
                10,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                12,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            // With large item feature enabled
            RunTest(numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                680,              // Expected leaf node size
                1074,           // Expected non empty leaf count
                6,             // Expected min non empty leaf node depth
                17,             // Expected max non empty leaf node depth
                11,             // Expected average non empty leaf node depth
                7,              // Expected empty leaf count
                10,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                12,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );        
        }

        void TestPath06()
        {
            // Testing a triangle fan, which spans PI/2.
            // Common case in practice, as has been observed in a number actual game assets.

            const uint32_t numVolumes = 12;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float length = 10.0f;
            const float angleStep = rwpmath::GetVecFloat_Pi() / rwpmath::VecFloat(2.0f * static_cast<float>(numVolumes));
            float angle = 0.0f;

            for (uint32_t volumeIndex =0; volumeIndex < numVolumes; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(0.0f, 0.0f, 0.0f);

                float x = rwpmath::Cos(angle) * length;
                angle += angleStep;
                float z = rwpmath::Sin(angle) * length;

                volumeExtents[(volumeIndex * 2) + 1] = VectorType(x, 0.0f, z);
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                1,              // Split threshold
                1.0f,           // Large item threshold
                32,             // Expected leaf node size
                12,              // Expected non empty leaf count
                2,              // Expected min non empty leaf node depth
                7,              // Expected max non empty leaf node depth
                4,              // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );     

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                1,              // Split threshold
                0.8f,           // Large item threshold
                5,              // Expected leaf node size
                11,              // Expected non empty leaf count
                2,              // Expected min non empty leaf node depth
                6,              // Expected max non empty leaf node depth
                4,              // Expected average non empty leaf node depth
                0,              // Expected empty leaf count
                0,              // Expected min empty leaf node depth
                0,              // Expected max empty leaf node depth
                0,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );     
        }

        void TestPath07()
        {
            // Testing a small number of Large items surrounding by a large number of small items.
            // Common case in practice, as has been observed in a number actual game assets.

            const uint32_t numSmallItemsAlongEachSide = 80;
            const uint32_t numVolumes = numSmallItemsAlongEachSide * 4 + 2;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float length = 10.0f;

            // LargeItems
            {
                volumeExtents[0] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[1] = VectorType(10.0f, 1.0f, 10.0f);

                volumeExtents[2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[3] = VectorType(10.0f, 1.0f, 10.0f);
            }

            // SmallItems
            {
                uint32_t volumeIndex = 2;
                float smallItemLength = length / numSmallItemsAlongEachSide;

                // Along "-x side"
                float xBase = 0.0f;
                float zBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase, 0.0f, zBase + (smallItemLength * (i-volumeIndex)));
                    volumeExtents[i * 2 + 1] = VectorType(xBase + smallItemLength, 1.0f, zBase + (smallItemLength * ((i-volumeIndex) + 1)));
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "+x side"
                xBase = 10.0f - smallItemLength;
                zBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase, 0.0f, zBase + (smallItemLength * (i-volumeIndex)));
                    volumeExtents[i * 2 + 1] = VectorType(xBase + smallItemLength, 1.0f, zBase + (smallItemLength * ((i-volumeIndex) + 1)));
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "-z side"
                xBase = 0.0f;
                zBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase + (smallItemLength * (i-volumeIndex)), 0.0f, zBase);
                    volumeExtents[i * 2 + 1] = VectorType(xBase + (smallItemLength * ((i-volumeIndex) + 1)), 1.0f, zBase + smallItemLength);
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "+z side"
                xBase = 0.0f;
                zBase = 10.0f - smallItemLength;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase + (smallItemLength * (i-volumeIndex)), 0.0f, zBase);
                    volumeExtents[i * 2 + 1] = VectorType(xBase + (smallItemLength * ((i-volumeIndex) + 1)), 1.0f, zBase + smallItemLength);
                }
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 5;
            tolerances.minNonEmptyLeafNodeDepth = 2;
            tolerances.maxNonEmptyLeafNodeDepth = 2;
            tolerances.averageNonEmptyLeafNodeDepth = 2;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 2;
            tolerances.maxEmptyLeafNodeDepth = 2;
            tolerances.averageEmptyLeafNodeDepth = 2;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                82,             // Expected leaf node size
                125,             // Expected non empty leaf count
                5,              // Expected min non empty leaf node depth
                12,             // Expected max non empty leaf node depth
                8,              // Expected average non empty leaf node depth
                26,             // Expected empty leaf count
                3,              // Expected min empty leaf node depth
                10,             // Expected max empty leaf node depth
                6,              // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,              // Expected leaf node size
                129,            // Expected non empty leaf count
                3,              // Expected min non empty leaf node depth
                14,             // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                26,             // Expected empty leaf count
                3,              // Expected min empty leaf node depth
                11,             // Expected max empty leaf node depth
                6,              // Expected average empty leaf node depth
                tolerances);
        }

        void TestPath07b()
        {
            // Testing a small number of Large items surrounding by a large number of small items.
            // Common case in practice, as has been observed in a number actual game assets.

            const uint32_t numSmallItemsAlongEachSide = 80;
            const uint32_t numVolumes = numSmallItemsAlongEachSide * 4 + 2;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float length = 10.0f;

            // LargeItems
            {
                volumeExtents[0] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[1] = VectorType(1.0f, 10.0f, 10.0f);

                volumeExtents[2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[3] = VectorType(1.0f, 10.0f, 10.0f);
            }

            // SmallItems
            {
                uint32_t volumeIndex = 2;
                float smallItemLength = length / numSmallItemsAlongEachSide;

                // Along "-y side"
                float yBase = 0.0f;
                float zBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(0.0f, yBase, zBase + (smallItemLength * (i-volumeIndex)));
                    volumeExtents[i * 2 + 1] = VectorType(1.0f, yBase + smallItemLength, zBase + (smallItemLength * ((i-volumeIndex) + 1)));
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "+y side"
                yBase = 10.0f - smallItemLength;
                zBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(0.0f, yBase, zBase + (smallItemLength * (i-volumeIndex)));
                    volumeExtents[i * 2 + 1] = VectorType(1.0f, yBase + smallItemLength, zBase + (smallItemLength * ((i-volumeIndex) + 1)));
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "-z side"
                yBase = 0.0f;
                zBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(0.0f, yBase + (smallItemLength * (i-volumeIndex)), zBase);
                    volumeExtents[i * 2 + 1] = VectorType(1.0f, yBase + (smallItemLength * ((i-volumeIndex) + 1)), zBase + smallItemLength);
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "+z side"
                yBase = 0.0f;
                zBase = 10.0f - smallItemLength;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(0.0f, yBase + (smallItemLength * (i-volumeIndex)), zBase);
                    volumeExtents[i * 2 + 1] = VectorType(1.0f, yBase + (smallItemLength * ((i-volumeIndex) + 1)), zBase + smallItemLength);
                }
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 5;
            tolerances.minNonEmptyLeafNodeDepth = 2;
            tolerances.maxNonEmptyLeafNodeDepth = 2;
            tolerances.averageNonEmptyLeafNodeDepth = 2;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 2;
            tolerances.maxEmptyLeafNodeDepth = 2;
            tolerances.averageEmptyLeafNodeDepth = 2;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                82,             // Expected leaf node size
                125,             // Expected non empty leaf count
                5,              // Expected min non empty leaf node depth
                12,             // Expected max non empty leaf node depth
                8,              // Expected average non empty leaf node depth
                26,             // Expected empty leaf count
                3,              // Expected min empty leaf node depth
                10,             // Expected max empty leaf node depth
                6,              // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,              // Expected leaf node size
                129,            // Expected non empty leaf count
                3,              // Expected min non empty leaf node depth
                14,             // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                26,             // Expected empty leaf count
                3,              // Expected min empty leaf node depth
                11,             // Expected max empty leaf node depth
                6,              // Expected average empty leaf node depth
                tolerances);
        }

        void TestPath07c()
        {
            // Testing a small number of Large items surrounding by a large number of small items.
            // Common case in practice, as has been observed in a number actual game assets.

            const uint32_t numSmallItemsAlongEachSide = 80;
            const uint32_t numVolumes = numSmallItemsAlongEachSide * 4 + 2;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float length = 10.0f;

            // LargeItems
            {
                volumeExtents[0] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[1] = VectorType(10.0f, 10.0f, 1.0f);

                volumeExtents[2] = VectorType(0.0f, 0.0f, 0.0f);
                volumeExtents[3] = VectorType(10.0f, 10.0f, 1.0f);
            }

            // SmallItems
            {
                uint32_t volumeIndex = 2;
                float smallItemLength = length / numSmallItemsAlongEachSide;

                // Along "-x side"
                float xBase = 0.0f;
                float yBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase, yBase + (smallItemLength * (i-volumeIndex)), 0.0f);
                    volumeExtents[i * 2 + 1] = VectorType(xBase + smallItemLength, yBase + (smallItemLength * ((i-volumeIndex) + 1)), 1.0f);
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "+x side"
                xBase = 10.0f - smallItemLength;
                yBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase, yBase + (smallItemLength * (i-volumeIndex)), 0.0f);
                    volumeExtents[i * 2 + 1] = VectorType(xBase + smallItemLength, yBase + (smallItemLength * ((i-volumeIndex) + 1)), 1.0f);
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "-z side"
                xBase = 0.0f;
                yBase = 0.0f;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase + (smallItemLength * (i-volumeIndex)), yBase, 0.0f);
                    volumeExtents[i * 2 + 1] = VectorType(xBase + (smallItemLength * ((i-volumeIndex) + 1)), yBase + smallItemLength, 1.0f);
                }
                volumeIndex += numSmallItemsAlongEachSide;

                // Along "+z side"
                xBase = 0.0f;
                yBase = 10.0f - smallItemLength;
                for (uint32_t i = volumeIndex ; i < volumeIndex + numSmallItemsAlongEachSide ; ++i)
                {
                    volumeExtents[i * 2] = VectorType(xBase + (smallItemLength * (i-volumeIndex)), yBase, 0.0f);
                    volumeExtents[i * 2 + 1] = VectorType(xBase + (smallItemLength * ((i-volumeIndex) + 1)), yBase + smallItemLength, 1.0f);
                }
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 5;
            tolerances.minNonEmptyLeafNodeDepth = 2;
            tolerances.maxNonEmptyLeafNodeDepth = 2;
            tolerances.averageNonEmptyLeafNodeDepth = 2;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 2;
            tolerances.maxEmptyLeafNodeDepth = 2;
            tolerances.averageEmptyLeafNodeDepth = 2;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                82,             // Expected leaf node size
                125,             // Expected non empty leaf count
                5,              // Expected min non empty leaf node depth
                12,             // Expected max non empty leaf node depth
                8,              // Expected average non empty leaf node depth
                26,             // Expected empty leaf count
                3,              // Expected min empty leaf node depth
                10,             // Expected max empty leaf node depth
                6,              // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,              // Expected leaf node size
                129,            // Expected non empty leaf count
                3,              // Expected min non empty leaf node depth
                14,             // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                26,             // Expected empty leaf count
                3,              // Expected min empty leaf node depth
                11,             // Expected max empty leaf node depth
                6,               // Expected average empty leaf node depth
                tolerances);
        }

        void TestPath08()
        {
            // Testing series of increasing small boxes.
            // At each stage of the build process only one item can be considered large.
            // The test aims to cause the large item approach to split one large item per level.

            const uint32_t numVolumes = 40;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float InitialLength = 200.0f;
            float width = InitialLength * 2;
            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(-InitialLength, 0.0f, -InitialLength);
                volumeExtents[(volumeIndex * 2) + 1] = VectorType(-InitialLength + width, 0.0f, InitialLength);
                width *= 0.77f;
            }

            ConsistencyTolerances tolerances;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                40,             // Expected leaf node size
                14,              // Expected non empty leaf count
                3,              // Expected min non empty leaf node depth
                11,              // Expected max non empty leaf node depth
                6,              // Expected average non empty leaf node depth
                7,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                10,              // Expected max empty leaf node depth
                5               // Expected average empty leaf node depth
                );

            // TEST FAILS - BUILDER GENERATES AN INVALID KDTREE

            // With large item feature enabled
            //RunTest(
            //    numVolumes,     // Entry count
            //    volumeExtents,  // Entry extents
            //    4,              // Split threshold
            //    0.8f,           // Large item threshold
            //    28,             // Expected leaf node size
            //    33,             // Expected non empty leaf count
            //    1,              // Expected min non empty leaf node depth
            //    32,             // Expected max non empty leaf node depth
            //    16,             // Expected average non empty leaf node depth
            //    0,              // Expected empty leaf count
            //    0,              // Expected min empty leaf node depth
            //    0,              // Expected max empty leaf node depth
            //    0               // Expected average empty leaf node depth
            //    );
        }

        void TestPath08b()
        {
            // Testing series of increasing small items centered around the same point.
            // At each stage of the build process only one item can be considered large.
            // The test aims to cause the large item approach to split one large item per level.

            const uint32_t numVolumes = 40;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float InitialLength = 200.0f;
            float width = InitialLength * 2;
            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(-InitialLength, -InitialLength, 0.0f);
                volumeExtents[(volumeIndex * 2) + 1] = VectorType(InitialLength, -InitialLength + width, 0.0f);
                width *= 0.77f;
            }

            ConsistencyTolerances tolerances;


            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                40,             // Expected leaf node size
                14,              // Expected non empty leaf count
                3,              // Expected min non empty leaf node depth
                11,              // Expected max non empty leaf node depth
                6,              // Expected average non empty leaf node depth
                7,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                10,              // Expected max empty leaf node depth
                5               // Expected average empty leaf node depth
                );

            // TEST FAILS - BUILDER GENERATES AN INVALID KDTREE

            // With large item feature enabled
            //RunTest(
            //    numVolumes,     // Entry count
            //    volumeExtents,  // Entry extents
            //    4,              // Split threshold
            //    0.8f,           // Large item threshold
            //    28,             // Expected leaf node size
            //    33,             // Expected non empty leaf count
            //    1,              // Expected min non empty leaf node depth
            //    32,             // Expected max non empty leaf node depth
            //    16,             // Expected average non empty leaf node depth
            //    0,              // Expected empty leaf count
            //    0,              // Expected min empty leaf node depth
            //    0,              // Expected max empty leaf node depth
            //    0               // Expected average empty leaf node depth
            //    );
        }

        void TestPath08c()
        {
            // Testing series of increasing small items centered around the same point.
            // At each stage of the build process only one item can be considered large.
            // The test aims to cause the large item approach to split one large item per level.

            const uint32_t numVolumes = 40;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            const float InitialLength = 200.0f;
            float width = InitialLength * 2;
            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(0.0f, -InitialLength, -InitialLength);
                volumeExtents[(volumeIndex * 2) + 1] = VectorType(0.0f, InitialLength, -InitialLength + width);
                width *= 0.77f;
            }

            ConsistencyTolerances tolerances;


            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                40,             // Expected leaf node size
                14,              // Expected non empty leaf count
                3,              // Expected min non empty leaf node depth
                11,              // Expected max non empty leaf node depth
                6,              // Expected average non empty leaf node depth
                7,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                10,              // Expected max empty leaf node depth
                5               // Expected average empty leaf node depth
                );

            // TEST FAILS - BUILDER GENERATES AN INVALID KDTREE

            // With large item feature enabled
            //RunTest(
            //    numVolumes,     // Entry count
            //    volumeExtents,  // Entry extents
            //    4,              // Split threshold
            //    0.8f,           // Large item threshold
            //    28,             // Expected leaf node size
            //    33,             // Expected non empty leaf count
            //    1,              // Expected min non empty leaf node depth
            //    32,             // Expected max non empty leaf node depth
            //    16,             // Expected average non empty leaf node depth
            //    0,              // Expected empty leaf count
            //    0,              // Expected min empty leaf node depth
            //    0,              // Expected max empty leaf node depth
            //    0               // Expected average empty leaf node depth
            //    );
        }


        void TestPath09()
        {
            // Testing series of increasing small items centered around the same point.
            // At each stage of the build process only one item can be considered large.
            // The test aims to cause the large item approach to split one large item per level.

            const uint32_t numVolumes = 40;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            float length = 100.0f;
            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(-length, 0.0f, -length);
                volumeExtents[(volumeIndex * 2) + 1] = VectorType(length, 0.0f, length);
                length *= 0.77f;
            }

            ConsistencyTolerances tolerances;

            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                63,             // Expected leaf node size
                17,              // Expected non empty leaf count
                2,              // Expected min non empty leaf node depth
                17,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                14,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                17,              // Expected max empty leaf node depth
                8,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                38,             // Expected leaf node size
                22,              // Expected non empty leaf count
                1,              // Expected min non empty leaf node depth
                27,              // Expected max non empty leaf node depth
                13,              // Expected average non empty leaf node depth
                13,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                23,              // Expected max empty leaf node depth
                11,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   
        }

        void TestPath09b()
        {
            // Testing series of increasing small items centered around the same point.
            // At each stage of the build process only one item can be considered large.
            // The test aims to cause the large item approach to split one large item per level.

            const uint32_t numVolumes = 40;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            float length = 100.0f;
            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(-length, -length, 0.0f);
                volumeExtents[(volumeIndex * 2) + 1] = VectorType(length, length, 0.0f);
                length *= 0.77f;
            }

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                63,             // Expected leaf node size
                17,              // Expected non empty leaf count
                2,              // Expected min non empty leaf node depth
                17,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                14,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                17,              // Expected max empty leaf node depth
                8,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                38,             // Expected leaf node size
                22,              // Expected non empty leaf count
                1,              // Expected min non empty leaf node depth
                27,              // Expected max non empty leaf node depth
                13,              // Expected average non empty leaf node depth
                13,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                23,              // Expected max empty leaf node depth
                11,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   
        
        }

        void TestPath09c()
        {
            // Testing series of increasing small items centered around the same point.
            // At each stage of the build process only one item can be considered large.
            // The test aims to cause the large item approach to split one large item per level.

            const uint32_t numVolumes = 40;
            VectorType * volumeExtents = new VectorType[numVolumes * 2];

            float length = 100.0f;
            for (uint32_t volumeIndex = 0 ; volumeIndex < numVolumes ; ++volumeIndex)
            {
                volumeExtents[(volumeIndex * 2)] = VectorType(0.0f, -length, -length);
                volumeExtents[(volumeIndex * 2) + 1] = VectorType(0.0f, length, length);
                length *= 0.77f;
            }

            ConsistencyTolerances tolerances;

            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                63,             // Expected leaf node size
                17,              // Expected non empty leaf count
                2,              // Expected min non empty leaf node depth
                17,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                16,              // Expected empty leaf count
                2,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                8,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                38,             // Expected leaf node size
                22,              // Expected non empty leaf count
                1,              // Expected min non empty leaf node depth
                27,              // Expected max non empty leaf node depth
                13,              // Expected average non empty leaf node depth
                19,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                27,              // Expected max empty leaf node depth
                12,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   
        
        }

        void TestGameAsset01()
        {
            // Testing game asset - environment mesh from StarWars.
            // This asset exhibits the large item issue.
            VectorType * volumeExtents = NULL;
            uint32_t numVolumes = 0;

            EATESTAssert(LoadMeshFile("all_arch_und_cantina_hallway_01", &volumeExtents, numVolumes), "Failed to load file");

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 10;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                4,              // Expected leaf node size
                223,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                15,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                39,              // Expected empty leaf count
                6,              // Expected min empty leaf node depth
                13,              // Expected max empty leaf node depth
                8,               // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,              // Expected leaf node size
                223,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                15,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                39,              // Expected empty leaf count
                6,              // Expected min empty leaf node depth
                13,              // Expected max empty leaf node depth
                8,               // Expected average empty leaf node depth
                tolerances);

            if (volumeExtents)
            {
                EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeExtents);
            }
        }

        void TestGameAsset02()
        {
            // Testing game asset - environment mesh from StarWars.
            // This asset exhibits the large item issue.
            VectorType * volumeExtents = NULL;
            uint32_t numVolumes = 0;            

            EATESTAssert(LoadMeshFile("all_arch_und_cantina_hallway_02", &volumeExtents, numVolumes), "Failed to load file");

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                4,             // Expected leaf node size
                219,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                15,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                37,              // Expected empty leaf count
                6,              // Expected min empty leaf node depth
                13,              // Expected max empty leaf node depth
                8,               // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,             // Expected leaf node size
                219,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                15,              // Expected max non empty leaf node depth
                9,              // Expected average non empty leaf node depth
                37,              // Expected empty leaf count
                6,              // Expected min empty leaf node depth
                13,              // Expected max empty leaf node depth
                8,               // Expected average empty leaf node depth
                tolerances);

            if (volumeExtents)
            {
                EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeExtents);
            }
        }

        void TestGameAsset03()
        {
            // Testing game asset - environment mesh from StarWars.
            // This asset exhibits the large item issue.
            VectorType * volumeExtents = NULL;
            uint32_t numVolumes = 0;

            EATESTAssert(LoadMeshFile("all_arch_und_cantina_room_01", &volumeExtents, numVolumes), "Failed to load file");

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                4,             // Expected leaf node size
                253,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                18,              // Expected max non empty leaf node depth
                10,              // Expected average non empty leaf node depth
                60,              // Expected empty leaf count
                5,              // Expected min empty leaf node depth
                15,              // Expected max empty leaf node depth
                9,               // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,             // Expected leaf node size
                255,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                18,              // Expected max non empty leaf node depth
                10,              // Expected average non empty leaf node depth
                60,              // Expected empty leaf count
                5,              // Expected min empty leaf node depth
                15,              // Expected max empty leaf node depth
                9,               // Expected average empty leaf node depth
                tolerances);

            if (volumeExtents)
            {
                EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeExtents);
            }
        }

        void TestGameAsset04()
        {
            // Testing game asset - environment mesh from StarWars.
            // This asset exhibits the large item issue.
            VectorType * volumeExtents = NULL;
            uint32_t numVolumes = 0;

            EATESTAssert(LoadMeshFile("all_arch_und_cantina_room_02a", &volumeExtents, numVolumes), "Failed to load file");

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                4,             // Expected leaf node size
                221,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                16,              // Expected max non empty leaf node depth
                10,              // Expected average non empty leaf node depth
                58,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                9,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );   

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,             // Expected leaf node size
                220,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                16,              // Expected max non empty leaf node depth
                10,              // Expected average non empty leaf node depth
                57,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                9,              // Expected average empty leaf node depth
                tolerances      // Tolerances
                );

            if (volumeExtents)
            {
                EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeExtents);
            }
        }

        void TestGameAsset05()
        {
            // Testing game asset - environment mesh from StarWars.
            // This asset exhibits the large item issue.
            VectorType * volumeExtents = NULL;
            uint32_t numVolumes = 0;

            EATESTAssert(LoadMeshFile("all_arch_und_cantina_room_02b", &volumeExtents, numVolumes), "Failed to load file");

            ConsistencyTolerances tolerances;
            tolerances.nonEmptyLeafCount = 20;
            tolerances.minNonEmptyLeafNodeDepth = 3;
            tolerances.maxNonEmptyLeafNodeDepth = 3;
            tolerances.averageNonEmptyLeafNodeDepth = 3;
            tolerances.emptyLeafCount = 5;
            tolerances.minEmptyLeafNodeDepth = 3;
            tolerances.maxEmptyLeafNodeDepth = 3;
            tolerances.averageEmptyLeafNodeDepth = 3;

            // With large item feature disabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                1.0f,           // Large item threshold
                4,             // Expected leaf node size
                277,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                18,              // Expected max non empty leaf node depth
                10,              // Expected average non empty leaf node depth
                50,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                9,               // Expected average empty leaf node depth
                tolerances);

            // With large item feature enabled
            RunTest(
                numVolumes,     // Entry count
                volumeExtents,  // Entry extents
                4,              // Split threshold
                0.8f,           // Large item threshold
                4,             // Expected leaf node size
                276,              // Expected non empty leaf count
                4,              // Expected min non empty leaf node depth
                18,              // Expected max non empty leaf node depth
                10,              // Expected average non empty leaf node depth
                49,              // Expected empty leaf count
                4,              // Expected min empty leaf node depth
                16,              // Expected max empty leaf node depth
                9,               // Expected average empty leaf node depth
                tolerances);

            if (volumeExtents)
            {
                EA::Allocator::ICoreAllocator::GetDefaultAllocator()->Free(volumeExtents);
            }
        }


    public:

#define TESTKDTREEBUILDER_REGISTER(N,D) EATEST_REGISTER(#N, D, TestKDTreeBuilder, N);

        /// Create the test suite and register all the tests
        void Initialize()
        {
            SuiteName("TestKDTreeBuilder");
#if !defined EA_PLATFORM_MOBILE
            TESTKDTREEBUILDER_REGISTER(Test00, "Test00");
            TESTKDTREEBUILDER_REGISTER(Test01, "Test01");
            TESTKDTREEBUILDER_REGISTER(Test02, "Test02");
            TESTKDTREEBUILDER_REGISTER(Test03, "Test03");
            TESTKDTREEBUILDER_REGISTER(Test04, "Test04");
            TESTKDTREEBUILDER_REGISTER(Test05, "Test05");
            TESTKDTREEBUILDER_REGISTER(Test06, "Test06");
#if !defined EA_DEBUG // Only build these tests for non-debug builds as they are time consuming
            TESTKDTREEBUILDER_REGISTER(Test07, "Test07");
            TESTKDTREEBUILDER_REGISTER(Test08, "Test08");
#endif

            TESTKDTREEBUILDER_REGISTER(Test09, "Test09");

            TESTKDTREEBUILDER_REGISTER(TestPath01, "TestPath01");
            TESTKDTREEBUILDER_REGISTER(TestPath02, "TestPath02");
            TESTKDTREEBUILDER_REGISTER(TestPath02b, "TestPath02b");
            TESTKDTREEBUILDER_REGISTER(TestPath02c, "TestPath02c");
            TESTKDTREEBUILDER_REGISTER(TestPath02d, "TestPath02d");
            TESTKDTREEBUILDER_REGISTER(TestPath03, "TestPath03");
            TESTKDTREEBUILDER_REGISTER(TestPath03b, "TestPath03b");
            TESTKDTREEBUILDER_REGISTER(TestPath03c, "TestPath03c");
            TESTKDTREEBUILDER_REGISTER(TestPath03d, "TestPath03d");
            TESTKDTREEBUILDER_REGISTER(TestPath03e, "TestPath03e");
            TESTKDTREEBUILDER_REGISTER(TestPath04, "TestPath04");
            TESTKDTREEBUILDER_REGISTER(TestPath05, "TestPath05");
            TESTKDTREEBUILDER_REGISTER(TestPath06, "TestPath06");
            TESTKDTREEBUILDER_REGISTER(TestPath07, "TestPath07");
            TESTKDTREEBUILDER_REGISTER(TestPath07b, "TestPath07b");
            TESTKDTREEBUILDER_REGISTER(TestPath07c, "TestPath07c");
            TESTKDTREEBUILDER_REGISTER(TestPath08, "TestPath08");
            TESTKDTREEBUILDER_REGISTER(TestPath08b, "TestPath08b");
            TESTKDTREEBUILDER_REGISTER(TestPath08c, "TestPath08c");
            TESTKDTREEBUILDER_REGISTER(TestPath09, "TestPath09");
            TESTKDTREEBUILDER_REGISTER(TestPath09b, "TestPath09b");
            TESTKDTREEBUILDER_REGISTER(TestPath09c, "TestPath09c");

            TESTKDTREEBUILDER_REGISTER(TestGameAsset01,"TestGameAsset01");
            TESTKDTREEBUILDER_REGISTER(TestGameAsset02,"TestGameAsset02");
            TESTKDTREEBUILDER_REGISTER(TestGameAsset03,"TestGameAsset03");
            TESTKDTREEBUILDER_REGISTER(TestGameAsset04,"TestGameAsset04");
            TESTKDTREEBUILDER_REGISTER(TestGameAsset05,"TestGameAsset05");
#endif // EA_PLATFORM_MOBILE
        }
    } gTestKDTreeBuilder;

} // namespace test

