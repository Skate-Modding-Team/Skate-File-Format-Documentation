// (c) Electronic Arts. All Rights Reserved.


#ifndef RWCOLLISION_VOLUMES_EXAMPLES_COMMON_COMMON_H
#define RWCOLLISION_VOLUMES_EXAMPLES_COMMON_COMMON_H


#if !defined EA_PLATFORM_PS3_SPU

#include <rw/collision/common.h>

#include <stdio.h>

#include <rw/collision/clustertriangleiterator.h>
#include <rw/collision/clusteredmeshcluster_methods.h>
#include <rw/collision/simplemappedarray.h>
#include <rw/collision/triangle.h>
#include <rw/collision/triangleclusterprocedural.h>

#include <coreallocator/icoreallocator.h>

namespace common
{


void DescribeSMA(const rw::collision::SimpleMappedArray *const sma);
inline void DescribeSMA(const rw::collision::SimpleMappedArray *const sma)
{
    const uint32_t numTriangles(sma->GetVolumeCount());
    printf("SingleMappedArray has %d triangle volumes\n", numTriangles);

    rwpmath::Vector3 p0(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 p1(rwpmath::GetVector3_Zero());
    rwpmath::Vector3 p2(rwpmath::GetVector3_Zero());

    for (uint16_t triangleIndex = 0; static_cast<uint32_t>(triangleIndex) < numTriangles; ++triangleIndex)
    {
        const rw::collision::Volume *const volume(sma->GetVolume(triangleIndex));
        const rw::collision::TriangleVolume *const triangle(static_cast<const rw::collision::TriangleVolume *>(volume));

        printf("\nTriangle %d:\n", triangleIndex);

        if (triangle)
        {
            triangle->GetPoints(p0, p1, p2);

            const float p0_x(p0.GetX());
            const float p0_y(p0.GetY());
            const float p0_z(p0.GetZ());
            const float p1_x(p1.GetX());
            const float p1_y(p1.GetY());
            const float p1_z(p1.GetZ());
            const float p2_x(p2.GetX());
            const float p2_y(p2.GetY());
            const float p2_z(p2.GetZ());

            printf("  Vertices:\n    <%f, %f, %f>\n    <%f, %f, %f>\n    <%f, %f, %f>\n",
                p0_x, p0_y, p0_z,
                p1_x, p1_y, p1_z,
                p2_x, p2_y, p2_z);

            const float edgeCosine0(triangle->GetEdgeCos(0));
            const float edgeCosine1(triangle->GetEdgeCos(1));
            const float edgeCosine2(triangle->GetEdgeCos(2));

            printf("  Edge cosines:\n    %f\n    %f\n    %f\n",
                edgeCosine0,
                edgeCosine1,
                edgeCosine2);

            printf("  Volume flags:\n");

            const uint32_t volumeFlags(triangle->GetFlags());
            if (volumeFlags & rw::collision::VOLUMEFLAG_ISENABLED)             printf("    %s\n", "VOLUMEFLAG_ISENABLED");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLENORMALISDIRTY) printf("    %s\n", "VOLUMEFLAG_TRIANGLENORMALISDIRTY");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEONESIDED)      printf("    %s\n", "VOLUMEFLAG_TRIANGLEONESIDED");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX)   printf("    %s\n", "VOLUMEFLAG_TRIANGLEEDGE0CONVEX");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX)   printf("    %s\n", "VOLUMEFLAG_TRIANGLEEDGE1CONVEX");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX)   printf("    %s\n", "VOLUMEFLAG_TRIANGLEEDGE2CONVEX");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEUSEEDGECOS)    printf("    %s\n", "VOLUMEFLAG_TRIANGLEUSEEDGECOS");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEVERT0DISABLE)  printf("    %s\n", "VOLUMEFLAG_TRIANGLEVERT0DISABLE");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEVERT1DISABLE)  printf("    %s\n", "VOLUMEFLAG_TRIANGLEVERT1DISABLE");
            if (volumeFlags & rw::collision::VOLUMEFLAG_TRIANGLEVERT2DISABLE)  printf("    %s\n", "VOLUMEFLAG_TRIANGLEVERT2DISABLE");
        }
    }
}


inline void DescribeCluster(const rw::collision::ClusteredMeshCluster & cluster,
                            const uint16_t flagsDefault,
                            const uint8_t groupIDSize,
                            const uint8_t surfaceIDSize,
                            const float vertexCompressionGranularity);

void DescribeCluster(const rw::collision::ClusteredMeshCluster & cluster,
                     const uint16_t flagsDefault,
                     const uint8_t groupIDSize,
                     const uint8_t surfaceIDSize,
                     const float vertexCompressionGranularity)
{
    rw::collision::ClusterParams clusterParams;
    clusterParams.mFlags = flagsDefault;
    clusterParams.mGroupIdSize = groupIDSize;
    clusterParams.mSurfaceIdSize = surfaceIDSize;
    clusterParams.mVertexCompressionGranularity = vertexCompressionGranularity;

    const uint32_t numUnits(cluster.unitCount);
    printf("Cluster has %d units\n", numUnits);

    uint32_t triangleIndex = 0;
    for (rw::collision::ClusterTriangleIterator<> it(cluster, clusterParams, 0, cluster.unitCount); !it.AtEnd(); it.Next())
    {
        printf("\nUnit Triangle %d:\n", triangleIndex);
        ++triangleIndex;

        rwpmath::Vector3 p0, p1, p2;
        it.GetVertices(p0, p1, p2);

        const float p0_x(p0.GetX());
        const float p0_y(p0.GetY());
        const float p0_z(p0.GetZ());
        const float p1_x(p1.GetX());
        const float p1_y(p1.GetY());
        const float p1_z(p1.GetZ());
        const float p2_x(p2.GetX());
        const float p2_y(p2.GetY());
        const float p2_z(p2.GetZ());

        printf("  Vertices:\n    <%f, %f, %f>\n    <%f, %f, %f>\n    <%f, %f, %f>\n",
            p0_x, p0_y, p0_z,
            p1_x, p1_y, p1_z,
            p2_x, p2_y, p2_z);

        rwpmath::Vector3 edgeCosines;
        uint32_t triangleFlags = it.GetEdgeCosinesAndFlags(edgeCosines);

        const float edgeA = edgeCosines.GetX();
        const float edgeB = edgeCosines.GetY();
        const float edgeC = edgeCosines.GetZ();

        printf("  Edge cosines:\n    %f\n    %f\n    %f\n",
            edgeA,
            edgeB,
            edgeC);

        if (triangleFlags & rw::collision::VOLUMEFLAG_ISENABLED)             printf("    %s\n", "VOLUMEFLAG_ISENABLED");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLENORMALISDIRTY) printf("    %s\n", "VOLUMEFLAG_TRIANGLENORMALISDIRTY");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEONESIDED)      printf("    %s\n", "VOLUMEFLAG_TRIANGLEONESIDED");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEEDGE0CONVEX)   printf("    %s\n", "VOLUMEFLAG_TRIANGLEEDGE0CONVEX");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEEDGE1CONVEX)   printf("    %s\n", "VOLUMEFLAG_TRIANGLEEDGE1CONVEX");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEEDGE2CONVEX)   printf("    %s\n", "VOLUMEFLAG_TRIANGLEEDGE2CONVEX");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEUSEEDGECOS)    printf("    %s\n", "VOLUMEFLAG_TRIANGLEUSEEDGECOS");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEVERT0DISABLE)  printf("    %s\n", "VOLUMEFLAG_TRIANGLEVERT0DISABLE");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEVERT1DISABLE)  printf("    %s\n", "VOLUMEFLAG_TRIANGLEVERT1DISABLE");
        if (triangleFlags & rw::collision::VOLUMEFLAG_TRIANGLEVERT2DISABLE)  printf("    %s\n", "VOLUMEFLAG_TRIANGLEVERT2DISABLE");
    }
}


void DescribeClusteredMesh(const rw::collision::ClusteredMesh & clusteredMesh);
inline void DescribeClusteredMesh(const rw::collision::ClusteredMesh & clusteredMesh)
{
    const uint32_t numClusters(clusteredMesh.GetNumCluster());
    const uint16_t flags(clusteredMesh.GetFlags());
    const uint8_t groupIDSize(clusteredMesh.GetGroupIdSize());
    const uint8_t surfaceIDSize(clusteredMesh.GetSurfaceIdSize());
    const float vertexCompressionGranularity(clusteredMesh.GetVertexCompressionGranularity());

    printf("Clustered mesh has %d cluster(s)\n\n", numClusters);

    for (uint32_t clusterIndex = 0; clusterIndex < numClusters; ++clusterIndex)
    {
        const rw::collision::ClusteredMeshCluster &cluster(clusteredMesh.GetCluster(clusterIndex));

        printf("Cluster [%d]:\n\n", clusterIndex);
        DescribeCluster(
            cluster,
            flags,
            groupIDSize,
            surfaceIDSize,
            vertexCompressionGranularity);
    }
}


void DescribeTriangleClusterProcedural(const rw::collision::TriangleClusterProcedural & tcp);
inline void DescribeTriangleClusterProcedural(const rw::collision::TriangleClusterProcedural & tcp)
{
    const rw::collision::ClusteredMeshCluster &cluster(tcp.GetCluster());
    const rw::collision::ClusterParams &params(tcp.GetClusterParams());

    printf("TriangleClusterProcedural cluster:\n\n");
    DescribeCluster(
        cluster,
        params.mFlags,
        params.mGroupIdSize,
        params.mSurfaceIdSize,
        params.mVertexCompressionGranularity);
}


} // namespace common


#endif // #if !defined EA_PLATFORM_PS3_SPU

#endif // RWCOLLISION_VOLUMES_EXAMPLES_COMMON_COMMON_H

