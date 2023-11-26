// (c) Electronic Arts. All Rights Reserved.
/*************************************************************************************************************

 File: rwcScaledClusteredMesh.cpp

 Purpose: A wrapper class around the given clusteredMesh which performs
 a uniform scaling before running line and bbox queries.

 */
// ***********************************************************************************************************
// Includes
#include "rw/collision/procedural.h"
#include "rw/collision/scaledclusteredmesh.h"

// ***********************************************************************************************************
// Static Variables + Static Data Member Definitions

/*
This is the default vtable used by objects of type ScaledClusteredMesh.
*/
rw::collision::Procedural::VTable rw::collision::ScaledClusteredMesh::s_vTable = 
{
#if defined __SNC__
#pragma diag_suppress=1787 // This extension of an enumeration isn't strictly valid however it works so the warning is disabled
#endif
    RWCOBJECTTYPE_SCALEDCLUSTEREDMESH,
#if defined __SNC__
#pragma diag_default=1787
#endif

    //Cast function pointers to they types defined in aggregate
    static_cast<rw::collision::Aggregate::GetSizeFn>              (&ScaledClusteredMesh::GetSizeThis),
    RWMATH_VECTOR3_ALIGNMENT,
    TRUE,
    static_cast<rw::collision::Aggregate::UpdateFn>               (&ScaledClusteredMesh::UpdateThis),
    static_cast<rw::collision::Aggregate::LineIntersectionQueryFn>(&ScaledClusteredMesh::LineIntersectionQueryThis),
    static_cast<rw::collision::Aggregate::BBoxOverlapQueryFn>     (&ScaledClusteredMesh::BBoxOverlapQueryThis)
};


//Update the bounding box of the scaledClusteredMesh
void rw::collision::ScaledClusteredMesh::UpdateThis()
{
    m_clusteredMesh->Update();
    m_AABB = m_clusteredMesh->GetBBox();

    // Recompute the bounding box with the scale
    m_AABB.m_max *= m_scale;
    m_AABB.m_min *= m_scale;
}


//Line query for scaledClusteredMesh
uint32_t rw::collision::ScaledClusteredMesh::LineIntersectionQueryThis(rw::collision::VolumeLineQuery *lineQuery, const rwpmath::Matrix44Affine *tm)
{
    const uint32_t startCount = lineQuery->m_instVolCount;    
    const rwpmath::VecFloat scale = m_scale;

    rwpmath::Vector3 originalLineStart = lineQuery->m_pt1;
    rwpmath::Vector3 originalLineEnd = lineQuery->m_pt2;
    float originalFatness = lineQuery->m_fatness;
     
    rwpmath::Vector3 meshToWorld = rwpmath::GetVector3_Zero();
    if(tm)
    {
        meshToWorld = tm->GetW();
    }

    //transform line from world space into the mesh space, scale and transform back  
    lineQuery->m_pt1 = lineQuery->m_pt1*m_invScale + meshToWorld*(1.0f - m_invScale);
    lineQuery->m_pt2 = lineQuery->m_pt2*m_invScale + meshToWorld*(1.0f - m_invScale);
    lineQuery->m_fatness *= m_invScale;

    //perform query in world space
    const uint32_t ret = m_clusteredMesh->LineIntersectionQuery(lineQuery, tm);
    const uint32_t endCount = lineQuery->m_instVolCount;

    for (uint32_t i = startCount; i < endCount; ++i)
    {
        EA_ASSERT(reinterpret_cast<rw::collision::Volume*>(lineQuery->m_instVolPool + i)->GetType() == rw::collision::VOLUMETYPETRIANGLE);
        rw::collision::TriangleVolume* tri = static_cast<rw::collision::TriangleVolume*>(lineQuery->m_instVolPool + i);

        // scale triangle volumes which are in mesh space
        rwpmath::Vector3 p1;
        rwpmath::Vector3 p2;
        rwpmath::Vector3 p3;
        tri->GetPoints(p1, p2, p3);
        tri->SetPoints(p1 * scale, p2 * scale, p3 * scale);

        // scale the hit point as well
        //transform to mesh space, scale and transform back to world space
        lineQuery->m_resBuffer[i].position = lineQuery->m_resBuffer[i].position*scale + meshToWorld*(1.0f-scale);       

        //scale depth parameter 
        lineQuery->m_resBuffer[i].volParam.Z() *= scale*scale;
    }

    // put the line back
    lineQuery->m_pt1 = originalLineStart;
    lineQuery->m_pt2 = originalLineEnd;
    lineQuery->m_fatness = originalFatness;

    return(ret);
}

// BBox query for scaledClusteredMesh
uint32_t rw::collision::ScaledClusteredMesh::BBoxOverlapQueryThis(rw::collision::VolumeBBoxQuery *bboxQuery, const rwpmath::Matrix44Affine *tm)
{
    const uint32_t startCount = bboxQuery->m_instVolCount;
    const rwpmath::VecFloat scale = m_scale;

    rwpmath::Vector3 meshToWorldTranslation = rwpmath::GetVector3_Zero();
    if(tm)
    {
        //the rotation is not important in the following
        meshToWorldTranslation = tm->GetW();
    }

    rw::collision::AABBox originalAABB = bboxQuery->m_aabb;

    //transform bounding box from world space into the mesh space, scale and transform back  
    bboxQuery->m_aabb.m_min = bboxQuery->m_aabb.m_min*m_invScale + meshToWorldTranslation*(1.0f - m_invScale);
    bboxQuery->m_aabb.m_max = bboxQuery->m_aabb.m_max*m_invScale + meshToWorldTranslation*(1.0f - m_invScale);

    const uint32_t ret = m_clusteredMesh->BBoxOverlapQuery(bboxQuery, tm);
    const uint32_t endCount = bboxQuery->m_instVolCount;

    for (uint32_t i = startCount; i < endCount; ++i)
    {
        EA_ASSERT(reinterpret_cast<rw::collision::Volume*>(bboxQuery->m_instVolPool + i)->GetType() == rw::collision::VOLUMETYPETRIANGLE);
        rw::collision::TriangleVolume* tri = static_cast<rw::collision::TriangleVolume*>(bboxQuery->m_instVolPool + i);

        // scale triangle volumes
        rwpmath::Vector3 p1;
        rwpmath::Vector3 p2;
        rwpmath::Vector3 p3;
        tri->GetPoints(p1, p2, p3);
        tri->SetPoints(p1 * scale, p2 * scale, p3 * scale);
        // scale the bbox of the triangle
        rw::collision::AABBox & bbox = bboxQuery->m_primVRefBuffer[i].bBox;

        // the bbox of the triangle is in world coordinates
        // we need to transform the bbox into mesh space, scale it and then 
        // reapply the mesh to world transform, in formulas e.g. (bBox.Min() - meshToWorldTranslation) * scale + meshToWorldTranslation
        bbox.Set( bbox.Min() * scale + meshToWorldTranslation * ( 1.0f - scale ),
                  bbox.Max() * scale + meshToWorldTranslation * ( 1.0f - scale ) );
    }

    // put bbox back
    bboxQuery->m_aabb = originalAABB;

    return(ret);
}
