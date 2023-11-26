// (c) Electronic Arts. All Rights Reserved.

/*************************************************************************************************************

File: volumecompare.h

Purpose: Volume comparison functions

*/

#ifndef VOLUME_COMPARE_H
#define VOLUME_COMPARE_H

#include <EAAssert/eaassert.h>
#include <EABase/eabase.h>
#include <rw/collision/common.h>

namespace rw
{
namespace collision
{
namespace unittest
{



    inline bool IsSimilar(const rw::collision::Volume& vol1, const rw::collision::Volume& vol2, rwpmath::VecFloatInParam epsilon = rwpmath::VecFloat(rw::math::SMALL_FLOAT))
    {
        using namespace rwpmath;

        bool isSimilar = (vol1.GetType() == vol2.GetType())
            && (vol1.GetFlags() == vol2.GetFlags())
            && (vol1.GetGroup() == vol2.GetGroup())
            && (vol1.GetSurface() == vol2.GetSurface())
            && rwpmath::IsSimilar(vol1.GetRadius(), vol2.GetRadius(), epsilon);

        if (vol1.GetType() != VOLUMETYPETRIANGLE)
        {
            isSimilar = isSimilar && rwpmath::IsSimilar(vol1.GetLocalTransform(), vol2.GetLocalTransform(), epsilon);
        }

        if (isSimilar)
        {
            switch (vol1.GetType())
            {
            case VOLUMETYPESPHERE:
                {
                    //const SphereVolume &sph1 = static_cast<SphereVolume &>(vol1);
                    //const SphereVolume &sph2 = static_cast<SphereVolume &>(vol2);
                    break;
                }
            case VOLUMETYPECAPSULE:
                {
                    const rw::collision::CapsuleVolume &cap1 = static_cast<const rw::collision::CapsuleVolume &>(vol1);
                    const rw::collision::CapsuleVolume &cap2 = static_cast<const rw::collision::CapsuleVolume &>(vol2);
                    isSimilar = isSimilar && rwpmath::IsSimilar(cap1.GetHalfHeight(), cap2.GetHalfHeight(), epsilon);
                    break;
                }
            case VOLUMETYPEBOX:
                {
                    const rw::collision::BoxVolume &box1 = static_cast<const rw::collision::BoxVolume &>(vol1);
                    const rw::collision::BoxVolume &box2 = static_cast<const rw::collision::BoxVolume &>(vol2);
                    Vector3 dims1;
                    Vector3 dims2;
                    box1.GetDimensions(dims1);
                    box2.GetDimensions(dims2);
                    isSimilar = isSimilar && IsSimilar(dims1, dims2, epsilon);
                    break;
                }
            case VOLUMETYPECYLINDER:
                {
                    const rw::collision::CylinderVolume &cyl1 = static_cast<const rw::collision::CylinderVolume &>(vol1);
                    const rw::collision::CylinderVolume &cyl2 = static_cast<const rw::collision::CylinderVolume &>(vol2);
                    isSimilar = isSimilar && rwpmath::IsSimilar(cyl1.GetHalfHeight(), cyl2.GetHalfHeight(), epsilon) 
                        && rwpmath::IsSimilar(cyl1.GetInnerRadius(), cyl2.GetInnerRadius(), epsilon);
                }
            case VOLUMETYPETRIANGLE:
                {
                    const TriangleVolume &tri1 = static_cast<const TriangleVolume &>(vol1);
                    const TriangleVolume &tri2 = static_cast<const TriangleVolume &>(vol2);

                    rwpmath::Vector3 triangle1Vertices[3];
                    tri1.GetPoints(triangle1Vertices[0], triangle1Vertices[1], triangle1Vertices[2], NULL);
                    rwpmath::Vector3 triangle1Normal;
                    tri1.GetNormal(triangle1Normal);

                    rwpmath::Vector3 triangle2Vertices[3];
                    tri2.GetPoints(triangle2Vertices[0], triangle2Vertices[1], triangle2Vertices[2], NULL);
                    rwpmath::Vector3 triangle2Normal;
                    tri2.GetNormal(triangle2Normal);

                    isSimilar = isSimilar && rwpmath::IsSimilar(triangle1Vertices[0], triangle2Vertices[0]);
                    isSimilar = isSimilar && rwpmath::IsSimilar(triangle1Vertices[1], triangle2Vertices[1]);
                    isSimilar = isSimilar && rwpmath::IsSimilar(triangle1Vertices[2], triangle2Vertices[2]);
                    isSimilar = isSimilar && rwpmath::IsSimilar(triangle1Normal, triangle2Normal);

                    break;
                }
            case VOLUMETYPEAGGREGATE:
                {
                    const rw::collision::AggregateVolume &agg1 = static_cast<const rw::collision::AggregateVolume &>(vol1);
                    const rw::collision::AggregateVolume &agg2 = static_cast<const rw::collision::AggregateVolume &>(vol2);
                    isSimilar = (agg1.GetAggregate() == agg2.GetAggregate());
                    if (!isSimilar)
                    {
                        EA_FAIL_MSG("rw::collision::unittest::IsSimilar() does not currently compare aggregates");
                    }
                    break;
                }
            default:
                EA_FAIL_MSG("Volume type not handled by rw::collision::unittest::IsSimilar()");
                break;
            }
        }

        return isSimilar;
    }


}
}
}


#endif // !VOLUME_COMPARE_H
