// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_CYLINDERQUERY_H
#define PUBLIC_RW_COLLISION_CYLINDERQUERY_H

/*************************************************************************************************************

 File: rwccylinderquery.hpp

 Purpose: Definitions for the system for querying intersections of primitive pairs.

 */

#include "rw/collision/common.h"
#include "rw/collision/volume.h"

namespace rw
{
namespace collision
{


rwpmath::VecFloat
FindBestSepDirWithCylinder( rwpmath::Vector3 &bestSepDir, const GPCylinder &gp1, const GPInstance &gp2 );


inline rwpmath::VecFloat
FindBestSeparatingDirCylVol( rwpmath::Vector3 &bestSepDir, const GPInstance &gpCylinder, const GPInstance &gpOther )
{
    return FindBestSepDirWithCylinder(bestSepDir, static_cast<const GPCylinder&> (gpCylinder), gpOther);
}

inline rwpmath::VecFloat
FindBestSeparatingDirVolCyl( rwpmath::Vector3 &bestSepDir, const GPInstance &gpOther, const GPInstance &gpCylinder )
{
    rwpmath::VecFloat ret = FindBestSepDirWithCylinder(bestSepDir, static_cast<const GPCylinder&> (gpCylinder), gpOther);
    bestSepDir = -bestSepDir;
    return ret;
}


} // namespace collision
} // namespace rw
#endif // PUBLIC_RW_COLLISION_CYLINDERQUERY_H
