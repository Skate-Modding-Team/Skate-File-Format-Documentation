// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RW_COLLISION_PROCEDURAL_H
#define PUBLIC_RW_COLLISION_PROCEDURAL_H

/*************************************************************************************************************

 File: rwcprocedural.hpp

 Purpose: Pure virtual base class for procedural aggregates.
 */


#include "rw/collision/common.h"
#include "rw/collision/aggregate.h"

namespace rw
{
namespace collision
{

class Procedural;


/**
\brief Aggregate data consisting of compressed volume data.

This is an abstract class for procedural collision containers where the data
isn't stored in the form of raw volumes as is  MappedArray.
Only derived classes should be instanced. The derived class will define
what the collision data is and what format it is stored in.
The base aggregate class provides methods to query the collision data
using a line or a bounding box however the derived class methods will actually perform
the query.

\see rw::collision::Aggregate::IsProcedural()

\see TriangleKDTreeProcedural

\importlib rwccore
 */
class Procedural : public Aggregate
{
public:

    /**
    \internal
    */
    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
        void Serialize(Archive &ar, uint32_t /*version*/)
    {
        // Serialize base class
        ar & EA::Serialization::MakeNamedValue(*static_cast<Aggregate*>(this), "Aggregate");

    }

protected:
    /**
    \internal
    */
    Procedural(uint32_t numVolumes, VTable *vTable)
    : Aggregate(numVolumes, vTable)
    {
    }
};

} // namespace collision
} // namespace rw

#endif // PUBLIC_RW_COLLISION_PROCEDURAL_H
