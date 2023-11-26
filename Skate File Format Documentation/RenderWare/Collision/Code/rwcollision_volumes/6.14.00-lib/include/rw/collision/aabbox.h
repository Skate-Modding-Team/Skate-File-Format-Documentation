// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RWC_AABBOX_H
#define PUBLIC_RWC_AABBOX_H

/*************************************************************************************************************

 File: rwcaabbox.hpp

 Purpose: Axis aligned bounding box.
 */

#include "rw/collision/common.h"
#include "rw/collision/volumedata.h"
#include "eaphysics_base/version.h"

#if defined EAPHYSICS_BASE_HAS_AABBOX

#include "eaphysics/aabbox.h"

namespace rw
{
    namespace collision
    {
        class AABBoxU : public rwpmath::fpu::AABBox
        {
        public:
            typedef float FloatType;
            typedef rw::math::fpu::Vector3 Vector3Type;

            EA_FORCE_INLINE AABBoxU()
            {
            }

            EA_FORCE_INLINE AABBoxU(rw::math::fpu::Vector3::InParam min, rw::math::fpu::Vector3::InParam max)
            {
                m_min = min;
                m_max = max;
            }

            EA_FORCE_INLINE AABBoxU(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
            {
                m_min.Set(minX, minY, minZ);
                m_max.Set(maxX, maxY, maxZ);
            }

            template <class Archive> void Serialize(Archive &ar, uint32_t /*version*/)
            {
                ar & EA_SERIALIZATION_NAMED_VALUE(m_min);
                ar & EA_SERIALIZATION_NAMED_VALUE(m_max);
            }

            EA_FORCE_INLINE const rw::math::fpu::Vector3 &Min() const
            {
                return m_min;
            }

            EA_FORCE_INLINE const rw::math::fpu::Vector3 &Max() const
            {
                return m_max;
            }

            EA_FORCE_INLINE void Set(rw::math::fpu::Vector3::InParam min, rw::math::fpu::Vector3::InParam max)
            {
                m_min = min;
                m_max = max;
            }
        };

        EA_FORCE_INLINE AABBoxU Union(const AABBoxU &a, const AABBoxU &b)
        {
            return AABBoxU(Min(a.Min(), b.Min()), Max(a.Max(), b.Max()));
        }

        class AABBox : public rwpmath::AABBox
        {
        public:
            typedef rwpmath::Vector3 Vector3Type;

            using rwpmath::AABBox::m_min;
            using rwpmath::AABBox::m_max;

            EA_FORCE_INLINE AABBox()
                : rwpmath::AABBox()
            {
            }
            
            explicit EA_FORCE_INLINE AABBox(const rwpmath::AABBox &box)
                : rwpmath::AABBox(box)
            {
            }

            EA_FORCE_INLINE AABBox(rwpmath::Vector3::InParam min, rwpmath::Vector3::InParam max)
            {
                // Assertions prevent uses of AABBoxFromMinMax as this constructor is used to generate empty bboxes
                m_min = min;
                m_max = max;
            }

            EA_FORCE_INLINE AABBox(float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
                : rwpmath::AABBox(rwpmath::AABBoxFromMinMax(rwpmath::Vector3(minX, minY, minZ), rwpmath::Vector3(maxX, maxY, maxZ)))
            {
            }

            const rwpmath::Vector3 &Min() const
            {
                return m_min;
            }

            const rwpmath::Vector3 &Max() const
            {
                return m_max;
            }

            template <class Archive> void Serialize(Archive &ar, uint32_t /*version*/)
            {
                ar & EA_SERIALIZATION_NAMED_VALUE(m_min);
                ar & EA_SERIALIZATION_NAMED_VALUE(m_max);
            }

            EA_FORCE_INLINE void Set(rwpmath::Vector3::InParam min, rwpmath::Vector3::InParam max)
            {
                m_min = min;
                m_max = max;
            }

            inline RwpBool IsValid() const
            {
                RwpBool result = static_cast<RwpBool>(
                    (m_min.X() <= m_max.X())
                    &&  (m_min.Y() <= m_max.Y())
                    &&  (m_min.Z() <= m_max.Z()));

                return result;
            }

            EA_FORCE_INLINE AABBox Scale(const float &scale) const
            {
                EA_ASSERT(IsValid());

                // Convert to center,diagonal representation
                rwpmath::VecFloat half = rwpmath::GetVecFloat_Half();
                rwpmath::Vector3 center = half * (m_max + m_min);
                rwpmath::Vector3 diagonal = half * (m_max - m_min);

                // Scale diagonal
                rwpmath::Vector3 newDiagonal = rwpmath::VecFloat(scale) * diagonal;

                return AABBox(center - newDiagonal, center + newDiagonal);
            }

            inline AABBox Transform(const rwpmath::Matrix44Affine *matrix) const
            {
                EA_ASSERT(IsValid());

                if (0 == matrix)
                {
                    return *this;
                }

                // Convert to center,diagonal representation
                rwpmath::VecFloat half = rwpmath::GetVecFloat_Half();
                rwpmath::Vector3 center = half * (m_max + m_min);
                rwpmath::Vector3 diagonal = half * (m_max - m_min);

                // Transform center
                rwpmath::Vector3 newCenter = TransformPoint(center, *matrix);

                // Calculate new diagonal that gives a bounding box for the transformed old box.
                // Original box (Z not shown):
                // +----Y----T
                // |    |    |
                // +----C----X
                // |    |    |
                // +----+----U
                // Transformed box:
                // +---+-------R
                // |  / \      |
                // | +   \     |
                // |/ \   \    |
                // +   \   Y   |
                // |\   \ / \  |
                // | \   C   \ |
                // |  \ / \   \|
                // |   +   \   T
                // |    \   \ /|
                // |     \   X |
                // |      \ /  |
                // +-------U---+
                // The new diagonal CR is calculated by transforming each axis of the original box separately and combining the results.
                // In the diagrams above:
                // * the x extent of the CR is the x extent of CT which is the sum of the x extents of CX and CY
                // * the y extent of the CR is the y extent of CU which is the sum of the y extents of -CX and CY
                // In general CR = |CX| + |CY| + |CZ| hence:
                rwpmath::Vector3 newDiagonal = rwpmath::Transform(diagonal, rwpmath::Matrix33(
                    Abs(matrix->XAxis()), Abs(matrix->YAxis()), Abs(matrix->ZAxis())));

                return AABBox(newCenter - newDiagonal, newCenter + newDiagonal);
            }

            inline rwpmath::VecFloat Distance(const AABBox &other) const
            {
                EA_ASSERT(IsValid() && other.IsValid());

                rwpmath::Vector3 axisSeparations = rwpmath::Max(m_min - other.m_max, other.m_min - m_max);
                return rwpmath::Max(rwpmath::VecFloat(axisSeparations.X()), rwpmath::VecFloat(axisSeparations.Y()),
                    rwpmath::VecFloat(axisSeparations.Z()));
            }

            EA_FORCE_INLINE RwpBool Contains(const AABBox &box) const
            {
                EA_ASSERT(IsValid());

                return static_cast<RwpBool>(rwpmath::Contains(*this, box));
            }

            EA_FORCE_INLINE RwpBool Contains(rwpmath::Vector3::InParam point) const
            {
                EA_ASSERT(IsValid());

                return static_cast<RwpBool>(rwpmath::Contains(*this, point));
            }

            EA_FORCE_INLINE RwpBool Overlaps(const AABBox &box) const
            {
                EA_ASSERT(IsValid());

                return static_cast<RwpBool>(rwpmath::Overlaps(*this, box));
            }

            EA_FORCE_INLINE void Union(const AABBox &box)
            {
                m_min = rwpmath::Min(m_min, box.m_min);
                m_max = rwpmath::Max(m_max, box.m_max);
            }

            EA_FORCE_INLINE void Union(rwpmath::Vector3::InParam point)
            {
                m_min = rwpmath::Min(m_min, point);
                m_max = rwpmath::Max(m_max, point);
            }
        };

        EA_FORCE_INLINE AABBox Union(const AABBox &a, const AABBox &b)
        {
            return AABBox(rwpmath::Union(a, b));
        }

        EA_FORCE_INLINE AABBox Union(const AABBox &box, rwpmath::Vector3::InParam point)
        {
            return AABBox(rwpmath::Union(box, point));
        }

        EA_FORCE_INLINE AABBox ComputePadded(const AABBox &box, rwpmath::VecFloatInParam padding)
        {
            rwpmath::Vector3 padding3(padding, padding, padding);
            return AABBox(box.m_min - padding3, box.m_max + padding3);
        }
    }
}
#else

#if defined(EA_PLATFORM_PS3_SPU)
#include <vmx2spu.h>
#endif // !defined(EA_PLATFORM_PS3_SPU)

/**
\brief
Simple template class that defines the extents of an axis aligned bounding box, and provides
some useful utilities such as overlap tests.

The  rw::collision::AABBox and \ref rw::collision::AABBoxU types are based on this template. The \ref rw::collision::AABBox
uses aligned vectors and is optimal for use at runtime. The  rw::collision::AABBoxU uses unaligned vectors
for STL compatibility, and is intended for offline use in conditioning code.

\importlib rwccore
 */
template <typename MATRIX4X3TYPE, typename VECTOR3TYPE, class MemoryDumpPolicy>
class AABBoxTemplate //: public MemoryDumpPolicy // -------------------- TODO:
{
public:
    /**
    Define a type for a transformation matrix that can transform an AABBox.
    */
    typedef MATRIX4X3TYPE Matrix4x3Type;
    typedef VECTOR3TYPE Vector3Type;
    typedef typename VECTOR3TYPE::FloatType FloatType;
    typedef typename VECTOR3TYPE::FastFloatType FastFloatType;

    Vector3Type    m_min;          ///< Minimum of extent on X, Y, and Z axes.
    Vector3Type    m_max;          ///< Maximum of extent on X, Y, and Z axes.

    /**
    */
    // NOTE: If any changes to this object affecting its LL-Serialization, you'll also need to
    // make identical changes to its FPU version here: ".\include\cmn\rw\collision\detail\fpu\"
    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/)
    {
        ar & EA_SERIALIZATION_NAMED_VALUE(m_min);
        ar & EA_SERIALIZATION_NAMED_VALUE(m_max);

    }
    // --------------------

    /**
    \brief Default constructor, which does nothing.

    This allows arrays of bounding boxes to be declared, with no construction cost.
    */
    AABBoxTemplate() {}

    /**
    \brief Constructor for axis aligned bounding box.

    This creates a bounding box and sets the corners nearest and furthest from the origin.

    \param min_  Minimum 3D extent.
    \param max_  Maximum 3D extent.
    */
    AABBoxTemplate(const Vector3Type &min_,
                   const Vector3Type &max_)
    {
        m_min = min_;
        m_max = max_;

    }


    /**
    \brief Constructor for axis aligned bounding box.

    The minimum and maximum of the bounding box extent are initialized
    according to the specified parameters.

    \param minx  Min extent on x axis.
    \param miny  Min extent on y axis.
    \param minz  Min extent on z axis.
    \param maxx  Max extent on x axis.
    \param maxy  Max extent on y axis.
    \param maxz  Max extent on z axis.
    */
    AABBoxTemplate(FloatType minx, FloatType miny, FloatType minz,
                   FloatType maxx, FloatType maxy, FloatType maxz)
        : m_min(Vector3Type(minx, miny, minz)),
          m_max(Vector3Type(maxx, maxy, maxz)) {}


    /**
    \brief Set minumum and maximum extent values for axis aligned bounding box.

    \param min  Minimum 3D extent.
    \param max  Maximum 3D extent.
     */
    void
    Set(const Vector3Type &min,
        const Vector3Type &max)
    {
        m_min = min;
        m_max = max;
    }


    /**
    \brief Checks the validity of the bounding box.

    A box is invalid if any component of it's minumum is greater than the corresponding
    component of the maximum.
    \return TRUE if the bounding box min and max values are valid.
    */
    RwpBool
    IsValid() const
    {
        RwpBool result = static_cast<RwpBool>(
                (static_cast<float>(m_min.X()) <= static_cast<float>(m_max.X()))
            &&  (static_cast<float>(m_min.Y()) <= static_cast<float>(m_max.Y()))
            &&  (static_cast<float>(m_min.Z()) <= static_cast<float>(m_max.Z())));

        return result;
    }

    /**
    \brief Gets the bounding box corner nearest to the origin.
    \return Minimum extent of bounding box.
    */
    const Vector3Type &
    Min() const
    {
        return m_min;
    }

    /**
    \brief Gets the bounding box corner farthest to the origin.
    \return Maximum extent of bounding box.
    */
    const Vector3Type &
    Max() const
    {
        return m_max;
    }

    /**
    \brief
    Scale the bounding box about its center and return the result.
    \param scale Scale factor
    \return Scaled bounding box
    */
    AABBoxTemplate
    Scale(FloatType scale) const
    {
        EA_ASSERT(IsValid());

        FastFloatType scalefactor((scale - static_cast<FloatType>(1.0)) * static_cast<FloatType>(0.5));
        Vector3Type offset = (m_max - m_min) * scalefactor;
        AABBoxTemplate sclBox(m_min - offset, m_max + offset);

        return sclBox;
    }

    /**
    \brief
    Test whether bounding box overlaps another bounding box.
    \param bb2  Second bounding box.
    \return TRUE of bounding boxes overlap.
    */
    RwpBool
    Overlaps(const AABBoxTemplate &bb2) const
    {
        EA_ASSERT(IsValid());

        RwpBool result =
                (static_cast<float>(bb2.m_max.X()) >= static_cast<float>(m_min.X()))
            &&  (static_cast<float>(bb2.m_min.X()) <= static_cast<float>(m_max.X()))
            &&  (static_cast<float>(bb2.m_max.Y()) >= static_cast<float>(m_min.Y()))
            &&  (static_cast<float>(bb2.m_min.Y()) <= static_cast<float>(m_max.Y()))
            &&  (static_cast<float>(bb2.m_max.Z()) >= static_cast<float>(m_min.Z()))
            &&  (static_cast<float>(bb2.m_min.Z()) <= static_cast<float>(m_max.Z()));

        return result;
    }


    /**
    \brief Calculates the axis-aligned distance between two bounding boxes.

    Return the infinity-norm distance between two bboxes, in other words, the maximum distance between the
    two boxes on all three axes.   If the boxes are touching, this returns a negative number which is the
    minimum penetration distance of the three axis.

    \param other  Second bounding box.
    \return Axis-aligned distance between the two bboxes, or penetration distance (negative) if overlapping.
    */
    float
    Distance(const AABBoxTemplate &other) const
    {
        EA_ASSERT(IsValid() && other.IsValid());

        Vector3Type temp = rwpmath::Max(m_min - other.m_max, other.m_min - m_max);

        //  The Max ought to vectorise nicely, but the max(x,y,z) might need some work.

        // N.B. casting to float to avoid issues with there not being a
        // Max(XAxisType, YAxisType, ZAxisType) function available in the PS2 libraries
        return rwpmath::Max((float)(temp.X()), (float)(temp.Y()), (float)(temp.Z()));
    }


    /**
    \brief Tests whether the bounding box contains another bounding box.

    \param bb2 Second bounding box.
    \return TRUE if second bounding box is contained.
    */
    RwpBool
    Contains(const AABBoxTemplate &bb2) const
    {
        EA_ASSERT(IsValid());

        RwpBool result =
                (static_cast<float>(bb2.m_max.X()) <= static_cast<float>(m_max.X()))
            &&  (static_cast<float>(bb2.m_min.X()) >= static_cast<float>(m_min.X()))
            &&  (static_cast<float>(bb2.m_max.Y()) <= static_cast<float>(m_max.Y()))
            &&  (static_cast<float>(bb2.m_min.Y()) >= static_cast<float>(m_min.Y()))
            &&  (static_cast<float>(bb2.m_max.Z()) <= static_cast<float>(m_max.Z()))
            &&  (static_cast<float>(bb2.m_min.Z()) >= static_cast<float>(m_min.Z()));

        return result;
    }

    /**
    \brief Tests whether the bounding box contains a point.

    \param point point
    \return TRUE if the point is contained in the bounding box.
    */
    RwpBool
        Contains(rwpmath::Vector3::InParam point) const
    {
        EA_ASSERT(IsValid());

        RwpBool result =
            (static_cast<float>(point.GetX()) <= static_cast<float>(m_max.X()))
            &&  (static_cast<float>(point.GetX()) >= static_cast<float>(m_min.X()))
            &&  (static_cast<float>(point.GetY()) <= static_cast<float>(m_max.Y()))
            &&  (static_cast<float>(point.GetY()) >= static_cast<float>(m_min.Y()))
            &&  (static_cast<float>(point.GetZ()) <= static_cast<float>(m_max.Z()))
            &&  (static_cast<float>(point.GetZ()) >= static_cast<float>(m_min.Z()));

        return result;
    }

    /**
    \brief Computes a new bounding box that contains the current bounding box
    after applying the specified transformation.

    \param mtx  A transformation matrix

    \return New bounding box around the transformed bounding box.
    */
    AABBoxTemplate
    Transform(const Matrix4x3Type *mtx) const
    {
        EA_ASSERT(IsValid());

        if (!mtx)
        {
            return *this;
        }

        // New center
        Vector3Type center = (m_max + m_min) * FastFloatType(0.5f);
        center = TransformPoint(center, *mtx);

        // New diagonal
        Vector3Type diag = (m_max - m_min) * FastFloatType(0.5f);
        diag =  Abs(mtx->XAxis())     * (diag.X())
             +  Abs(mtx->YAxis())    * (diag.Y())
             +  Abs(mtx->ZAxis())    * (diag.Z());

        AABBoxTemplate result(center - diag, center + diag);

        return result;
    }

    /**
    Expands the current bounding box so that it will contain the specified point.
    \param point a point that the bbox is to contain.
    */
    void Union(rwpmath::Vector3::InParam point)
    {
        m_min = rwpmath::Min(m_min, point);
        m_max = rwpmath::Max(m_max, point);
    }

    /**
    Expands the current bounding box so that it will contain the specified bbox.
    \param bbox a bounding box that this bounding box is to contain.
    If the current bounding box already contains the specified bbox, then no change is made.
    */
    void Union(const AABBoxTemplate& bbox)
    {
        m_min = rwpmath::Min(m_min, bbox.m_min);
        m_max = rwpmath::Max(m_max, bbox.m_max);
    }

};



/**
\brief Calculates an axis aligned bounding box which is the union of two others.

\param bb1 First bounding box.
\param bb2 Second bounding box.

\return New bounding box representing the union.
*/
template <class AABBoxType>
AABBoxType
Union(const AABBoxType &bb1, const AABBoxType &bb2)
{
    AABBoxType bb;
    bb.m_min = Min(bb1.m_min, bb2.m_min);
    bb.m_max = Max(bb1.m_max, bb2.m_max);

    return bb;
}

/**
Calculates an axis aligned bounding box that contains a specified bounding box and a point.

\param bb1 First bounding box.
\param point a point.

\return New bounding box representing the union.
*/
template <class AABBoxType>
AABBoxType
Union(const AABBoxType &bb1, rwpmath::Vector3::InParam point)
{
    AABBoxType bb;
    bb.m_min = Min(bb1.m_min, point);
    bb.m_max = Max(bb1.m_max, point);

    return bb;
}


namespace rw
{
namespace collision
{

/**
\internal
\brief
Dummy structure. The AABBoxU is not an arena object so has no memory dump policy.

\importlib rwccore
*/
struct AABBoxUMemoryDumpPolicyInterface {};


/**
\brief
An axis aligned bounding box using unaligned  rw::math::fpu::Vector3U_32 to store the
minimum and maximum of the extent.



This is based on the  rw::collision::AABBoxTemplate, which provides the available API methods.
It is intended for offline use in the conditioning code, and is compatible with STL. To
convert to the aligned runtime version ( rw::collision::AABBox), use:

\code
    AABBoxU acBBox(min, max);
    AABBox  rtBBox(acBBox.Min(), acBBox.Max());
\endcode
*/
typedef AABBoxTemplate<rw::math::fpu::Matrix44AffineU_32,
                       rw::math::fpu::Vector3U_32,
                       AABBoxUMemoryDumpPolicyInterface> AABBoxU;


/**
\internal
\brief
Memory dump methods for the  rw::collision::AABBox type. This is a traits class for use with
the  rw::collision::AABBoxTemplate, providing a mechanism to treat the AABBox as an arena object.

\importlib rwccore
*/
struct AABBoxMemoryDumpPolicy
{
    template <class Archive>
    void Serialize(Archive &ar, uint32_t /*version*/);
};

/**
\brief An axis aligned bounding box using aligned and padded  rwpmath::Vector3 to store the
minimum and maximum of the extent.

This is based on the  rw::collision::AABBoxTemplate, which provides the available API methods.
It is intended for use in the runtime data and code. The  rw::collision::AABBoxU is an unaligned version,
more suited to offline use in conditioning.

Although they are normally embedded within other objects, it is possible to store an
AABBox as a separate arena object. The read/write callbacks can be registered with:
\code
    rw::collision::AABBox::RegisterArenaReadCallbacks()
    rw::collision::AABBox::RegisterArenaWriteCallbacks()
\endcode

When embedded in other arena objects, the  AABBox::Unfix, \ref AABBox::Fixup, and
 AABBox::Refix can be called directly from the corresponding fixup function of the
containing object.
*/
typedef AABBoxTemplate<rwpmath::Matrix44Affine,
                       rwpmath::Vector3,
                       AABBoxMemoryDumpPolicy> AABBox;



/**
Calculates an axis aligned bounding box that is the specified bounding box padded all around by given distance.

\param bb The bounding box.
\param pad The padding amount.

\return The padded bounding box.
*/
inline AABBox ComputePadded(const AABBox &bb1, rwpmath::VecFloatInParam pad)
{
    rwpmath::Vector3 padVec(pad, pad, pad);
    return AABBox(bb1.m_min - padVec, bb1.m_max + padVec);
}

} // namespace collision
} // namespace rw

template <>
inline RwpBool
AABBoxTemplate<rwpmath::Matrix44Affine, rwpmath::Vector3, rw::collision::AABBoxMemoryDumpPolicy>::Overlaps(const AABBoxTemplate &bb2) const
{
#if defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)

    vector unsigned int zero   = static_cast<vector unsigned int>(vec_splat_s32(0));
    vector unsigned int cmpgt1 = static_cast<vector unsigned int>(vec_cmpgt(m_min.GetVector(), bb2.m_max.GetVector()));
    vector unsigned int cmpgt2 = static_cast<vector unsigned int>(vec_cmpgt(bb2.m_min.GetVector(), m_max.GetVector()));
    vector unsigned int both   = vec_or(cmpgt1, cmpgt2);
    vector unsigned int chop   = vec_sld(zero,  both,  12); // remove 'w' field
    return static_cast<RwpBool>(vec_all_eq(zero, chop));

#elif defined(EA_PLATFORM_XENON)

    __vector4 zero   = __vspltisw(0);
    __vector4 cmpgt1 = __vcmpgtfp(m_min.GetVector(), bb2.m_max.GetVector());
    __vector4 cmpgt2 = __vcmpgtfp(bb2.m_min.GetVector(), m_max.GetVector());
    __vector4 both   = __vor(cmpgt1, cmpgt2);
    __vector4 chop   = __vsldoi(zero,  both,  12); // remove 'w' field
    return static_cast<RwpBool>(static_cast<rwpmath::Vector4>(zero) == static_cast<rwpmath::Vector4>(chop));

#else // rwpmath:: vectorized for other platforms

    rwpmath::Vector3 zero = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 temp = rwpmath::Max(m_min - bb2.m_max, bb2.m_min - m_max);
    temp = rwpmath::Max(temp, zero);
    return static_cast<RwpBool>(zero == temp);

#endif // defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)
}


template <>
inline RwpBool
AABBoxTemplate<rwpmath::Matrix44Affine, rwpmath::Vector3, rw::collision::AABBoxMemoryDumpPolicy>::Contains(const AABBoxTemplate &bb2) const
{
#if defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)

    vector unsigned int zero   = static_cast<vector unsigned int>(vec_splat_s32(0));
    vector unsigned int cmpgt1 = static_cast<vector unsigned int>(vec_cmpgt(bb2.m_max.GetVector(), m_max.GetVector()));
    vector unsigned int cmpgt2 = static_cast<vector unsigned int>(vec_cmpgt(m_min.GetVector(), bb2.m_min.GetVector()));
    vector unsigned int both   = vec_or(cmpgt1, cmpgt2);
    vector unsigned int chop   = vec_sld(zero,  both,  12); // remove 'w' field
    return static_cast<RwpBool>(vec_all_eq(zero, chop));

#elif defined(EA_PLATFORM_XENON)

    __vector4 zero   = __vspltisw(0);
    __vector4 cmpgt1 = __vcmpgtfp(bb2.m_max.GetVector(), m_max.GetVector());
    __vector4 cmpgt2 = __vcmpgtfp(m_min.GetVector(), bb2.m_min.GetVector());
    __vector4 both   = __vor(cmpgt1, cmpgt2);
    __vector4 chop   = __vsldoi(zero,  both,  12); // remove 'w' field
    return static_cast<RwpBool>(static_cast<rwpmath::Vector4>(zero) == static_cast<rwpmath::Vector4>(chop));

#else // rwpmath:: vectorized for other platforms

    rwpmath::Vector3 zero = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 temp = rwpmath::Max(m_min - bb2.m_min, bb2.m_max - m_max);
    temp = rwpmath::Max(temp, zero);
    return static_cast<RwpBool>(zero == temp);

#endif // defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)
}

/**
This is a specialization of the template for testing if one bbox contains another.
This has vector optimizations for PS3 and Xbox360.
\return TRUE if the point is contained in the bounding box.
*/
template <>
inline RwpBool
AABBoxTemplate<rwpmath::Matrix44Affine, rwpmath::Vector3, rw::collision::AABBoxMemoryDumpPolicy>::Contains(rwpmath::Vector3::InParam point) const
{
#if defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)

    vector unsigned int zero   = static_cast<vector unsigned int>(vec_splat_s32(0));
    vector unsigned int cmpgt1 = static_cast<vector unsigned int>(vec_cmpgt(point.GetVector(), m_max.GetVector()));
    vector unsigned int cmpgt2 = static_cast<vector unsigned int>(vec_cmpgt(m_min.GetVector(), point.GetVector()));
    vector unsigned int both   = vec_or(cmpgt1, cmpgt2);
    vector unsigned int chop   = vec_sld(zero,  both,  12); // remove 'w' field
    return static_cast<RwpBool>(vec_all_eq(zero, chop));

#elif defined(EA_PLATFORM_XENON)

    __vector4 zero   = __vspltisw(0);
    __vector4 cmpgt1 = __vcmpgtfp(point.GetVector(), m_max.GetVector());
    __vector4 cmpgt2 = __vcmpgtfp(m_min.GetVector(), point.GetVector());
    __vector4 both   = __vor(cmpgt1, cmpgt2);
    __vector4 chop   = __vsldoi(zero,  both,  12); // remove 'w' field
    return static_cast<RwpBool>(static_cast<rwpmath::Vector4>(zero) == static_cast<rwpmath::Vector4>(chop));

#else // rwpmath:: vectorized for other platforms

    rwpmath::Vector3 zero = rwpmath::GetVector3_Zero();
    rwpmath::Vector3 temp = rwpmath::Max(m_min - point, point - m_max);
    temp = rwpmath::Max(temp, zero);
    return static_cast<RwpBool>(zero == temp);

#endif // defined(EA_PLATFORM_PS3_PPU) || defined(EA_PLATFORM_PS3_SPU)
}

#endif // EAPHYSICS_BASE_HAS_AABBOX

#endif // PUBLIC_RWC_AABBOX_H
