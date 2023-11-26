// (c) Electronic Arts. All Rights Reserved.
#ifndef PUBLIC_RWC_AALINECLIPPER_H
#define PUBLIC_RWC_AALINECLIPPER_H

/*************************************************************************************************************

 File: rwcaalineclipper.hpp

 Purpose: Line clipping to axis aligned planes.
 */


#include "rw/collision/common.h"
#include "rw/collision/aabbox.h"


/**
\internal
 */
#define rwcAALINECLIPPER_FEPS        1e-6f


namespace rw
{
namespace collision
{


// ***********************************************************************************************************
//                                            AALineClipper CLASS
// ***********************************************************************************************************

/**
\internal

\brief Line clipper.

Structure used to cache information for a parametric line specifically when clipping
the line to many axis aligned planes. This structure represents a fattened infinite
line. Segments can be defined by additional start and end parameters.

\warning The \e  m_delta and \e  m_recip components are never allowed to be zero. Axis
aligned lines are skewed slightly to ensure this, with appropriate padding added so that
the resulting skewed fat line encloses the original line.

\importlib rwccore
 */
class AALineClipper
{
private:

    static inline rwpmath::VecFloat GetTolerance()
    {
#if defined EA_COMPILER_SN
#pragma diag_suppress=229,341,1363
#endif
        return rwpmath::VecFloat(rwcAALINECLIPPER_FEPS);
#if defined EA_COMPILER_SN
#pragma diag_default=229,341,1363
#endif
    }

public:
    /// Store data for access through fpu rather than vpu registers since mainly used for 
    /// decisions to branch based on fpu data which cause unnecessary flushes in vpu on Xenon.
    typedef rw::math::fpu::Vector3 Vector3Type;

    Vector3Type    m_origin;  ///< Origin for line definition.
    Vector3Type    m_delta;   ///< Vector along line from origin.
    Vector3Type    m_recip;   ///< Reciprocals of the delta x, y, z values.
    Vector3Type    m_padding; ///< Padding values in x, y, and z.
                                    ///  This represents a line fattened in positive and negative
                                    ///  direction by these values on each axis.
                                    ///  The resulting shape is equivalent to a swept box where the
                                    ///  padding is the half dimensions of the box.
    uint32_t       m_farBranch[3]; ///< Whether line is in positive or negative direction in each dimension.

#if defined(RWC_KDTREELINEQUERYBASE_OPT)
    rwpmath::Vector4    m_swizzled_data[3]; ///< the aalineclipper data swizzled so that
                                             ///  each Vector4 in this array is made up of the
                                             ///  x, y or z compoenents of origin, delta, recip and padding
                                             ///  This will become invalidated if any of the above
                                             ///  public members are altered
#endif // defined(RWC_KDTREELINEQUERYBASE_OPT)


    /**
    \internal
    \brief Initialize line info.

    \param start    Start position.
    \param end      End position.
    \param padding  Padding amount.
    */
    void
    Init(rwpmath::Vector3::InParam start,
         rwpmath::Vector3::InParam end,
         rwpmath::Vector3::InParam padding,
         const AABBox &bbox)
    {
        //  The following code deals with lines that are very close to having zero
        //  component in one or more axes. We actually skew such lines slightly, but
        //  fatten the padding so that the extent of the skewed line contains the original
        //  line. This all means we never have to have any special code paths for axis aligned
        //  lines (no div by zero etc). Clip parameters are more conservative for the skewed
        //  line so we never lose any intersections.

        rwpmath::Vector3 extentTol, hdelta, absHDelta, signHDelta, extentTolPad, offset;

        extentTol = rwpmath::Max(rwpmath::Abs(bbox.m_min), rwpmath::Abs(bbox.m_max));
        extentTol = rwpmath::Max(extentTol, rwpmath::Abs(start));
        extentTol *= GetTolerance();

        hdelta = (end - start) * rwpmath::GetVecFloat_Half();
        absHDelta = rwpmath::Abs(hdelta);
		signHDelta.Set(rwpmath::SgnNonZero(hdelta.X()), rwpmath::SgnNonZero(hdelta.Y()), rwpmath::SgnNonZero(hdelta.Z()));

        extentTolPad = rwpmath::Max(absHDelta, extentTol) - absHDelta;
        offset = Mult(signHDelta, extentTolPad);

        rwpmath::Vector3 origin = start - offset;
        rwpmath::Vector3 delta = end + offset - origin;
        m_origin = Vector3Type(origin);
        m_delta = Vector3Type(delta);
        EA_ASSERT_MSG( rwpmath::Abs(m_delta.X()) > rwpmath::GetVecFloat_MinValue(), ("Trying to divide by 0.") );
        EA_ASSERT_MSG( rwpmath::Abs(m_delta.Y()) > rwpmath::GetVecFloat_MinValue(), ("Trying to divide by 0.") );
        EA_ASSERT_MSG( rwpmath::Abs(m_delta.Z()) > rwpmath::GetVecFloat_MinValue(), ("Trying to divide by 0.") );
        m_recip = Vector3Type(rw::physics::mathutils::Reciprocal(delta));
        m_padding = Vector3Type(padding + extentTolPad);


#if defined(RWC_KDTREELINEQUERYBASE_OPT)
        // construct the swizzled data
        rwpmath::Matrix44 data_matrix(m_origin.GetVector(),
                                       m_delta.GetVector(),
                                       m_recip.GetVector(),
                                       m_padding.GetVector());
        m_swizzled_data[0] = data_matrix.GetXColumn();
        m_swizzled_data[1] = data_matrix.GetYColumn();
        m_swizzled_data[2] = data_matrix.GetZColumn();
#endif // defined(RWC_KDTREELINEQUERYBASE_OPT)

        m_farBranch[0] = uint32_t(m_recip.GetComponent(0) > 0.0f);
        m_farBranch[1] = uint32_t(m_recip.GetComponent(1) > 0.0f);
        m_farBranch[2] = uint32_t(m_recip.GetComponent(2) > 0.0f);

    }

    /**
    \internal

    \brief Construct line information from start and end points, plus padding value.

    \param start    Start position.
    \param end      End position.
    \param padding  Padding amount.
    */
    AALineClipper(rwpmath::Vector3::InParam start,
                  rwpmath::Vector3::InParam end,
                  rwpmath::Vector3::InParam padding,
                  const AABBox &bbox)
    {
        // Allow an epsilon amount of padding for float precision
        rwpmath::Vector3 epsPadding = rwpmath::Max(rwpmath::Abs(start), rwpmath::Abs(end)) * GetTolerance();
        Init(start, end, padding + epsPadding, bbox);
    }

    /**
    \internal

    \brief Construct line information from start and end points.

    \param start  Start position.
    \param end    End position.
    \param bbox   Bounding box of region in which clipping will take place.
    */
    AALineClipper(rwpmath::Vector3::InParam start,
                  rwpmath::Vector3::InParam end,
                  const AABBox &bbox)
    {
        // Allow an epsilon amount of padding for float precision
        rwpmath::Vector3 epsPadding = rwpmath::Max(rwpmath::Abs(start), rwpmath::Abs(end)) * GetTolerance();
        Init(start, end, epsPadding, bbox);
    }

    /**
    \internal
    \brief Clip line to an axis aligned bbox
    \param pa  Start parameter, before and after clipping.
    \param pb  End parameter, before and after clipping.
    \param bbox Bounding box of region in which clipping will take place.
    \return TRUE if line intersects bounding box.
    */
    RW_COLLISION_FORCE_INLINE RwpBool
    ClipToAABBox(float  &pa,
                 float  &pb,
                 const AABBox &bbox)
    {
        // Get line clip parameters for bbox interval on each axis
        Vector3Type  vpmin = Mult(m_recip, (Vector3Type(bbox.m_min) - m_padding - m_origin));
        Vector3Type  vpmax = Mult(m_recip, (Vector3Type(bbox.m_max) + m_padding - m_origin));
        Vector3Type  vp0 = Min(vpmin, vpmax);
        Vector3Type  vp1 = Max(vpmin, vpmax);

        // Accumulate clipping. If this makes pa>pb line does not intersect box.
        float pmax = rwpmath::Max(vp0.X(), vp0.Y(), vp0.Z());
        float pmin = rwpmath::Min(vp1.X(), vp1.Y(), vp1.Z());
        pa = rwpmath::Max(pa, pmax);
        pb = rwpmath::Min(pb, pmin);

        return static_cast<RwpBool>(pa < pb);
    }

};


} // namespace collision
} // namespace rw

#endif // PUBLIC_RWC_AALINECLIPPER_H
