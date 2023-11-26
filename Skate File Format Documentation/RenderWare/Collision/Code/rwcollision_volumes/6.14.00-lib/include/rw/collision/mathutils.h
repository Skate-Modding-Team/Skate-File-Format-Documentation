// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_MATHUTILS_H
#define RWCOLLISION_MATHUTILS_H

#include "rw/collision/common.h"

#if !defined(RWP_NO_VPU_MATH)
#include "rw/math/vpu/vector_intrinsic.h"
#include "rw/math/vpu/mask4_operation.h"
#include "rw/math/vpl.h"
#endif 

namespace rw
{
namespace physics
{
namespace mathutils
{

template <class T>  
RW_COLLISION_FORCE_INLINE T
MultAdd(const T &a,
        const T &b,
        const T &c)
{
#if defined(RWP_NO_VPU_MATH)
    return a * b + c;
#else //RWP_NO_VPU_MATH
    return rw::math::MultAdd(a,b ,c);
#endif
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector3
Reciprocal(rwpmath::Vector3::InParam a)
{
#if RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER( 1, 5, 0 )  //rwmath only supports Vector Reciprocal after 1.5.0
#if defined(RWP_NO_VPU_MATH)
    return rwpmath::GetVector3_One() / a;
#else //RWP_NO_VPU_MATH
    return rwpmath::_asmReciprocal(a.GetVector());
#endif
#else //#if RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER( 1, 5, 0 )  
    return rwpmath::Reciprocal(a);
#endif //#if RW_MATH_VERSION < RW_MATH_CREATE_VERSION_NUMBER( 1, 5, 0 )
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector3
InvSqrt(rwpmath::Vector3::InParam a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Vector3 result(rwpmath::InvSqrt(a.GetX()),
                            rwpmath::InvSqrt(a.GetY()),
                            rwpmath::InvSqrt(a.GetZ()));
    return result;
#else //RWP_NO_VPU_MATH
    using namespace rwpmath;
    VectorIntrinsic result;
    RW_MATH_VPU_ReciprocalSquareRoot( result, a.mV );
    return Vector3(result);
#endif
}


template <class T>  
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ScalarToVector3(T a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Vector3 result(a, a, a);
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Vector3(a.GetVector());
#endif
}


template <class T>  
RW_COLLISION_FORCE_INLINE rwpmath::Vector3
ToVector3(T a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Vector3 result(a.GetX(), a.GetY(), a.GetZ());
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Vector3(a.GetVector());
#endif
}


#if !defined(RWP_NO_VPU_MATH)
RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ScalarToVector4(rwpmath::VecFloatRefX a)
{
    using namespace rw::math::vpu;
    VectorIntrinsic ret;
    RW_MATH_VPU_Splat(ret, (a).mV, 0);
    return Vector4(ret);
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ScalarToVector4(rwpmath::VecFloatRefY a)
{
    using namespace rw::math::vpu;
    VectorIntrinsic ret;
    RW_MATH_VPU_Splat(ret, (a).mV, 1);
    return Vector4(ret);
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ScalarToVector4(rwpmath::VecFloatRefZ a)
{
    using namespace rw::math::vpu;
    VectorIntrinsic ret;
    RW_MATH_VPU_Splat(ret, (a).mV, 2);
    return Vector4(ret);
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ScalarToVector4(rwpmath::VecFloatRefW a)
{
    using namespace rw::math::vpu;
    VectorIntrinsic ret;
    RW_MATH_VPU_Splat(ret, (a).mV, 3);
    return Vector4(ret);
}
#endif //!defined(RWP_NO_VPU_MATH)


template <class T>  
RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ScalarToVector4(T a)
{
    rwpmath::Vector4 result(a, a, a, a);
    return result;
}


RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ToVector4(rwpmath::Vector3::InParam a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Vector4 result(a.GetX(),
        a.GetY(),
        a.GetZ(),
        1.0f);
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Vector4(a.GetVector());
#endif
}


template <class T>  
RW_COLLISION_FORCE_INLINE rwpmath::Vector4
ToVector4(T a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Vector4 result(a.GetX(),
                            a.GetY(),
                            a.GetZ(),
                            a.GetW());
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Vector4(a.GetVector());
#endif
}


template <class T>  
RW_COLLISION_FORCE_INLINE rwpmath::Quaternion
ToQuaternion(T a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Quaternion result(a.GetX(),
                               a.GetY(),
                               a.GetZ(),
                               a.GetW());
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Quaternion(a.GetVector());
#endif
}


RW_COLLISION_FORCE_INLINE rwpmath::Mask3
ToMask3(rwpmath::MaskScalar::InParam a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Mask3 result(a,
                          a,   
                          a);
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Mask3(a.mV);
#endif
}


template <class T>  
RW_COLLISION_FORCE_INLINE rwpmath::Mask3
ToMask3(T a)
{
#if defined(RWP_NO_VPU_MATH)
    rwpmath::Mask3 result(a.GetX(),
                          a.GetY(),
                          a.GetZ());
    return result;
#else //RWP_NO_VPU_MATH
    return rwpmath::Mask3(a.GetVector());
#endif
}


inline rwpmath::VecFloat ClampMagnitude(rwpmath::VecFloatInParam x, rwpmath::VecFloatInParam h)
{
#if defined EA_PLATFORM_PS3_SPU
    return rwpmath::VecFloat(spu_sel(x.mV, h.mV, spu_rlmaska(spu_cmpabsgt(x.mV, h.mV), -1)));
#else
    return rwpmath::Clamp(x, -h, h);
#endif
}


inline rwpmath::VecFloat ClampUnordered(rwpmath::VecFloatInParam x, rwpmath::VecFloatInParam a, rwpmath::VecFloatInParam b)
{
    rwpmath::MaskScalar c = rwpmath::CompGreaterThan(x, b);
    return Select(Xor(rwpmath::CompGreaterThan(x, a), c), x, Select(Xor(rwpmath::CompGreaterThan(a, b), c), b, a));
}


inline rwpmath::MaskScalar TestRangeUnordered(rwpmath::VecFloatInParam x, rwpmath::VecFloatInParam a, rwpmath::VecFloatInParam b)
{
    return Xor(rwpmath::CompGreaterThan(x, a), rwpmath::CompGreaterThan(x, b));
}


inline rwpmath::VecFloat ReplaceSign(rwpmath::VecFloatInParam x, rwpmath::VecFloatInParam y)
{
#if defined EA_PLATFORM_PS3_PPU
    return Select(rwpmath::MaskScalar((vector float)vec_sl(vec_splat_u32(-1), vec_splat_u32(-1))), y, x);
#elif defined EA_PLATFORM_XENON
    return Select(rwpmath::MaskScalar(__vslw(__vspltisw(-1), __vspltisw(-1))), y, x);
#elif RWPMATH_IS_VPU
    return Select(rwpmath::MaskScalar(0x80000000), y, x);
#else
    return Select(rwpmath::CompGreaterThan(y, rwpmath::GetVecFloat_Zero()), rwpmath::Abs(x), -rwpmath::Abs(x));
#endif
}


}
}
}



#endif // RWCOLLISION_MATHUTILS_H
