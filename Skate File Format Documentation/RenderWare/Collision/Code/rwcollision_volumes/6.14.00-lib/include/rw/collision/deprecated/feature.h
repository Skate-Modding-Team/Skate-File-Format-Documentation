// (c) Electronic Arts. All Rights Reserved.
#ifndef RWCOLLISION_VOLUMES_DEPRECATED_FEATURE_H
#define RWCOLLISION_VOLUMES_DEPRECATED_FEATURE_H

#include "rw/collision/common.h"

namespace rw
{
    namespace collision
    {

        const rwpmath::VecFloat VEC_EPSILON(rwpmath::EPSILON); 
        const rwpmath::VecFloat VEC_EPSILON_SQUARED = VEC_EPSILON * VEC_EPSILON; 

        /**
        \brief Legacy parametric edge structure, used for building primitive features
        \deprecated The rwcollision_volumes package no longer creates objects of this class 
                    and it will be removed from a subsequent release.
        \importlib rwccore
        */
        struct FeatureEdge
        {
            rwpmath::Vector3 base;     ///< Base end point of the edge.
            rwpmath::Vector3 dir;      ///< Unit direction vector to the other endpoint specific direction.
            rwpmath::Vector3 pn;       ///< \brief Unit direction vector perpendicular to the edge.
            ///< It is the cross product of the edge and the direction of the feature
            ///< query (such as the face normal, in the case of a face feature).
            rwpmath::VecFloat length; ///< Length of the edge.

            /**

            \internal
            */
            inline FeatureEdge()
            {
            }

            /**
            \internal
            \param p1 an end point of the edge
            \param p2 an end point of the edge
            */
            inline FeatureEdge( const rwpmath::Vector3 &p1,
                const rwpmath::Vector3 &p2 )
            {
                base = p1;
                rwpmath::Vector3 delta = p2 - p1;
                rwpmath::VecFloat lengthSq = MagnitudeSquared(delta);

                rwpmath::MaskScalar nonZero = rwpmath::CompGreaterThan(lengthSq, VEC_EPSILON);
                length = rwpmath::Select(nonZero, rwpmath::SqrtFast(lengthSq), rwpmath::GetVecFloat_Zero());
                dir = rwpmath::Select(nonZero, delta * rwpmath::ReciprocalFast(length), rwpmath::GetVector3_Zero());
            }

            /**
            \internal
            \param newbase base end point of the edge
            \param newdir direction to the other endpoint
            \param newlength length of the edge
            */
            FeatureEdge( const rwpmath::Vector3 & newbase, const rwpmath::Vector3 & newdir,
                const rwpmath::VecFloat & newlength )
            {
                base = newbase;
                dir = newdir;
                length = newlength;
            }

            /**
            \internal
            \param newbase base end point of the edge
            \param newdir unit direction vector to the other endpoint
            \param newpn unit direction vector, the cross product of the edge and the feature query direction.
            \param newlength length of the edge
            */
            FeatureEdge( const rwpmath::Vector3 & newbase, const rwpmath::Vector3 & newdir,
                const rwpmath::Vector3 & newpn, const rwpmath::VecFloat & newlength )
            {
                base = newbase;
                dir = newdir;
                pn = newpn;
                length = newlength;
            }

            /**
            \internal
            Compute the pn vector which is the cross product of the edge direction and the feature query direction.
            \param extrusion_dir the direction of the feature query
            */
            inline void
                build_plane( const rwpmath::Vector3 &extrusion_dir )
            {
                pn = rwpmath::Cross( dir, extrusion_dir );
                rwpmath::VecFloat lengthSq = MagnitudeSquared(pn);
                pn *= rwpmath::Select(rwpmath::CompGreaterThan(lengthSq, VEC_EPSILON), rwpmath::InvSqrtFast(lengthSq),rwpmath::GetVecFloat_Zero());
            }

            /**
            \internal
            Gets the end point of the edge
            \return the end point which is the base point + edge direction * length.
            */
            rwpmath::Vector3 endpoint() const
            {
                return base + dir * length;
            }

            /**
            \internal
            Gets the midpoint of the edge.
            \return the midpoint which is the base point + edge direction * length * 0.5
            */
            rwpmath::Vector3 midpoint() const
            {
                return base + dir * length * rwpmath::GetVecFloat_Half();
            }

            /**
            \internal

            \brief Constrain the point to the edge feature.

            This returns the point on the edge that is nearest to the given point.   The given point is projected
            onto the edge line, and clamped to the edge segment.

            \param pt a point for which the nearest point on the edge is found.

            \Return clamping region, 1=base point, 2=edge, 3=endpoint.
            */

            uint32_t
                constrain_point( rwpmath::Vector3 &pt ) const
            {
                rwpmath::VecFloat pt_t = rwpmath::Dot( pt - base, dir );

                if (pt_t < rwpmath::GetVecFloat_Zero())
                {
                    pt = base;
                    return 1;
                }
                if (pt_t > length)
                {
                    pt = endpoint();
                    return 3;
                }
                pt = base + dir * pt_t;
                return 2;
            }

        };

        /**
        \brief Legacy feature on a primitive returned from collision calculation.
        \see FeatureEdge
        \deprecated The rwcollision_volumes package no longer creates objects of this class
                    and it will be removed from a subsequent release.
        \importlib rwccore
        */
        struct Feature
        {
            /**
            */
            enum
            {
                MAXEDGECOUNT = 8   ///< the maximum number of edges that can be stored in a feature.
            };

            /** \brief when finding the intersection of two features, one or both may be clamped to a
            sub-feature. If so, the region is set to the sub-feature index.
            Region=0 means face.  Region=1,3,5,... means a vertex.
            Region=2,4,6,... means an edge.
            */
            uint32_t             region;

            FeatureEdge          edges[MAXEDGECOUNT];  ///< An array of up to 4 edges which define the feature
            rwpmath::Vector3    ownNormal; ///< the normal of the feature, if it is a face

            rwpmath::Vector3    pt;        ///< the location of the feature, if it is a single point

            int32_t              numedges;  ///< number of edges, 0=point, 1=edge, 2 or more = face.

            /**
            \internal
            \brief Computes the center of a feature.

            The center of a point is the point.  The center of an edge is the midpoint.
            The center of a face is the centroid, which is the average of all the vertices.
            \return the center point of the feature.
            */
            rwpmath::Vector3 Center()
            {
                if ( numedges == 0 )    // point feature
                {
                    return pt;
                }
                rwpmath::Vector3 c(0,0,0);
                for ( int32_t i = 0;  i < numedges;  i++ )
                {
                    c += edges[i].base + edges[i].endpoint();
                }
#if defined EA_COMPILER_SN
#pragma diag_suppress=229,341,1363
#endif
                return (c / rwpmath::VecFloat( numedges * 2.0f ));
#if defined EA_COMPILER_SN
#pragma diag_default=229,341,1363
#endif
            }

            /**
            \internal
            \brief Gets the mapped type of the feature.

            In the primitive pair code, the feature pairs are categorized by the type of the pair, and the sum of
            the mapped type of the two features is used to categorize the pair.   The mapped type is 0=point, 1=edge,
            and 3=face.   You can see that a the pairs will have these sums.
            \li 0 point-point
            \li 1 point-edge
            \li 2 edge-edge
            \li 3 point-face
            \li 4 edge-face
            \li 6 face-face
            \return the mapped type of the feature, 0=point, 1=edge, and 3=face.
            */
            int32_t MappedType()
            {
                return (numedges > 1 ? 3 : (numedges & 1));
            }

            /**
            \internal
            \brief Build the edge plane vector for each edge.

            For a face feature, the edge planes are the boundaries of the voronoi region of the face.
            \param ccw counter-clockwise winding.  If false, the extrusion_dir is negated.
            \param extrusion_dir the direction of the feature query
            \see FeatureEdge::build_plane
            */
            void BuildEdgePlanes( RwpBool ccw, const rwpmath::Vector3 &extrusion_dir )
            {
                if ( ccw )
                {
                    for ( int32_t i = 0;  i < numedges;  i++ )
                    {
                        edges[i].build_plane( extrusion_dir );
                    }
                }
                else
                {
                    for ( int32_t i = 0;  i < numedges;  i++ )
                    {
                        edges[i].build_plane( -extrusion_dir );
                    }
                }
            }

            /**
            \internal
            \brief Build the edge plane vector for each edge.

            For a face feature, the edge planes are the boundaries of the voronoi region of the face.
            \param extrusion_dir the direction of the feature query
            \see FeatureEdge::build_plane
            */
            void BuildEdgePlanes( const rwpmath::Vector3 &extrusion_dir )
            {
                for ( int32_t i = 0;  i < numedges;  i++ )
                {
                    edges[i].build_plane( extrusion_dir );
                }
            }
        };

    } // namespace collision
} // namespace rw

#endif // RWCOLLISION_VOLUMES_DEPRECATED_FEATURE_H
