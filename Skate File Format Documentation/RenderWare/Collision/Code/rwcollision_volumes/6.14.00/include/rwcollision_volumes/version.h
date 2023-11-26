// (c) Electronic Arts. All Rights Reserved.
#ifndef RW_COLLISION_VOLUMES_VERSION_H
#define RW_COLLISION_VOLUMES_VERSION_H

#if defined _MSC_VER
#pragma once
#endif

/** @file version.h

This file provides a version number of the rwcollision_volumes library for your own code to check against.

Major, minor and patch versions are defined and updated each release.
*/

// Define the major, minor and patch versions.
// This information is updated with each release.

//! This define indicates the major version number for the package.
//! \sa RW_COLLISION_VOLUMES_VERSION
#define RW_COLLISION_VOLUMES_VERSION_MAJOR   6
//! This define indicates the minor version number for the package.
//! \sa RW_COLLISION_VOLUMES_VERSION
#define RW_COLLISION_VOLUMES_VERSION_MINOR   13
//! This define indicates the patch version number for the package.
//! \sa RW_COLLISION_VOLUMES_VERSION
#define RW_COLLISION_VOLUMES_VERSION_PATCH   0
//! This define can be used for convenience when printing the version number
//! \sa RW_COLLISION_VOLUMES_VERSION
#define RW_COLLISION_VOLUMES_VERSION_STRING "6.14.00"

/*!
 * This is a utility macro that users may use to create a single version number
 * that can be compared against RW_COLLISION_VOLUMES_VERSION.
 *
 * For example:
 *
 * \code
 *
 * #if RW_COLLISION_VOLUMES_VERSION > RW_COLLISION_VOLUMES_CREATE_VERSION_NUMBER( 1, 1, 0 )
 * printf("rwcollision_volumes version is greater than 1.1.0.\n");
 * #endif
 *
 * \endcode
 */
#define RW_COLLISION_VOLUMES_CREATE_VERSION_NUMBER( major_ver, minor_ver, patch_ver ) \
    ((major_ver) * 1000000 + (minor_ver) * 1000 + (patch_ver))

/*!
 * This macro is an aggregate of the major, minor and patch version numbers.
 * \sa RW_COLLISION_VOLUMES_CREATE_VERSION_NUMBER
 */
#define RW_COLLISION_VOLUMES_VERSION \
    RW_COLLISION_VOLUMES_CREATE_VERSION_NUMBER( RW_COLLISION_VOLUMES_VERSION_MAJOR, RW_COLLISION_VOLUMES_VERSION_MINOR, RW_COLLISION_VOLUMES_VERSION_PATCH )

#define RWCOLLISION_VOLUMES_HASSCALEDCLUSTEREDMESH 1

#endif // RW_COLLISION_VOLUMES_VERSION_H
