// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://opencyphal.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.8.3 (serialization was enabled)
// Source file:   C:/Projekte/unimoc/public_regulated_data_types/reg/udral/physics/kinematics/geodetic/Point.0.1.dsdl
// Generated at:  2022-07-15 20:03:01.131147 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.udral.physics.kinematics.geodetic.Point
// Version:       0.1
//
// Platform
//     python_implementation:  CPython
//     python_version:  3.9.13
//     python_release_level:  final
//     python_build:  ('tags/v3.9.13:6de2ca5', 'May 17 2022 16:36:42')
//     python_compiler:  MSC v.1929 64 bit (AMD64)
//     python_revision:  6de2ca5
//     python_xoptions:  {}
//     runtime_platform:  Windows-10-10.0.19044-SP0
//
// Language Options
//     target_endianness:  little
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  True
//     enable_override_variable_array_capacity:  False
//     cast_format:  (({type}) {value})

#ifndef REG_UDRAL_PHYSICS_KINEMATICS_GEODETIC_POINT_0_1_INCLUDED_
#define REG_UDRAL_PHYSICS_KINEMATICS_GEODETIC_POINT_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <uavcan/si/unit/length/WideScalar_1_0.h>
#include <stdlib.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 434322821,
              "C:/Projekte/unimoc/public_regulated_data_types/reg/udral/physics/kinematics/geodetic/Point.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "C:/Projekte/unimoc/public_regulated_data_types/reg/udral/physics/kinematics/geodetic/Point.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 1,
              "C:/Projekte/unimoc/public_regulated_data_types/reg/udral/physics/kinematics/geodetic/Point.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 0,
              "C:/Projekte/unimoc/public_regulated_data_types/reg/udral/physics/kinematics/geodetic/Point.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_CAST_FORMAT == 2368206204,
              "C:/Projekte/unimoc/public_regulated_data_types/reg/udral/physics/kinematics/geodetic/Point.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.opencyphal.org/t/choosing-message-and-service-ids/889
#define reg_udral_physics_kinematics_geodetic_Point_0_1_HAS_FIXED_PORT_ID_ false

#define reg_udral_physics_kinematics_geodetic_Point_0_1_FULL_NAME_             "reg.udral.physics.kinematics.geodetic.Point"
#define reg_udral_physics_kinematics_geodetic_Point_0_1_FULL_NAME_AND_VERSION_ "reg.udral.physics.kinematics.geodetic.Point.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_udral_physics_kinematics_geodetic_Point_0_1_EXTENT_BYTES_                    24UL
#define reg_udral_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 24UL
static_assert(reg_udral_physics_kinematics_geodetic_Point_0_1_EXTENT_BYTES_ >= reg_udral_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

typedef struct
{
    /// saturated float64 latitude
    double latitude;

    /// saturated float64 longitude
    double longitude;

    /// uavcan.si.unit.length.WideScalar.1.0 altitude
    uavcan_si_unit_length_WideScalar_1_0 altitude;
} reg_udral_physics_kinematics_geodetic_Point_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_udral_physics_kinematics_geodetic_Point_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_udral_physics_kinematics_geodetic_Point_0_1_serialize_(
    const reg_udral_physics_kinematics_geodetic_Point_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    if ((8U * (size_t) capacity_bytes) < 192UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated float64 latitude
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 64ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- assume the native representation of float64 is conformant.
        static_assert(NUNAVUT_PLATFORM_IEEE754_DOUBLE, "Native IEEE754 binary64 required. TODO: relax constraint");
        static_assert(NUNAVUT_PLATFORM_IEEE754_DOUBLE, "Native IEEE754 binary64 required. TODO: relax constraint");
        (void) memmove(&buffer[offset_bits / 8U], &obj->latitude, 8U);
        offset_bits += 64U;
    }




    {   // saturated float64 longitude
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 64ULL) <= (capacity_bytes * 8U));
        // Saturation code not emitted -- assume the native representation of float64 is conformant.
        static_assert(NUNAVUT_PLATFORM_IEEE754_DOUBLE, "Native IEEE754 binary64 required. TODO: relax constraint");
        static_assert(NUNAVUT_PLATFORM_IEEE754_DOUBLE, "Native IEEE754 binary64 required. TODO: relax constraint");
        (void) memmove(&buffer[offset_bits / 8U], &obj->longitude, 8U);
        offset_bits += 64U;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad0_ > 0);
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += _pad0_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }

    {   // uavcan.si.unit.length.WideScalar.1.0 altitude
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits + 64ULL) <= (capacity_bytes * 8U));
        size_t _size_bytes0_ = 8UL;  // Nested object (max) size, in bytes.
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        NUNAVUT_ASSERT((offset_bits / 8U + _size_bytes0_) <= capacity_bytes);
        int8_t _err1_ = uavcan_si_unit_length_WideScalar_1_0_serialize_(
            &obj->altitude, &buffer[offset_bits / 8U], &_size_bytes0_);
        if (_err1_ < 0)
        {
            return _err1_;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        NUNAVUT_ASSERT((_size_bytes0_ * 8U) == 64ULL);
        offset_bits += _size_bytes0_ * 8U;  // Advance by the size of the nested object.
        NUNAVUT_ASSERT(offset_bits <= (capacity_bytes * 8U));
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad1_ = (uint8_t)(8U - offset_bits % 8U);
        NUNAVUT_ASSERT(_pad1_ > 0);
        const int8_t _err2_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad1_);  // Optimize?
        if (_err2_ < 0)
        {
            return _err2_;
        }
        offset_bits += _pad1_;
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.

    NUNAVUT_ASSERT(offset_bits == 192ULL);

    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (offset_bits / 8U);

    return NUNAVUT_SUCCESS;
}

/// Deserialize an instance from the provided buffer.
/// The lifetime of the resulting object is independent of the original buffer.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original buffer where possible.
///
/// @param obj      The object to update from the provided serialized representation.
///
/// @param buffer   The source buffer containing the serialized representation. There are no alignment requirements.
///                 If the buffer is shorter or longer than expected, it will be implicitly zero-extended or truncated,
///                 respectively; see Specification for "implicit zero extension" and "implicit truncation" rules.
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the supplied serialized
///                                 representation, in bytes. Upon return this value will be updated with the
///                                 size of the consumed fragment of the serialized representation (in bytes),
///                                 which may be smaller due to the implicit truncation rule, but it is guaranteed
///                                 to never exceed the original buffer size even if the implicit zero extension rule
///                                 was activated. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_udral_physics_kinematics_geodetic_Point_0_1_deserialize_(
    reg_udral_physics_kinematics_geodetic_Point_0_1* const out_obj, const uint8_t* buffer, size_t* const inout_buffer_size_bytes)
{
    if ((out_obj == NULL) || (inout_buffer_size_bytes == NULL) || ((buffer == NULL) && (0 != *inout_buffer_size_bytes)))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }
    if (buffer == NULL)
    {
        buffer = (const uint8_t*)"";
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    const size_t capacity_bits = capacity_bytes * (size_t) 8U;
    size_t offset_bits = 0U;





    // saturated float64 latitude
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    out_obj->latitude = nunavutGetF64(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 64U;




    // saturated float64 longitude
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    out_obj->longitude = nunavutGetF64(&buffer[0], capacity_bytes, offset_bits);
    offset_bits += 64U;


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    // uavcan.si.unit.length.WideScalar.1.0 altitude
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    {
        size_t _size_bytes1_ = (size_t)(capacity_bytes - nunavutChooseMin((offset_bits / 8U), capacity_bytes));
        NUNAVUT_ASSERT(offset_bits % 8U == 0U);
        const int8_t _err3_ = uavcan_si_unit_length_WideScalar_1_0_deserialize_(
            &out_obj->altitude, &buffer[offset_bits / 8U], &_size_bytes1_);
        if (_err3_ < 0)
        {
            return _err3_;
        }
        offset_bits += _size_bytes1_ * 8U;  // Advance by the size of the nested serialized representation.
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.
    NUNAVUT_ASSERT(offset_bits % 8U == 0U);
    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);
    NUNAVUT_ASSERT(capacity_bytes >= *inout_buffer_size_bytes);

    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_udral_physics_kinematics_geodetic_Point_0_1_initialize_(reg_udral_physics_kinematics_geodetic_Point_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_udral_physics_kinematics_geodetic_Point_0_1_deserialize_(out_obj, &buf, &size_bytes);
        NUNAVUT_ASSERT(err >= 0);
        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_UDRAL_PHYSICS_KINEMATICS_GEODETIC_POINT_0_1_INCLUDED_