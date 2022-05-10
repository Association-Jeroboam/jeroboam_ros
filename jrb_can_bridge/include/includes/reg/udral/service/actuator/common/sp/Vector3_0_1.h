// This is an AUTO-GENERATED UAVCAN DSDL data type implementation. Curious? See https://opencyphal.org.
// You shouldn't attempt to edit this file.
//
// Checking this file under version control is not recommended unless it is used as part of a high-SIL
// safety-critical codebase. The typical usage scenario is to generate it as part of the build process.
//
// To avoid conflicts with definitions given in the source DSDL file, all entities created by the code generator
// are named with an underscore at the end, like foo_bar_().
//
// Generator:     nunavut-1.8.0 (serialization was enabled)
// Source file:   /home/thomas/git/CyphalDemo/public_regulated_data_types/reg/udral/service/actuator/common/sp/Vector3.0.1.dsdl
// Generated at:  2022-05-08 12:36:34.942614 UTC
// Is deprecated: no
// Fixed port-ID: None
// Full name:     reg.udral.service.actuator.common.sp.Vector3
// Version:       0.1
//
// Platform
//     python_implementation:  CPython
//     python_version:  3.10.4
//     python_release_level:  final
//     python_build:  ('main', 'Mar 31 2022 08:41:55')
//     python_compiler:  GCC 7.5.0
//     python_revision:
//     python_xoptions:  {}
//     runtime_platform:  Linux-5.4.0-109-generic-x86_64-with-glibc2.27
//
// Language Options
//     target_endianness:  little
//     omit_float_serialization_support:  False
//     enable_serialization_asserts:  False
//     enable_override_variable_array_capacity:  False
//     cast_format:  (({type}) {value})

#ifndef REG_UDRAL_SERVICE_ACTUATOR_COMMON_SP_VECTOR3_0_1_INCLUDED_
#define REG_UDRAL_SERVICE_ACTUATOR_COMMON_SP_VECTOR3_0_1_INCLUDED_

#include <nunavut/support/serialization.h>
#include <stdlib.h>
#include <string.h>

static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_TARGET_ENDIANNESS == 434322821,
              "/home/thomas/git/CyphalDemo/public_regulated_data_types/reg/udral/service/actuator/common/sp/Vector3.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_OMIT_FLOAT_SERIALIZATION_SUPPORT == 0,
              "/home/thomas/git/CyphalDemo/public_regulated_data_types/reg/udral/service/actuator/common/sp/Vector3.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_SERIALIZATION_ASSERTS == 0,
              "/home/thomas/git/CyphalDemo/public_regulated_data_types/reg/udral/service/actuator/common/sp/Vector3.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_ENABLE_OVERRIDE_VARIABLE_ARRAY_CAPACITY == 0,
              "/home/thomas/git/CyphalDemo/public_regulated_data_types/reg/udral/service/actuator/common/sp/Vector3.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );
static_assert( NUNAVUT_SUPPORT_LANGUAGE_OPTION_CAST_FORMAT == 2368206204,
              "/home/thomas/git/CyphalDemo/public_regulated_data_types/reg/udral/service/actuator/common/sp/Vector3.0.1.dsdl is trying to use a serialization library that was compiled with "
              "different language options. This is dangerous and therefore not allowed." );

#ifdef __cplusplus
extern "C" {
#endif

/// This type does not have a fixed port-ID. See https://forum.opencyphal.org/t/choosing-message-and-service-ids/889
#define reg_udral_service_actuator_common_sp_Vector3_0_1_HAS_FIXED_PORT_ID_ false

#define reg_udral_service_actuator_common_sp_Vector3_0_1_FULL_NAME_             "reg.udral.service.actuator.common.sp.Vector3"
#define reg_udral_service_actuator_common_sp_Vector3_0_1_FULL_NAME_AND_VERSION_ "reg.udral.service.actuator.common.sp.Vector3.0.1"

/// Extent is the minimum amount of memory required to hold any serialized representation of any compatible
/// version of the data type; or, on other words, it is the the maximum possible size of received objects of this type.
/// The size is specified in bytes (rather than bits) because by definition, extent is an integer number of bytes long.
/// When allocating a deserialization (RX) buffer for this data type, it should be at least extent bytes large.
/// When allocating a serialization (TX) buffer, it is safe to use the size of the largest serialized representation
/// instead of the extent because it provides a tighter bound of the object size; it is safe because the concrete type
/// is always known during serialization (unlike deserialization). If not sure, use extent everywhere.
#define reg_udral_service_actuator_common_sp_Vector3_0_1_EXTENT_BYTES_                    512UL
#define reg_udral_service_actuator_common_sp_Vector3_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_ 6UL
static_assert(reg_udral_service_actuator_common_sp_Vector3_0_1_EXTENT_BYTES_ >= reg_udral_service_actuator_common_sp_Vector3_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_,
              "Internal constraint violation");

/// Array metadata for: saturated float16[3] value
#define reg_udral_service_actuator_common_sp_Vector3_0_1_value_ARRAY_CAPACITY_           3U
#define reg_udral_service_actuator_common_sp_Vector3_0_1_value_ARRAY_IS_VARIABLE_LENGTH_ false

typedef struct
{
    /// saturated float16[3] value
    float value[3];
} reg_udral_service_actuator_common_sp_Vector3_0_1;

/// Serialize an instance into the provided buffer.
/// The lifetime of the resulting serialized representation is independent of the original instance.
/// This method may be slow for large objects (e.g., images, point clouds, radar samples), so in a later revision
/// we may define a zero-copy alternative that keeps references to the original object where possible.
///
/// @param obj      The object to serialize.
///
/// @param buffer   The destination buffer. There are no alignment requirements.
///                 @see reg_udral_service_actuator_common_sp_Vector3_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_
///
/// @param inout_buffer_size_bytes  When calling, this is a pointer to the size of the buffer in bytes.
///                                 Upon return this value will be updated with the size of the constructed serialized
///                                 representation (in bytes); this value is then to be passed over to the transport
///                                 layer. In case of error this value is undefined.
///
/// @returns Negative on error, zero on success.
static inline int8_t reg_udral_service_actuator_common_sp_Vector3_0_1_serialize_(
    const reg_udral_service_actuator_common_sp_Vector3_0_1* const obj, uint8_t* const buffer,  size_t* const inout_buffer_size_bytes)
{
    if ((obj == NULL) || (buffer == NULL) || (inout_buffer_size_bytes == NULL))
    {
        return -NUNAVUT_ERROR_INVALID_ARGUMENT;
    }


    const size_t capacity_bytes = *inout_buffer_size_bytes;
    if ((8U * (size_t) capacity_bytes) < 48UL)
    {
        return -NUNAVUT_ERROR_SERIALIZATION_BUFFER_TOO_SMALL;
    }
    // Notice that fields that are not an integer number of bytes long may overrun the space allocated for them
    // in the serialization buffer up to the next byte boundary. This is by design and is guaranteed to be safe.
    size_t offset_bits = 0U;





    {   // saturated float16[3] value
        const size_t _origin0_ = offset_bits;
        for (size_t _index0_ = 0U; _index0_ < 3UL; ++_index0_)
        {
            float _sat0_ = obj->value[_index0_];
            if (isfinite(_sat0_))
            {
                if (_sat0_ < ((float) -65504.0))
                {
                    _sat0_ = ((float) -65504.0);
                }
                if (_sat0_ > ((float) 65504.0))
                {
                    _sat0_ = ((float) 65504.0);
                }
            }
            const uint16_t _half0_ = nunavutFloat16Pack(_sat0_);
            (void) memmove(&buffer[offset_bits / 8U], &_half0_, 2U);
            offset_bits += 16U;
        }
        // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.
        (void) _origin0_;
    }


    if (offset_bits % 8U != 0U)  // Pad to 8 bits. TODO: Eliminate redundant padding checks.
    {
        const uint8_t _pad0_ = (uint8_t)(8U - offset_bits % 8U);
        const int8_t _err0_ = nunavutSetUxx(&buffer[0], capacity_bytes, offset_bits, 0U, _pad0_);  // Optimize?
        if (_err0_ < 0)
        {
            return _err0_;
        }
        offset_bits += _pad0_;
    }
    // It is assumed that we know the exact type of the serialized entity, hence we expect the size to match.




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
static inline int8_t reg_udral_service_actuator_common_sp_Vector3_0_1_deserialize_(
    reg_udral_service_actuator_common_sp_Vector3_0_1* const out_obj, const uint8_t* buffer, size_t* const inout_buffer_size_bytes)
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





    // saturated float16[3] value
    for (size_t _index1_ = 0U; _index1_ < 3UL; ++_index1_)
    {
        out_obj->value[_index1_] = nunavutGetF16(&buffer[0], capacity_bytes, offset_bits);
        offset_bits += 16U;
    }


    offset_bits = (offset_bits + 7U) & ~(size_t) 7U;  // Align on 8 bits.

    *inout_buffer_size_bytes = (size_t) (nunavutChooseMin(offset_bits, capacity_bits) / 8U);


    return NUNAVUT_SUCCESS;
}

/// Initialize an instance to default values. Does nothing if @param out_obj is NULL.
/// This function intentionally leaves inactive elements uninitialized; for example, members of a variable-length
/// array beyond its length are left uninitialized; aliased union memory that is not used by the first union field
/// is left uninitialized, etc. If full zero-initialization is desired, just use memset(&obj, 0, sizeof(obj)).
static inline void reg_udral_service_actuator_common_sp_Vector3_0_1_initialize_(reg_udral_service_actuator_common_sp_Vector3_0_1* const out_obj)
{
    if (out_obj != NULL)
    {
        size_t size_bytes = 0;
        const uint8_t buf = 0;
        const int8_t err = reg_udral_service_actuator_common_sp_Vector3_0_1_deserialize_(out_obj, &buf, &size_bytes);

        (void) err;
    }
}



#ifdef __cplusplus
}
#endif
#endif // REG_UDRAL_SERVICE_ACTUATOR_COMMON_SP_VECTOR3_0_1_INCLUDED_

