#include "cobs.h"
#include "debug.h"

/**
Consistent Overhead Byte Stuffing (COBS) removes a specific value from a
list of values. The removed value can then, for example, be used as a
frame marker.

Refer to:
https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

## Notation

The character `x' is used to denote a single non-zero byte. To denote a
sequence of non-zero bytes, "Nx" is used where N is the number of bytes.
For example 4x denotes 4 consecutive non-zero bytes.

## Considerations:

1. Configurable marker byte value. The current implementation assumes
0x00.

2. Append the marker byte to the encoded result. The current
implementation appends the marker byte.

3. If the encoding results in [ ... | 255 | 254x | 1 | 0 ], encode it as
[ ... | 255 | 254x | 0 ]. This is possible because when we reach the
frame marker, the zero from the last offset is not appended. Since the
maximum offset does not append a zero either, the encoding can be
shortened. The same principle applies to encoding the empty byte array.
The COBS paper mentions this but does not implement it in their
examples. The current implementation does not apply this reduced
encoding because it requires keeping track of the last offset.

4. If the encoding results in [ ... | n | ... | x | 0 ] where n is the
last offset and x > n, encode it as [ ... | x | ... | 0 ]. The standard
implementation would run into an decoding error because x > n is bigger
than the remaining number of bytes. The current implementation does not
apply this reduced encoding for compatibility with other
implementations.

## Examples:

decoded, length -> encoded, length, note
       , 0      -> 0      , 1     , reduced (consideration 3)
       , 0      -> 1|0    , 2     , standard (consideration 3)
x      , 1      -> 2|x|0  , 3     ,
0      , 1      -> 1|1|0  , 3     ,
x|x    , 2      -> 3|x|x|0, 4     ,
x|0    , 2      -> 2|x|1|0, 4     ,
0|x    , 2      -> 1|2|x|0, 4     ,
0|0    , 2      -> 1|1|1|0, 4     ,

decoded, length -> encoded       , length, note
252x|0 , 253    -> 253|252x|1|0  , 255   ,
253x   , 253    -> 254|253x|0    , 255   ,
253x|0 , 254    -> 254|253x|1|0  , 256   ,
254x   , 254    -> 255|254x|0    , 256   , reduced (consideration 3)
254x   , 254    -> 255|254x|1|0  , 257   , standard (consideration 3)
254x|0 , 255    -> 255|254x|1|1|0, 258   ,
255x   , 255    -> 255|254x|2|x|0, 258   ,

decoded, length -> encoded                , length, note
506x|0 , 507    -> 255|254x|253|252x|1|0  , 510   ,
507x   , 507    -> 255|254x|254|253x|0    , 510   ,
507x|0 , 508    -> 255|254x|254|253x|1|0  , 511   ,
508x   , 508    -> 255|254x|255|254x|0    , 511   , reduced (consideration 3)
508x   , 508    -> 255|254x|255|254x|1|0  , 512   , standard (consideration 3)
508x|0 , 509    -> 255|254x|255|254x|1|1|0, 513   ,
509x   , 509    -> 255|254x|255|254x|1|x|0, 513   ,
*/

namespace cobs {
    /**
    Given the length of a decoded byte array, compute the maximum length
    the encoded byte array can attain.

    The maximum length is attained when the decoded byte array contains
    no zeros. The maximum length consists of the decoded byte array
    length, the offset overhead and the frame marker. It is computed as
    decoded_length + floor(decoded_length/254) + 2.

    This function does not handle decoded_length < 0.
    */
    size_t max_encoded_length(size_t decoded_length) {
        debug_assert(decoded_length >= 0, "decoded_length should be at least 0.");
        return decoded_length + decoded_length/0xfe + 2;
    }

    /**
    Given the length of a COBS encoded byte array, compute the maximum
    length the decoded byte array can attain.

    A case where the minimum overhead is achieved is when the decoded
    byte array consists only of marker bytes. This means that the
    maximum decoded length can be as big as the encoded length minus
    the packet overhead, consisting of the first offset and the marker
    byte. The maximum decoded length is thus given by encoded_length -
    2.

    This function does not handle encoded_length < 2.
    */
    size_t max_decoded_length(size_t encoded_length) {
        debug_assert(encoded_length >= 2, "encoded_length should be at least 2.");
        return encoded_length - 2;
    }

    /**
    COBS encode a byte array.
    */
    EncodeResult encode(const uint8_t * const src_start, size_t src_len, uint8_t * const dst_start, size_t dst_len) {
        const uint8_t * const src_end = src_start + src_len;
        const uint8_t * const dst_end = dst_start + dst_len;

        // This variable will always point to the next byte to read.
        const uint8_t * src = src_start;

        // Because this implementation copies bytes while seeking for
        // the next zero, we need to have two pointers. One to point to
        // the next location to copy bytes to and one to point to the
        // location where we should write the offset. Since dst_offset
        // is always < dst_copy when writing so we don't have to check
        // for a write overflow.
        uint8_t * dst_copy = dst_start;
        uint8_t * dst_offset = dst_copy++;

        auto bytes_produced = [&dst_copy, &dst_start]() -> size_t {
            // Can cast because dst_copy >= dst_start.
            return static_cast<size_t> (dst_copy - dst_start);
        };

        auto create_ok_status = [&bytes_produced]() -> EncodeResult {
            return EncodeResult { EncodeResult::Status::OK, bytes_produced() };
        };

        auto create_write_overflow_status = []() -> EncodeResult {
            return EncodeResult { EncodeResult::Status::WRITE_OVERFLOW, 0 };
        };

        while (src < src_end) {
            const uint8_t byte = *(src++);
            if (byte != 0x00) {
                // Append the data byte if possible.
                if (dst_copy >= dst_end) {
                    return create_write_overflow_status();
                }
                *(dst_copy++) = byte;

                // Unless we hit the maximum offset, keep copying.
                if ((dst_copy - dst_offset) != 0xff) continue;
            }

            // Write back the offset, set the offset index to the
            // current copy location and advance the copy index.
            *(dst_offset) = dst_copy - dst_offset;
            dst_offset = dst_copy++;
        }

        // Write back the offset. There is no need to update the pointer
        // anymore.
        *(dst_offset) = dst_copy - dst_offset;

        // Append the zero marker if possible. 
        if (dst_copy >= dst_end) {
            return create_write_overflow_status();
        }
        *(dst_copy++) = 0x00;

        return create_ok_status();
    }

    /**
    COBS decode a byte array.
    */
    DecodeResult decode(const uint8_t * const src_start, size_t src_len, uint8_t * const dst_start, size_t dst_len) {
        const uint8_t * const src_end = src_start + src_len;
        const uint8_t * const dst_end = dst_start + dst_len;

        // This variable will always point to the next byte to read.
        const uint8_t * src = src_start;

        // This variable will always point to the next byte to write.
        uint8_t * dst = dst_start;

        auto bytes_consumed = [&src, &src_start]() -> size_t {
            // Can cast because src >= src_start.
            return static_cast<size_t> (src - src_start);
        };

        auto bytes_produced = [&dst, &dst_start]() -> size_t {
            // Can cast because dst >= dst_start.
            return static_cast<size_t> (dst - dst_start);
        };

        auto create_ok_status = [&bytes_consumed, &bytes_produced]() -> DecodeResult {
            return DecodeResult { DecodeResult::Status::OK, bytes_consumed(), bytes_produced() };
        };

        auto create_unexpected_zero_status = [&bytes_consumed]() -> DecodeResult {
            return DecodeResult { DecodeResult::Status::UNEXPECTED_ZERO, bytes_consumed(), 0 };
        };

        auto create_read_overflow_status = []() -> DecodeResult {
            return DecodeResult { DecodeResult::Status::READ_OVERFLOW, 0, 0 };
        };

        auto create_write_overflow_status = []() -> DecodeResult {
            return DecodeResult { DecodeResult::Status::WRITE_OVERFLOW, 0, 0 };
        };

        // Read the first offset.
        if (src >= src_end) {
            return create_read_overflow_status();
        }
        uint8_t offset = *(src++);

        // If the first offset is 0x00 we can stop immediately.
        if (offset == 0x00) {
            return create_unexpected_zero_status();
        }

        while (true) {
            // Compute the location of the next zero. Subtract one from
            // offset to get the number of data bytes that follow.
            const uint8_t * const src_copy_end = src + offset - 1;
            const uint8_t * const dst_copy_end = dst + offset - 1;

            // Check if we can copy the data until the next zero. 
            if (src_copy_end > src_end) {
                return create_read_overflow_status();
            }
            if (dst_copy_end > dst_end) {
              return create_write_overflow_status();
            }

            // Copy data until we are at the next zero.
            while (src < src_copy_end) {
                const uint8_t byte = *(src++);

                // We should not encounter a zero in the encoded data.
                if (byte == 0x00) {
                    // Return control to the caller so it can restart a
                    // decoding operation from where we stop now.
                    return create_unexpected_zero_status();
                }

                *(dst++) = byte;
            }

            // Retrieve the next zero offset.
            if (src >= src_end) {
                return create_read_overflow_status();
            }
            const uint8_t next_offset = *(src++);

            // Check if we've hit the end.
            if (next_offset == 0x00) break;

            // If the last offset was not equal to 0xff and we have not
            // reached the end, output a zero.
            if (offset != 0xff) {
                if (dst >= dst_end) {
                    return create_write_overflow_status();
                }
                *(dst++) = 0x00;
            }

            // Store the offset for the next iteration.
            offset = next_offset;
        }

        return create_ok_status();
    }
}

