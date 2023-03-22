//
// Copyright (c) 2011 Christopher Baker <https://christopherbaker.net>
// Copyright (c) 2011 Jacques Fortier <https://github.com/jacquesf/COBS-Consistent-Overhead-Byte-Stuffing>
//
// SPDX-License-Identifier: MIT
//


#pragma once



/// \brief A Consistent Overhead Byte Stuffing (COBS) Encoder.
///
/// Consistent Overhead Byte Stuffing (COBS) is an encoding that removes all 0
/// bytes from arbitrary binary data. The encoded data consists only of bytes
/// with values from 0x01 to 0xFF. This is useful for preparing data for
/// transmission over a serial link (RS-232 or RS-485 for example), as the 0
/// byte can be used to unambiguously indicate packet boundaries. COBS also has
/// the advantage of adding very little overhead (at least 1 byte, plus up to an
/// additional byte per 254 bytes of data). For messages smaller than 254 bytes,
/// the overhead is constant.
///
/// \sa http://conferences.sigcomm.org/sigcomm/1997/papers/p062.pdf
/// \sa http://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
/// \sa https://github.com/jacquesf/COBS-Consistent-Overhead-Byte-Stuffing
/// \sa http://www.jacquesf.com/2011/03/consistent-overhead-byte-stuffing
class COBS
{
public:
    /// \brief Encode a byte buffer with the COBS encoder.
    /// \param buffer A pointer to the unencoded buffer to encode.
    /// \param size  The number of bytes in the \p buffer.
    /// \param encodedBuffer The buffer for the encoded bytes.
    /// \returns The number of bytes written to the \p encodedBuffer.
    /// \warning The encodedBuffer must have at least getEncodedBufferSize() 
    ///          allocated.
    static size_t encode(const uint8_t* buffer,
                         size_t size,
                         uint8_t* encodedBuffer)
    {
        size_t read_index  = 0;
        size_t write_index = 1;
        size_t code_index  = 0;
        uint8_t code       = 1;

        while (read_index < size)
        {
            if (buffer[read_index] == 0)
            {
                encodedBuffer[code_index] = code;
                code = 1;
                code_index = write_index++;
                read_index++;
            }
            else
            {
                encodedBuffer[write_index++] = buffer[read_index++];
                code++;

                if (code == 0xFF)
                {
                    encodedBuffer[code_index] = code;
                    code = 1;
                    code_index = write_index++;
                }
            }
        }

        encodedBuffer[code_index] = code;

        return write_index;
    }


    /// \brief Decode a COBS-encoded buffer.
    /// \param encodedBuffer A pointer to the \p encodedBuffer to decode.
    /// \param size The number of bytes in the \p encodedBuffer.
    /// \param decodedBuffer The target buffer for the decoded bytes.
    /// \returns The number of bytes written to the \p decodedBuffer.
    /// \warning decodedBuffer must have a minimum capacity of size.
    static size_t decode(const uint8_t* encodedBuffer,
                         size_t size,
                         uint8_t* decodedBuffer)
    {
        if (size == 0)
            return 0;

        size_t read_index  = 0;
        size_t write_index = 0;
        uint8_t code       = 0;
        uint8_t i          = 0;

        while (read_index < size)
        {
            code = encodedBuffer[read_index];

            if (read_index + code > size && code != 1)
            {
                return 0;
            }

            read_index++;

            for (i = 1; i < code; i++)
            {
                decodedBuffer[write_index++] = encodedBuffer[read_index++];
            }

            if (code != 0xFF && read_index != size)
            {
                decodedBuffer[write_index++] = '\0';
            }
        }

        return write_index;
    }

    /// \brief Get the maximum encoded buffer size for an unencoded buffer size.
    /// \param unencodedBufferSize The size of the buffer to be encoded.
    /// \returns the maximum size of the required encoded buffer.
    static size_t getEncodedBufferSize(size_t unencodedBufferSize)
    {
        return unencodedBufferSize + unencodedBufferSize / 254 + 1;
    }

};
