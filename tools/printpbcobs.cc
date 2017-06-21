#include <cstring>
#include <fstream>
#include <iostream>
#include <google/protobuf/io/coded_stream.h>
#include "cobs.h"
#include "pose.pb.h"
#include "simulation.pb.h"

namespace {
    void deserialize_and_print_packet(const uint8_t * const packet_buffer_start, const size_t packet_buffer_length) {
        google::protobuf::io::CodedInputStream input(
            packet_buffer_start,
            packet_buffer_length
        );

        uint32_t size;
        if (!input.ReadVarint32(&size)) {
            std::cerr << "Unable to decode packet size from serialized data." << std::endl;
            exit(EXIT_FAILURE);
        }

        SimulationMessage msg;
        if (!msg.ParsePartialFromCodedStream(&input)) {
            std::cerr << "Unable to parse protobuf message." << std::endl;
            exit(EXIT_FAILURE);
        }

        if (!input.ConsumedEntireMessage()) {
            std::cerr << "Entire message not consumed. Number of extra bytes: " << input.BytesUntilLimit() << std::endl;
            exit(EXIT_FAILURE);
        }

        msg.PrintDebugString();
    }
}

int main(int argc, char* argv[]) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (argc < 1) {
        std::cerr << "Usage: " << argv[0] << " <file_path>\n\n";
        return EXIT_FAILURE;
    }

    std::ifstream ifs;
    ifs.open(argv[1], std::ios::binary | std::ios::in);

    constexpr size_t read_buffer_capacity = 4096;
    uint8_t read_buffer_start[read_buffer_capacity];
    uint8_t * read_buffer_write = read_buffer_start;
    uint8_t * const read_buffer_end = read_buffer_start + read_buffer_capacity;
    auto read_buffer_remaining = [&]() { return read_buffer_end - read_buffer_write; };

    constexpr size_t packet_buffer_capacity = 4096;
    uint8_t packet_buffer_start[packet_buffer_capacity];

    while (ifs.read(reinterpret_cast<char *>(read_buffer_write), read_buffer_remaining())) {
        size_t bytes_read = ifs.gcount();

        // Set the location of the next serial_buffer_write.
        read_buffer_write += bytes_read;

        // Decode as many packets as possible starting from read_buffer_start.
        const uint8_t * read_buffer = read_buffer_start;
        while (read_buffer < read_buffer_write) {

            // Compute the remaining length.
            const size_t bytes_remaining = read_buffer_write - read_buffer;

            // Create an output buffer and decode into it.
            cobs::DecodeResult result = cobs::decode(
                read_buffer,
                bytes_remaining,
                packet_buffer_start,
                packet_buffer_capacity
            );

            switch (result.status) {
                case cobs::DecodeResult::Status::OK: {
                    read_buffer += result.consumed;
                    // Decoded result.consumed bytes into result.produced bytes.
                    deserialize_and_print_packet(packet_buffer_start, result.produced);
                    continue;
                }
                case cobs::DecodeResult::Status::WRITE_OVERFLOW: {
                    std::cerr << "Package too large to decode." << std::endl;
                    exit(EXIT_FAILURE);
                }
                case cobs::DecodeResult::Status::READ_OVERFLOW: {
                    // Hitting a read overflow is usually not a problem. It just
                    // means that the encoded data is not fully available yet. If
                    // however we were decoding using the entire buffer and still
                    // hit a read overflow...
                    if (read_buffer == read_buffer_start && read_buffer_write == read_buffer_end) {
                        // ...we need a bigger serial_buffer.
                        std::cerr << "Encoded package does not fit in read buffer." << std::endl;
                        exit(EXIT_FAILURE);
                    }
                    break;
                }
                case cobs::DecodeResult::Status::UNEXPECTED_ZERO: {
                    read_buffer += result.consumed;
                    std::cout << "Unexpected zero, resuming decoding at next byte." << std::endl;
                    continue;
                }
                default: {
                    std::cerr << "Unknown DecodeResult::Status." << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
            // Break the loop by default.
            break;
        }

        // Shove remaining bytes in buffer back to the beginning, making sure
        // new data is written after these left-over bytes.
        const size_t bytes_remaining = read_buffer_write - read_buffer;
        std::memmove(read_buffer_start, read_buffer, bytes_remaining);
        read_buffer_write = read_buffer_start + bytes_remaining;
    }

    ifs.close();

    return EXIT_SUCCESS;
}
