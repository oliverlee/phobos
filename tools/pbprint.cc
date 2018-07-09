#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <asio/signal_set.hpp>
#include <google/protobuf/io/coded_stream.h>
#include "cobs.h"
#include "pose.pb.h"
#include "simulation.pb.h"

namespace {

    using std::size_t;

    asio::io_service io_service;
    asio::serial_port port(io_service);
    std::ifstream* input_file = nullptr;

    // serial port
    //   --[read]--> serial_buffer
    //   --[cobs decode]--> packet_buffer
    //   --[protobuf deserialize]--> message_object
    constexpr size_t SERIAL_BUFFER_CAPACITY = 2000;
    uint8_t serial_buffer_start[SERIAL_BUFFER_CAPACITY];
    uint8_t * serial_buffer_write = serial_buffer_start;
    uint8_t * const serial_buffer_end = serial_buffer_start + SERIAL_BUFFER_CAPACITY;

    void handle_read(const asio::error_code&, size_t);

    void start_read() {
        const size_t serial_buffer_remaining = serial_buffer_end - serial_buffer_write;
        port.async_read_some(
            asio::buffer(serial_buffer_write, serial_buffer_remaining),
            &handle_read
        );
    }

    void start_read(std::ifstream* ifs) {
        const size_t serial_buffer_remaining = serial_buffer_end - serial_buffer_write;

        ifs->read(reinterpret_cast<char*>(serial_buffer_write), serial_buffer_remaining);
        const size_t bytes_read = ifs->gcount();

        if (bytes_read > 0) {
            asio::error_code error;
            handle_read(error, bytes_read);
        }
    }

    void deserialize_packet(const uint8_t * const packet_buffer_start, const size_t packet_buffer_length) {
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

    void handle_read(const asio::error_code& error, size_t serial_buffer_write_count) {
        if (error) {
            std::cerr << error.message() << std::endl;
            exit(EXIT_FAILURE);
        }

        // Update the location we will write to next time. This is
        // simultaneously the end of the readable part of serial_buffer.
        serial_buffer_write += serial_buffer_write_count;

        // Decode as many packets as possible starting from serial_buffer_start.
        const uint8_t * serial_buffer = serial_buffer_start;
        while (serial_buffer < serial_buffer_write) {
            // Compute the remaining length.
            const size_t serial_buffer_len = serial_buffer_write - serial_buffer;

            // Create an output buffer and decode into it.
            std::array<uint8_t, 2000> packet_buffer;
            cobs::DecodeResult result = cobs::decode(
                serial_buffer,
                serial_buffer_len,
                packet_buffer.data(),
                packet_buffer.size()
            );

            switch (result.status) {
                case cobs::DecodeResult::Status::OK: {
                    serial_buffer += result.consumed;
                    // Decoded result.consumed bytes into result.produced bytes.
                    deserialize_packet(packet_buffer.data(), result.produced);
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
                    if (serial_buffer == serial_buffer_start && serial_buffer_write == serial_buffer_end) {
                        // ...we need a bigger serial_buffer.
                        std::cerr << "Encoded package does not fit in read buffer." << std::endl;
                        exit(EXIT_FAILURE);
                    }
                    break;
                }
                case cobs::DecodeResult::Status::UNEXPECTED_ZERO: {
                    serial_buffer += result.consumed;
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
        const size_t serial_buffer_remaining = serial_buffer_write - serial_buffer;
        std::memmove(serial_buffer_start, serial_buffer, serial_buffer_remaining);
        serial_buffer_write = serial_buffer_start + serial_buffer_remaining;

        // Start another read operation.
        if (input_file == nullptr) {
            start_read();
        } else {
            start_read(input_file);
        }
    }

    void handle_stop(const asio::error_code&, int signal_number) {
        if ((signal_number == SIGINT) || (signal_number == SIGTERM)) {
            io_service.stop();
        }
    }

} // namespace

int main(int argc, char* argv[]) {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_device> [<baud_rate>]\n\n"
            << "Decode streaming serialized simulation protobuf messages.\n"
            << " <serial_device>      device or file from which to read serial data\n"
            << " <baud_rate=115200>   serial baud rate\n"

            << "A virtual serial port can be created using socat with:\n"
            << "  $ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n\n"
            << "This can be used to decode a log file of the serialized protobuf stream data.\n"
            << "Here is an example:\n"
            << "  $ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n"
            << "  2017/02/17 18:00:30 socat[71176] N PTY is /dev/ttys009\n"
            << "  2017/02/17 18:00:30 socat[71176] N PTY is /dev/ttys010\n"
            << "  2017/02/17 18:00:30 socat[71176] N starting data transfer loop with FDs [5,5] and [7,7]\n\n"
            << "  $ ./pbprint /dev/ttys010\n\n"
            << "  $ cat log.pb.cobs > /dev/ttys009\n";
        return EXIT_FAILURE;
    }

    const std::string devname(argv[1]);
    uint32_t baud_rate = 115200;
    if (argc > 2) {
        baud_rate = std::atoi(argv[2]);
    }

    try {
        port.open(devname);
    } catch (const std::system_error& e) {
        // assume we have a file
        std::ifstream ifs(devname, std::ios::binary);
        input_file = &ifs;
        start_read(input_file);
        return EXIT_SUCCESS;
    }
    port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    port.set_option(asio::serial_port_base::character_size(8));
    port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

    asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait(handle_stop);
    start_read();
    io_service.run();

    return EXIT_SUCCESS;
}
