#include <cstring>
#include <iomanip>
#include <iostream>
#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <asio/signal_set.hpp>
#include <google/protobuf/io/coded_stream.h>
#include "packet/frame.h"
#include "pose.pb.h"
#include "simulation.pb.h"

namespace {
    asio::io_service io_service;
    asio::serial_port port(io_service);

    constexpr size_t buffer_size = 1024;
    uint8_t rx_buffer[buffer_size];
    uint8_t sim_buffer[buffer_size];
    uint8_t frame_buffer[buffer_size];
    size_t frame_buffer_index = 0;

    SimulationMessage msg;

    void handle_read(const asio::error_code&, size_t);

    void start_read() {
        port.async_read_some(asio::buffer(rx_buffer, buffer_size),
                std::bind(&handle_read,
                    std::placeholders::_1,
                    std::placeholders::_2));
    }

    void handle_read(const asio::error_code& error, size_t bytes_transferred) {
        if (error) {
            std::cerr << error.message() << std::endl;
        } else {
            size_t start_index = 0;
            size_t scan_index = 0;

            auto copy_to_frame_buffer = [&]() -> void {
                const size_t length = scan_index - start_index + 1; /* include packet delimiter */
                if ((length + frame_buffer_index) > buffer_size) {
                    frame_buffer_index = 0;
                }
                std::memcpy(frame_buffer + frame_buffer_index, rx_buffer + start_index, length);
                frame_buffer_index += length;
                start_index = ++scan_index;
            };

            auto decode_message = [&]() -> void {
                const size_t unstuff_size = packet::frame::unstuff(
                        frame_buffer, sim_buffer, frame_buffer_index);
                google::protobuf::io::CodedInputStream input(
                        const_cast<const uint8_t*>(sim_buffer), unstuff_size);

                uint32_t size;
                if (!input.ReadVarint32(&size)) {
                    std::cerr << "Unable to decode packet size from serialized data." << std::endl;
                    return;
                } else {
                    if (frame_buffer_index < size) {
                        /* We haven't received enough data to decode this message */
                        return;
                    }
                }
                if (msg.MergeFromCodedStream(&input)) {
                    msg.PrintDebugString();
                    msg.Clear();

                    frame_buffer_index = 0;
                    std::cout << std::endl;

                    if (!input.ConsumedEntireMessage()) {
                        std::cerr << "Entire message not consumed. Number of extra bytes: " <<
                            input.BytesUntilLimit() << std::endl;
                    }
                } else {
                    std::cerr << "Decode of protobuf message failed." << std::endl;
                    std::cerr << "Current buffer size: " << frame_buffer_index << std::endl;
                    std::cerr << "Size after unstuffing: " << unstuff_size << std::endl;
                    std::cerr << "Buffer contents " << std::endl;
                    std::cerr << std::hex;
                    for (size_t i = 0; i < frame_buffer_index; ++i) {
                        std::cerr << std::setfill('0') << std::setw(2) << std::right <<
                            static_cast<int>(frame_buffer[i] & 0xff) <<  " ";
                    }
                    std::cerr << std::dec << std::endl;

                    if (unstuff_size > size) {
                        /* The size of the data stored in the buffer exceeds the
                         * size of the message obtained by decoding the first
                         * varint. We may have started reading in the middle of
                         * a message so throw away the data. */
                        frame_buffer_index = 0;
                    }
                }
            };

            while (scan_index < bytes_transferred) {
                if (rx_buffer[scan_index] == packet::frame::PACKET_DELIMITER) {
                    copy_to_frame_buffer();
                    decode_message();
                } else {
                    ++scan_index;
                }
            }

            --scan_index; /* don't copy an extra byte */
            copy_to_frame_buffer();
        }
        start_read();
    }

    void handle_stop(const asio::error_code&, int signal_number) {
        if ((signal_number == SIGINT) || (signal_number == SIGTERM)) {
            io_service.stop();
        }
    }
} // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <serial_device> [<baud_rate>]\n\n"
            << "Decode streaming serialized simulation protobuf messages.\n"
            << " <serial_device>      device from which to read serial data\n"
            << " <baud_rate=115200>   serial baud rate\n\n"

            << "A virtual serial port can be created using socat with:\n"
            << "  $ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n\n"
            << "This can be used to decode a log file of the serialized protobuf stream data.\n"
            << "Here is an example:\n"
            << "  $ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n"
            << "  2017/02/17 18:00:30 socat[71176] N PTY is /dev/ttys009\n"
            << "  2017/02/17 18:00:30 socat[71176] N PTY is /dev/ttys010\n"
            << "  2017/02/17 18:00:30 socat[71176] N starting data transfer loop with FDs [5,5] and [7,7]\n\n"
            << "  $ ./pbprint /dev/ttys010\n\n"
            << "  $ cat log.pb > /dev/ttys009\n";
        return EXIT_FAILURE;
    }

    const std::string devname(argv[1]);
    uint32_t baud_rate = 115200;
    if (argc > 2) {
        baud_rate = std::atoi(argv[2]);
    }

    port.open(devname);
    port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(asio::serial_port_base::parity(
                asio::serial_port_base::parity::none));
    port.set_option(asio::serial_port_base::character_size(8));
    port.set_option(asio::serial_port_base::flow_control(
                asio::serial_port_base::flow_control::none));
    port.set_option(asio::serial_port_base::stop_bits(
                asio::serial_port_base::stop_bits::one));


    asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait(handle_stop);

    std::cout << "Decoding protobuf files from " << devname << ".\n";
    std::cout << "Press Ctrl-C to terminate.\n";

    start_read();
    io_service.run();
    return EXIT_SUCCESS;
}
