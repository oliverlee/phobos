#include <cstring>
#include <iostream>
#include <iomanip>
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
    char rx_buffer[buffer_size];
    char sim_buffer[buffer_size];
    char frame_buffer[buffer_size];
    size_t frame_buffer_index = 0;
    SimulationMessage msg;

    void handle_read(const asio::error_code&, size_t);

    void start_read() {
        port.async_read_some(asio::buffer(rx_buffer, 512),
                std::bind(&handle_read,
                    std::placeholders::_1,
                    std::placeholders::_2));
    }

    void handle_read(const asio::error_code& error, size_t bytes_transferred) {
        if (error) {
            std::cerr << error.message() << "\n";
        } else {
            std::cout << "read " << bytes_transferred << " bytes" << std::endl;

            std::cout << std::hex;
            for (size_t i = 0; i < bytes_transferred; ++i) {
                std::cout << std::setw(2) << ((int)rx_buffer[i] & 0xff) << " ";
            }
            std::cout << std::dec << std::endl;

            size_t start_index = 0;
            size_t scan_index = 0;

            auto copy_to_frame_buffer = [&]() -> void {
                size_t length = scan_index - start_index + 1; // delimiter included
                std::memcpy(frame_buffer + frame_buffer_index, rx_buffer + start_index, length);
                frame_buffer_index += length;
                start_index = ++scan_index;
            };

            auto decode_message = [&]() -> void {
                size_t unstuff_size = packet::frame::unstuff(frame_buffer, sim_buffer, frame_buffer_index);
                std::cout << "frame buffer index: " << frame_buffer_index << std::endl;
                std::cout << "unstuff size " << unstuff_size << std::endl;
                google::protobuf::io::CodedInputStream input(
                        (const uint8_t*)sim_buffer, frame_buffer_index);

                uint32_t size;
                if (!input.ReadVarint32(&size)) {
                    std::cerr << "Unable to decode packet size" << std::endl;
                    return;
                } else {
                    std::cout << "message size is " << size << std::endl;
                    if (frame_buffer_index < size) {
                        std::cout << "frame buffer index is smaller, skipping decode" << std::endl;
                        return;
                    }
                }
                if (msg.MergeFromCodedStream(&input)) {
                    msg.PrintDebugString();
                    msg.Clear();
                    frame_buffer_index = 0;
                    std::cout << std::endl;
                } else {
                    std::cout << "decode failed" << std::endl;
                }
            };

            while (scan_index < bytes_transferred) {
                if (rx_buffer[scan_index] == packet::frame::PACKET_DELIMITER) {
                    std::cout << "got packet delimiter at byte " << scan_index << std::endl;
                    copy_to_frame_buffer();
                    decode_message();
                } else {
                    ++scan_index;
                }
            }

            --scan_index;
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
        std::cerr << "Usage: " << argv[0] << " <serial_device> <baud_rate>\n"
            << "\nTest serial communication.\n"
            << "A virtual serial port can be created using socat with:\n"
            << "\t$ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n";
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
