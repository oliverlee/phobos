#include <cstring>
#include <iostream>
#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <asio/signal_set.hpp>
#include <google/protobuf/io/coded_stream.h>
#include "packet/frame.h"
#include "pose.pb.h"

namespace {
    asio::io_service io_service;
    asio::serial_port port(io_service);
    char rx_buffer[512];
    char pose_buffer[512];
    size_t pose_buffer_index = 0;
    BicyclePoseMessage pose;

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
            // TODO: handle multiple packet delimiters
            std::memcpy(pose_buffer + pose_buffer_index, rx_buffer, bytes_transferred);

            google::protobuf::io::CodedInputStream input(
                    (const uint8_t*)pose_buffer, bytes_transferred - 1);
            if (pose.MergeFromCodedStream(&input)) {
                pose.PrintDebugString();
                pose.Clear();
            }
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
    if (argc > 1) {
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
