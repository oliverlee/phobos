#include <exception>
#include <fstream>
#include <iostream>
#include <string>
#include <asio.hpp>
#include <asio/serial_port.hpp>
#include <asio/signal_set.hpp>

namespace {
    asio::io_service io_service;
    asio::serial_port port(io_service);
    std::fstream output;
    std::array<char, 512> receive_buffer;

    void handle_read(const asio::error_code&, size_t);

    void start_read() {
        port.async_read_some(asio::buffer(receive_buffer),
                std::bind(&handle_read,
                    std::placeholders::_1,
                    std::placeholders::_2));
    }

    void handle_read(const asio::error_code& error, size_t bytes_transferred) {
        if (error) {
            std::cerr << error.message() << "\n";
        } else {
            output.write(receive_buffer.data(), bytes_transferred);
        }
        start_read();
    }

    void handle_stop(const asio::error_code&, int signal_number) {
        if ((signal_number == SIGINT) || (signal_number == SIGTERM)) {
            io_service.stop();
            if (output.is_open()) {
                output.close();
            }
        }
    }
} // namespace

int main(int argc, char* argv[]) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <serial_device> <baud_rate> <log_file>\n"
            << "\nLog serial data to file.\n"
            << "A virtual serial port can be created using socat with:\n"
            << "\t$ socat -d -d pty,raw,echo=0 pty,raw,echo=0\n";
        return EXIT_FAILURE;
    }

    const std::string devname(argv[1]);
    const uint32_t baud_rate = std::atoi(argv[2]);
    const std::string filename(argv[3]);

    port.open(devname);
    port.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port.set_option(asio::serial_port_base::parity(
                asio::serial_port_base::parity::none));
    port.set_option(asio::serial_port_base::character_size(8));
    port.set_option(asio::serial_port_base::flow_control(
                asio::serial_port_base::flow_control::none));
    port.set_option(asio::serial_port_base::stop_bits(
                asio::serial_port_base::stop_bits::one));

    output.open(filename, std::ios::out | std::ios::binary | std::ios::trunc);

    asio::signal_set signals(io_service, SIGINT, SIGTERM);
    signals.async_wait(handle_stop);

    std::cout << "Logging data from " << devname << " to file " << filename << ".\n";
    std::cout << "Press Ctrl-C to terminate logging.\n";

    start_read();
    io_service.run();
    return EXIT_SUCCESS;
}
