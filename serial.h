#ifndef _SERIAL_HEADER_
#define _SERIAL_HEADER_

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

namespace serial
{

    class SerialPort
    {
        public:
            bool openPort(std::string portName);
            std::string readPort(void);
            bool closePort(void);
            SerialPort(void);
            
        private:
            boost::asio::io_service io;
            boost::asio::serial_port *port;
    };
}

#endif