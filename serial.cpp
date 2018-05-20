#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include <string>
#include <iostream>

#include "serial.h"

namespace serial
{
    
    namespace asio = boost::asio;
    
    SerialPort::SerialPort(void) 
    { 
        port = new asio::serial_port(io);
    }
    
    bool SerialPort::openPort(std::string portName)
    {
        port->open(portName.c_str());
        port->set_option(asio::serial_port_base::baud_rate(115200));
        
        return true;
    }
    
    std::string SerialPort::readPort(void)
    {
        asio::streambuf buf;
        asio::read_until(*port, buf, "\n");
        
        std::istream is(&buf);
        std::string s;
        std::getline(is, s);
        
        return s;
    }
    
    bool SerialPort::closePort(void)
    {
        port->close();
        
        return true;
    }
}