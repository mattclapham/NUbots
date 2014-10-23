#ifndef MODULES_ROBOTX_SOCKETBUFFER_H
#define MODULES_ROBOTX_SOCKETBUFFER_H

#include <thread>
#include <streambuf>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>

namespace modules {
namespace robotx {

    class SocketBuffer: public std::streambuf
    {

    private:
            char read_buffer[1024];
            boost::asio::io_service& io_service;
            boost::asio::ip::tcp::socket& socket;

    public:

        SocketBuffer( boost::asio::io_service& io_service, boost::asio::ip::tcp::socket& socket);
        virtual int_type underflow();
        virtual int_type overflow(int_type c);
        std::streamsize xsputn(const char_type *buf, std::streamsize n);
        int_type sputc (char_type c);
        virtual int sync();

    };

}
}
#endif // MODULES_ROBOTX_SOCKETBUFFER_H
