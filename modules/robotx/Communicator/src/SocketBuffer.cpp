#include "SocketBuffer.h"

namespace modules {
namespace robotx {

    using boost::asio::ip::tcp;
    using namespace std;

    SocketBuffer::SocketBuffer(boost::asio::io_service& io_service, boost::asio::ip::tcp::socket& socket)
        : io_service(io_service)
        , socket(socket)
    {
        setg(read_buffer, read_buffer, read_buffer);
    }

    int SocketBuffer::sync()
    {
        setg(read_buffer, read_buffer, read_buffer);
        return 0;
    }

    std::streamsize SocketBuffer::xsputn(const char_type *buf, std::streamsize n)
    {
        // Write a string to the socket
        if (socket.is_open())  {
            return socket.write_some(boost::asio::buffer(buf, n));
        }
        else {
            return 0;
        }
    }

    SocketBuffer::int_type SocketBuffer::sputc (char_type c)
    {
        if (socket.is_open()) {
            socket.write_some(boost::asio::buffer(&c, 1));
            return c;
        }
        else {
            return 0;
        }
    }

    SocketBuffer::int_type SocketBuffer::overflow(int_type c)
    {
        // Gets called with an additional character
        if (c !=  traits_type::eof() && socket.is_open()) {
            socket.write_some(boost::asio::buffer(&c, 1));
            return c;
        }
        return traits_type::eof();
    }

    SocketBuffer::int_type SocketBuffer::underflow()
    {
        if (gptr() < egptr()) {
            // Already data available
            return traits_type::to_int_type(*gptr());
        }

        bool finished = false;
        std::size_t  read=0;

        socket.async_receive(boost::asio::buffer(read_buffer,sizeof(read_buffer)),
                              0,
                              [&finished,&read,this](const boost::system::error_code& error, size_t bytes_transferred)
        {
            if (!error) {
                read = bytes_transferred;
            }
            else {
                socket.close();
            }
            finished = true;
        });

        while(!finished) {
            io_service.run_one();
            io_service.reset();

            if (!socket.is_open()) {
                break;
            }
        }

        if (!socket.is_open()) {
            return traits_type::eof();
        }

        // Set pointers
        setg(read_buffer, read_buffer, read_buffer+read);

        // Return the next character
        return traits_type::to_int_type(*gptr());
    }

}
}
