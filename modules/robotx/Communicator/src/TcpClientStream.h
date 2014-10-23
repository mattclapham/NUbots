#ifndef TCPCLIENTSTREAM_H
#define TCPCLIENTSTREAM_H

#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "SocketBuffer.h"

namespace modules {
namespace robotx {

    class TcpClientStream: public std::iostream
    {
    public:
        enum ConnectionStatus  { DISCONNECTED=0, CONNECTED=1, CONNECTING=2 };

    private:
        bool stop_thread;
        ConnectionStatus connection_status;
        boost::asio::io_service io_service;
        boost::asio::ip::tcp::resolver resolver;
        boost::asio::ip::tcp::socket socket;
        SocketBuffer read_buffer;
        std::shared_ptr<std::thread> worker_thread;
        boost::asio::ip::tcp::resolver::iterator endpoint_iterator;

    public:
        TcpClientStream();
        void init(std::string host, std::string port);
        ~TcpClientStream();

        void close();
        void connect();
        void disconnect();
        bool isConnected();
        ConnectionStatus connectionStatus();

    protected:
        void doWork();
    };

}
}
#endif // TCPCLIENTSTREAM_H
