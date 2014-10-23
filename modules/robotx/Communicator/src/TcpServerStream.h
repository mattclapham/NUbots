#ifndef TCPSERVERSTREAM_H
#define TCPSERVERSTREAM_H

#include <thread>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "SocketBuffer.h"

namespace modules {
namespace robotx {

    class TcpServerStream: public std::iostream
    {
    public:
        enum ConnectionStatus  { DISCONNECTED=0, CONNECTED=1, CONNECTING=2 };

    private:
        bool stop_thread;
        ConnectionStatus connection_status;
        boost::asio::io_service io_service;
        boost::system::error_code error;
        boost::asio::ip::tcp::endpoint endpoint;
        boost::asio::ip::tcp::acceptor acceptor;
        boost::asio::ip::tcp::socket socket;
        SocketBuffer read_buffer;
        std::shared_ptr<std::thread> worker_thread;

    public:
        TcpServerStream();
        void init(uint port);
        ~TcpServerStream();

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
#endif // TCPSERVERSTREAM_H
