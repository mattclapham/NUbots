#include "TcpServerStream.h"

namespace modules {
namespace robotx {

    using boost::asio::ip::tcp;
    using namespace std;

    TcpServerStream::TcpServerStream()
        : connection_status(DISCONNECTED)
        , endpoint(tcp::v4(), 4000)
        , acceptor(io_service, endpoint)
        , socket(io_service)
        , read_buffer(io_service, socket)
    {
        iostream::rdbuf(&read_buffer);
    }

    void TcpServerStream::init(uint port)
    {
        endpoint = boost::asio::ip::tcp::endpoint(tcp::v4(), port);
        acceptor = boost::asio::ip::tcp::acceptor(io_service, endpoint);
        worker_thread.reset(new std::thread(std::bind(&TcpServerStream::doWork, this )));
    }

    TcpServerStream::~TcpServerStream()
    {
        TcpServerStream::close();
        stop_thread = true;
        worker_thread->join();
    }

    void TcpServerStream::doWork()
    {
        while(1)
        {
            if (stop_thread) break;

            if (connection_status == CONNECTING) {
                flush();
                read_buffer.pubsync();
                clear();

                acceptor.accept(socket, &error);

                if (!error) {
                    connection_status = CONNECTED;
                }
                else {
                    connection_status = DISCONNECTED;
                }
            }

            if (connection_status == CONNECTED && !socket.is_open()) {
                connection_status = DISCONNECTED;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void TcpServerStream::close()
    {
        connection_status = DISCONNECTED;
        flush();
        read_buffer.pubsync();
        socket.close();
    }

    void TcpServerStream::connect()
    {
        if (connection_status == DISCONNECTED) {
            connection_status = CONNECTING;
        }
    }

    void TcpServerStream::disconnect()
    {
        TcpServerStream::close();
    }

    bool TcpServerStream::isConnected()
    {
        return connection_status == CONNECTED;
    }

    TcpServerStream::ConnectionStatus TcpServerStream::connectionStatus()
    {
        return connection_status;
    }

}
}
