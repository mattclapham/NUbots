#include "TcpClientStream.h"

namespace modules {
namespace robotx {

    using namespace std;

    TcpClientStream::TcpClientStream()
        : stop_thread(false)
        , connection_status(DISCONNECTED)
        , resolver(io_service)
        , socket(io_service)
        , read_buffer(io_service, socket)
    {
        iostream::rdbuf(&read_buffer);
    }

    void TcpClientStream::init(std::string host, std::string  port)
    {
        worker_thread.reset(new std::thread(std::bind(&TcpClientStream::doWork, this )));
        endpoint_iterator = resolver.resolve({ host, port });
    }

    TcpClientStream::~TcpClientStream()
    {
        TcpClientStream::close();
        stop_thread = true;
        worker_thread->join();
    }

    void TcpClientStream::doWork()
    {
        while(1)
        {
            if (stop_thread) break;

            if (connection_status == CONNECTING) {
                flush();
                read_buffer.pubsync();
                clear();

                boost::system::error_code ec;
                boost::asio::connect(socket, endpoint_iterator, ec);

                if (!ec) {
                    connection_status = CONNECTED;
                }
                else {
                    connection_status = DISCONNECTED;
                }
            }

            if (connection_status == CONNECTED && !socket.is_open()) {
                connection_status = DISCONNECTED;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void TcpClientStream::close()
    {
        connection_status = DISCONNECTED;
        flush();
        read_buffer.pubsync();
        socket.close();
    }

    void TcpClientStream::connect()
    {
        if (connection_status == DISCONNECTED) {
            connection_status = CONNECTING;
        }
    }

    void TcpClientStream::disconnect()
    {
        TcpClientStream::close();
    }

    bool TcpClientStream::isConnected()
    {
        return connection_status == CONNECTED;
    }

    TcpClientStream::ConnectionStatus TcpClientStream::connectionStatus()
    {
        return connection_status;
    }

}
}
