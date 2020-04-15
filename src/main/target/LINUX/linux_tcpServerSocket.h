

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <functional>
#include <mutex>
#include <deque>
#include <vector>
#include <boost/asio.hpp>

#define MAX_TX_QUEUE_SIZE 1024
typedef std::function<void (uint8_t *, std::size_t)> ReadCallback_t;
typedef std::function<void (boost::system::error_code )> ErrorCallback_t;

class tcpClient
{

protected:
  enum { max_length = MAX_TX_QUEUE_SIZE };

  boost::asio::ip::tcp::socket  m_socket;
  ReadCallback_t                m_readCallback;
  ErrorCallback_t               m_errorCallback;
  uint8_t                       m_data[max_length];
  uint8_t                       m_txData[max_length];
  std::mutex                    m_mutex;
  std::deque<uint8_t>           m_txQueue;
  std::deque<uint8_t>           m_rxQueue;
  std::atomic<bool>             m_isWriting;
  
protected:
  void start_read(void);
  void writeKeepSending(size_t bytesToSend, size_t bytesTransfer);

public: 
tcpClient( boost::asio::ip::tcp::socket , 
           ReadCallback_t &,
           ErrorCallback_t &);
           
virtual ~tcpClient();
void start();
void write(boost::asio::io_context &io,
            uint8_t *pWriteData, 
            size_t len);

void restartWrite();
void postWrite(void);
size_t bytesInRxQueue();
size_t bytesInTxQueue();
uint8_t readRxByte(void);

};


inline
tcpClient::tcpClient(boost::asio::ip::tcp::socket socket, 
                    ReadCallback_t &readCallback,
                    ErrorCallback_t &errorCallback): m_socket(std::move(socket)),
                                                     m_readCallback(readCallback),
                                                     m_errorCallback(errorCallback),
                                                     m_isWriting(false)
{

}


inline
tcpClient::~tcpClient()
{

}

inline 
void tcpClient::start_read()
{
  
  m_socket.async_read_some(boost::asio::buffer(m_data, max_length),
     [this](boost::system::error_code ec, std::size_t length)
     {
          if (!ec)
          {
            {  
              std::lock_guard<std::mutex> guard(m_mutex);
              for(size_t index=0; index < length; ++index)
                m_rxQueue.push_back(m_data[index]);
            }
            start_read();
          }
          else
          {
            m_errorCallback(ec);
          }
     });
    
}
inline
void tcpClient::start()
{
    m_socket.non_blocking(true);
    start_read();
}

inline 
void tcpClient::postWrite()
{
    if (m_isWriting)
        return;

    restartWrite();
}
inline 
void tcpClient::writeKeepSending(size_t bytesToSend, size_t bytesTransfer)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    auto buffer = boost::asio::buffer(&m_txData[bytesTransfer], bytesToSend- bytesTransfer);
    m_socket.async_write_some(buffer, [this,bytesToSend, bytesTransfer](const boost::system::error_code &ec, std::size_t bytes_transferred)
    {
      std::lock_guard<std::mutex> guard(m_mutex);
      size_t total_bytesSent = bytesTransfer + bytes_transferred;
      if (ec || bytes_transferred == 0)
      {
        m_errorCallback(ec); 
        m_isWriting = false;
        std::cout << __PRETTY_FUNCTION__ << "Error:" << std::endl;
        boost::asio::post( m_socket.get_executor(), [this]() { restartWrite(); });
      }
      else if ( bytesToSend != bytes_transferred)
      {
          boost::asio::post( m_socket.get_executor(), [this, total_bytesSent, bytes_transferred]() { writeKeepSending(total_bytesSent, bytes_transferred); });
      }
      else
      {
        m_isWriting = false;
        boost::asio::post( m_socket.get_executor(), [this]() { restartWrite(); });
      }
      });
}

inline
void tcpClient::restartWrite()
{
   std::lock_guard<std::mutex> guard(m_mutex);
    if (m_txQueue.size() == 0)
    {
        m_isWriting = false;
        return;
    }

    m_isWriting = true;
    size_t bytesToSend = 0;
    while( m_txQueue.size() && (bytesToSend < sizeof(m_txData)))
    {
        m_txData[bytesToSend]= m_txQueue[0];
        bytesToSend++;
        m_txQueue.pop_front();
    }
    auto buffer = boost::asio::buffer(m_txData , bytesToSend);
    m_socket.async_write_some(buffer, [this, bytesToSend](const boost::system::error_code &ec,
                                                           std::size_t bytes_transferred)
    {
      std::lock_guard<std::mutex> guard(m_mutex);
      if (ec || bytes_transferred == 0)
      {
          m_errorCallback(ec); 
          std::cout << "Error" << std::endl;
          m_isWriting = false;
          boost::asio::post( m_socket.get_executor(),[this](){ restartWrite(); });
      }
      else if ( bytesToSend != bytes_transferred)
      {
          boost::asio::post( m_socket.get_executor(), [this, bytesToSend, bytes_transferred]() { writeKeepSending(bytesToSend, bytes_transferred); });
      }
      else
      {
        m_isWriting = false;
        boost::asio::post( m_socket.get_executor(), [this]() { restartWrite(); });
      }
     });
}


inline 
void tcpClient::write(boost::asio::io_context &io, 
                        uint8_t *pWriteData, 
                        size_t len)
{
    // set a guard around m_txQueue and update
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if ( (m_txQueue.size() + len) > MAX_TX_QUEUE_SIZE)
        {
            std::cout << "full " << std::endl;
            return ;
        }
        for(size_t index = 0; index < len; ++index)
          m_txQueue.push_back(pWriteData[index]);
    }

    //push the operation back onto single processing thread
    boost::asio::post(io, [this]() { postWrite(); });
}

inline 
size_t tcpClient::bytesInRxQueue()
{
  std::lock_guard<std::mutex> guard(m_mutex);
  return m_rxQueue.size();

}

inline 
size_t tcpClient::bytesInTxQueue()
{
  std::lock_guard<std::mutex> guard(m_mutex);
  return m_txQueue.size();
}

inline 
uint8_t tcpClient::readRxByte()
{
  uint8_t byteRead = 0;
  std::lock_guard<std::mutex> guard(m_mutex);
  if (m_rxQueue.size())
  {
    byteRead = m_rxQueue[0];
    m_rxQueue.pop_front();
  }

  return byteRead;
}





class tcpServer: public Port
{
protected:
  boost::asio::ip::tcp::acceptor m_acceptor;
  std::shared_ptr<tcpClient> m_client;
  boost::asio::io_context &m_io_context;

protected:
    void do_accept(void);
    void read(uint8_t *, std::size_t);
    void error(const boost::system::error_code &);
    void write(uint8_t *pWriteData, size_t len);
    

public:
  tcpServer(serialPort_t &entry, 
            boost::asio::io_context &, 
            uint16_t );
  virtual ~tcpServer();
serialPortVTable *getCallbackTable(void);  
    void write( serialPort_t *instance, uint8_t ch);
    size_t bytesInRxQueue(void);
    size_t bytesInTxQueue(void);
    uint8_t readRxByte(void);
};


/**
 * @brief Construct a new tcp Server::tcp Server object
 * 
 * @param entry 
 * @param io_context 
 * @param port 
 */
inline 
tcpServer::tcpServer(serialPort_t &entry, boost::asio::io_context& io_context, 
                    uint16_t port): Port(entry), 
                                    m_acceptor(io_context, {boost::asio::ip::tcp::v4(), port}),
                                    m_io_context(io_context)
{
    do_accept();
    m_entry.rxCallbackData = static_cast<void *>(this);
  
    m_callbackVTable.serialWrite = [](serialPort_t *instance, uint8_t ch)  -> void
    {
       tcpServer *pThis = static_cast<tcpServer *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return;

       pThis->write(&ch, 1);
    };
    m_callbackVTable.serialTotalRxWaiting = [](const serialPort_t *instance) -> uint32_t
    {
       tcpServer *pThis = static_cast<tcpServer *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return 0;

       return pThis->bytesInRxQueue();
    };

    m_callbackVTable.serialTotalTxFree =[](const serialPort_t *instance) -> uint32_t
    {
        tcpServer *pThis = static_cast<tcpServer *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return MAX_TX_QUEUE_SIZE;

        if (  pThis->bytesInTxQueue() > MAX_TX_QUEUE_SIZE)
        {
          return 0;
        }
        else
        {
          return  MAX_TX_QUEUE_SIZE - pThis->bytesInTxQueue();
        }
    };

    m_callbackVTable.serialRead = [](serialPort_t *instance) -> uint8_t 
    {
        tcpServer *pThis = static_cast<tcpServer *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return 0;

       return pThis->readRxByte();
    };
    
    m_callbackVTable.isSerialTransmitBufferEmpty = [](const serialPort_t *instance) -> bool
    {
        tcpServer *pThis = static_cast<tcpServer *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return false;

       if (0 == pThis->bytesInTxQueue())
       {
          return true;
       }
       else
       {
         return false;
       }
    };
}

/**
 * @brief Destroy the tcp Server::tcp Server object
 * 
 */
inline 
tcpServer::~tcpServer()
{

}

/**
 * @brief 
 * 
 */
inline
void tcpServer::do_accept()
{
    m_acceptor.async_accept(
        [this](boost::system::error_code ec, boost::asio::ip::tcp::socket socket)
        {
          if (!ec)
          {
            ReadCallback_t readcallback = std::bind(&tcpServer::read, 
                                                      this, 
                                                      std::placeholders::_1,
                                                      std::placeholders::_2);
            ErrorCallback_t errorcallback = std::bind(&tcpServer::error, 
                                                      this, 
                                                      std::placeholders::_1);
            m_client = std::make_shared<tcpClient>(std::move(socket), readcallback, errorcallback);
            m_client->start();
          }

          do_accept();
        });
  }


/**
 * @brief 
 * 
 * @param pData 
 * @param bytesRead 
 */
inline
void tcpServer::read(uint8_t *pData, std::size_t bytesRead)
{
UNUSED(pData);
UNUSED(bytesRead);

}

/**
 * @brief 
 * 
 * @param ec 
 */
inline
void tcpServer::error(const boost::system::error_code &ec)
{
  UNUSED(ec);
  m_client.reset();
}


/**
 * @brief 
 * 
 * @param pWriteData 
 * @param len 
 */
inline
void tcpServer::write(uint8_t *pWriteData, size_t len)
{
    if (m_client)
    {
        m_client->write(m_io_context,pWriteData, len);
    }
}

/**
 * @brief 
 * 
 * @return size_t 
 */
inline
size_t tcpServer::bytesInRxQueue()
{
    if (m_client)
    {
        return m_client->bytesInRxQueue();
    }

    return 0;
}

/**
 * @brief 
 * 
 * @return size_t 
 */
inline 
size_t tcpServer::bytesInTxQueue()
{
    if (m_client)
    {
        return m_client->bytesInTxQueue();
    }

    return 0;
}

/**
 * @brief 
 * 
 * @return uint8_t 
 */
inline
uint8_t tcpServer::readRxByte()
{
    if (m_client)
    {
        return m_client->readRxByte();
    }

    return 0;
}

// eof
