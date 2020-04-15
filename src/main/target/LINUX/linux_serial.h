
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <functional>
#include <mutex>
#include <deque>
#include <vector>
#include <boost/asio.hpp>



class LinuxSerialPort: public Port
{
public:
      enum { max_length = 1024 };

protected:

    std::unique_ptr<boost::asio::serial_port>   m_serialPort;
    boost::asio::io_context                     &m_io_context;
    std::string                                 m_name;
    std::mutex                                  m_mutex;
    std::deque<std::vector<uint8_t>>            m_txQueue;
    std::deque<uint8_t>                         m_rxQueue;
    std::atomic<bool>                           m_isWriting;
    uint8_t                                     m_data[max_length];

protected:
    void read(uint8_t *, std::size_t);
    void error(const boost::system::error_code &);
    void write(uint8_t *pWriteData, size_t len);
    void start_read(void);
    

public:
  LinuxSerialPort(serialPort_t &entry, 
            boost::asio::io_context &);
  virtual ~LinuxSerialPort();
serialPortVTable *getCallbackTable(void);  
    void write( serialPort_t *instance, uint8_t ch);
    size_t bytesInRxQueue(void);
    size_t bytesInTxQueue(void);
    uint8_t readRxByte(void);
    bool open(const std::string &device_name);
    bool set_baudrate( int baudrate);
    void do_write();
};


/**
 * @brief Construct a new tcp Server::tcp Server object
 * 
 * @param entry 
 * @param io_context 
 * @param port 
 */
inline 
LinuxSerialPort::LinuxSerialPort(   serialPort_t &entry, 
                                    boost::asio::io_context& io_context): Port(entry), 
                                    m_io_context(io_context),
                                    m_isWriting(false)

{

    m_entry.rxCallbackData = static_cast<void *>(this);
  
    m_callbackVTable.serialWrite = [](serialPort_t *instance, uint8_t ch)  -> void
    {
       LinuxSerialPort *pThis = static_cast<LinuxSerialPort *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return;

       pThis->write(&ch, 1);
    };
    m_callbackVTable.serialTotalRxWaiting = [](const serialPort_t *instance) -> uint32_t
    {
       LinuxSerialPort *pThis = static_cast<LinuxSerialPort *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return 0;

       return pThis->bytesInRxQueue();
    };

    m_callbackVTable.serialTotalTxFree =[](const serialPort_t *instance) -> uint32_t
    {
        LinuxSerialPort *pThis = static_cast<LinuxSerialPort *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return max_length;

        if (  pThis->bytesInTxQueue() > max_length)
        {
          return 0;
        }
        else
        {
          return  max_length - pThis->bytesInTxQueue();
        }
    };

    m_callbackVTable.serialRead = [](serialPort_t *instance) -> uint8_t 
    {
        LinuxSerialPort *pThis = static_cast<LinuxSerialPort *>(instance->rxCallbackData);
       if (pThis == nullptr)
          return 0;

       return pThis->readRxByte();
    };
    
    m_callbackVTable.isSerialTransmitBufferEmpty = [](const serialPort_t *instance) -> bool
    {
        LinuxSerialPort *pThis = static_cast<LinuxSerialPort *>(instance->rxCallbackData);
        if (pThis == nullptr)
          return true;

        if (0 == pThis->bytesInTxQueue())
        {
            return true;
        }
       else
       {
         return false;
       }
    };

    m_callbackVTable.serialSetBaudRate= [](serialPort_t *instance, uint32_t baudRate) -> void
    {
        LinuxSerialPort *pThis = static_cast<LinuxSerialPort *>(instance->rxCallbackData);
        if (pThis == nullptr)
          return;

        pThis->set_baudrate(baudRate);
    };
}

/**
 * @brief Destroy the tcp Server::tcp Server object
 * 
 */
inline 
LinuxSerialPort::~LinuxSerialPort()
{

}

inline
bool LinuxSerialPort::open(const std::string &device_name)
{
	boost::system::error_code ec;
  bool rc = false;

  m_name = device_name;
	try
	{
		m_serialPort = std::make_unique<boost::asio::serial_port>(m_io_context);
		m_serialPort->open(device_name);
		rc = true;
	}
	catch(boost::system::error_code &ex)
	{
		rc = false;
	}
	catch(std::exception  &ex)
	{
		std::cerr << __PRETTY_FUNCTION__ << " " << ex.what() << std::endl;
		rc = false;
	}
	catch(...)
	{
		rc = false;
	}

	return rc;
}
inline 
void LinuxSerialPort::start_read()
{

     boost::asio::async_read(*m_serialPort.get(), boost::asio::buffer(m_data, max_length),
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
          }
     });
    
}

inline
void LinuxSerialPort::do_write()
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        if (m_isWriting)
        {
            return;
        }
        else
        {
            m_isWriting = true;
        }
    }

    boost::asio::async_write(*m_serialPort.get(),
		        boost::asio::buffer(m_txQueue.front().data(),
		          m_txQueue.front().size()),
        [this](boost::system::error_code ec, std::size_t /*length*/)
        {
          if (!ec)
          {
            bool isEmpty;
            {
                std::lock_guard<std::mutex> guard(m_mutex);
                m_txQueue.pop_front();
                isEmpty = m_txQueue.empty();
            }

            if (isEmpty)
            {
              do_write();
            }
	        else
	        {
                std::lock_guard<std::mutex> guard(m_mutex);
		        m_isWriting = false;
	        }
          }
          else
          {
            m_serialPort->close();
	        m_isWriting = false;
          }
        });
}



inline
bool LinuxSerialPort::set_baudrate( int baudrate)
{

	bool rc = false;
	try
	{
		m_serialPort->set_option(boost::asio::serial_port_base::character_size(8));
		m_serialPort->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		m_serialPort->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		m_serialPort->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

		m_serialPort->set_option(boost::asio::serial_port_base::baud_rate(baudrate));	
		rc = true;
	}
	catch(boost::system::error_code &ex)
	{
		std::cerr << __PRETTY_FUNCTION__ << " " << ex << std::endl;
		rc = false;
	}
	catch(std::exception  &ex)
	{
		std::cerr << __PRETTY_FUNCTION__ << " " << ex.what() << std::endl;
		rc = false;
	}
	catch(...)
	{
		std::cerr << __PRETTY_FUNCTION__ << "unkwnon " <<std::endl;
		 rc = false;
	}

	return rc;
}


/**
 * @brief 
 * 
 * @param pData 
 * @param bytesRead 
 */
inline
void LinuxSerialPort::read(uint8_t *pData, std::size_t bytesRead)
{
    UNUSED(pData);
    UNUSED(bytesRead);

    std::cout<< __PRETTY_FUNCTION__ << " bytes read(" << bytesRead << ")" << std::endl;
  

}

/**
 * @brief 
 * 
 * @param ec 
 */
inline
void LinuxSerialPort::error(const boost::system::error_code &ec)
{
     UNUSED(ec);
    std::cout<< __PRETTY_FUNCTION__ << " error" << ec.message() << ")" << std::endl;
}


/**
 * @brief 
 * 
 * @param pWriteData 
 * @param len 
 */
inline
void LinuxSerialPort::write(uint8_t *pWriteData, size_t len)
{
    if ( bytesInTxQueue() > max_length)
    {
        do_write();
        return ;
    }
    else
    {
        std::vector<uint8_t> txData(pWriteData, pWriteData + len);
        std::lock_guard<std::mutex> guard(m_mutex);
        m_txQueue.push_back(txData);
    }

    do_write();
}

/**
 * @brief 
 * 
 * @return size_t 
 */
inline
size_t LinuxSerialPort::bytesInRxQueue()
{
  std::lock_guard<std::mutex> guard(m_mutex);
  return m_rxQueue.size();
}

/**
 * @brief 
 * 
 * @return size_t 
 */
inline 
size_t LinuxSerialPort::bytesInTxQueue()
{
  size_t bytesInQueue = 0;
  std::lock_guard<std::mutex> guard(m_mutex);
  for (auto &tx : m_txQueue)
  {
      bytesInQueue += tx.size();
  }

  return bytesInQueue;
}

/**
 * @brief 
 * 
 * @return uint8_t 
 */
inline
uint8_t LinuxSerialPort::readRxByte()
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