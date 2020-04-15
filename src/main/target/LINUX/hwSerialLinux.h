/*
 * hwSerialLinux.h
 *
 *  Created on: Dec 12, 2017
 *      Author: tcmichals
 */

// Need libevent library
#pragma once

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <memory>
#include <deque>

#include <fcntl.h> /* File Control Definitions           */
#include <string>
#include <termios.h> /* POSIX Terminal Control Definitions */
#include <unistd.h>  /* UNIX Standard Definitions 	   */


#ifndef _WIN32
#include <netinet/in.h>
#ifdef _XOPEN_SOURCE_EXTENDED
#include <arpa/inet.h>
#endif
#include <sys/socket.h>
#endif

#include <event2/bufferevent.h>
#include <event2/buffer.h>
#include <event2/listener.h>
#include <event2/util.h>
#include <event2/event.h>
#include <event2/event_struct.h>

#include "msgLogger.h"

class hwSerialLinux
{

public:
  typedef std::function<void (uint8_t *pBuff, int numOfBytesRead)> readCallBack_t;

typedef enum {
	SERIAL_RX_BUFFER = 1024
}serialBufferSizes_t;

  public:
  hwSerialLinux();
    virtual ~hwSerialLinux();
    bool init(std::shared_ptr<struct event_base> base);
    bool open(const std::string& serialPort, int speed);
    bool close();

    bool addReadCB(const readCallBack_t &handler, int readTimeInMS);
    bool removeReadCB(void);

    uint32_t serialTotalRxWaiting();
    uint32_t serialTotalTxFree();
    uint8_t serialRead();
    void serialSetBaudRate(int baudRate);
    uint32_t serialWrite(uint8_t ch);

protected:
    void socketEvt(evutil_socket_t fd, short event);

protected:
    int m_fd;
    std::mutex m_mutex;
    readCallBack_t m_readHandler;
    int m_readTimeOutInMS;
    struct event m_serialEvt;
    std::shared_ptr<struct event_base> m_base;
    std::deque<uint8_t> m_rxBuffer;


};

inline
hwSerialLinux::hwSerialLinux():m_fd(-1),
                              m_readTimeOutInMS(-1)
{

}

inline
hwSerialLinux::~hwSerialLinux()
{
    if(m_fd != -1)
        close();

    m_readTimeOutInMS= -1;
    m_fd = -1;
}

inline
void hwSerialLinux::socketEvt(evutil_socket_t fd, short event)
{

  logMsg(logMsgModule_libevent, logMsg_Debug,"%s+",  __PRETTY_FUNCTION__);

	switch(event)
	{

	case EV_TIMEOUT:
	{

	}
	break;
	case EV_READ:
	{
		std::array<uint8_t, 128> _serialRx;
		std::array<uint8_t, 128>::iterator _it;

		ssize_t bytes_read = read(m_fd,_serialRx.data(),_serialRx.size()); /* Read the data                   */

		std::lock_guard<std::mutex> _lock(m_mutex);
		while(bytes_read > 0 && _it != _serialRx.end())
		{
			m_rxBuffer.push_back(*_it);
			_it++;
			bytes_read--;
		}
        logMsg(logMsgModule_libevent, logMsg_Debug,"%s EV_READ",  __PRETTY_FUNCTION__);

	}
	break;
	case EV_WRITE:
		std::cout << __PRETTY_FUNCTION__ << "EV_WRITE" << std::endl ;
		break;
	default:
		std::cout << __PRETTY_FUNCTION__ << "UnKnown" << std::endl ;
		break;

	}
	
	 logMsg(logMsgModule_libevent, logMsg_Debug,"%s-",  __PRETTY_FUNCTION__);
	return ;
}


inline
bool hwSerialLinux::init(std::shared_ptr<struct event_base> base)
{
	m_base = base;
    return true;
}

inline
bool hwSerialLinux::open(const std::string& serialPort,
						int speed)
{
    speed_t baud;
    switch(speed)
    {
    // Do POSIX-specified rates first.
    case 0:
        baud = B0;
        break;
    case 50:
        baud = B50;
        break;
    case 75:
        baud = B75;
        break;
    case 110:
        baud = B110;
        break;
    case 134:
        baud = B134;
        break;
    case 150:
        baud = B150;
        break;
    case 200:
        baud = B200;
        break;
    case 300:
        baud = B300;
        break;
    case 600:
        baud = B600;
        break;
    case 1200:
        baud = B1200;
        break;
    case 1800:
        baud = B1800;
        break;
    case 2400:
        baud = B2400;
        break;
    case 4800:
        baud = B4800;
        break;
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
// And now the extended ones conditionally.
#ifdef B7200
    case 7200:
        baud = B7200;
        break;
#endif
#ifdef B14400
    case 14400:
        baud = B14400;
        break;
#endif
#ifdef B57600
    case 57600:
        baud = B57600;
        break;
#endif
#ifdef B115200
    case 115200:
        baud = B115200;
        break;
#endif
#ifdef B230400
    case 230400:
        baud = B230400;
        break;
#endif
#ifdef B460800
    case 460800:
        baud = B460800;
        break;
#endif
#ifdef B500000
    case 500000:
        baud = B500000;
        break;
#endif
#ifdef B576000
    case 576000:
        baud = B576000;
        break;
#endif
#ifdef B921600
    case 921600:
        baud = B921600;
        break;
#endif
#ifdef B1000000
    case 1000000:
        baud = B1000000;
        break;
#endif
#ifdef B1152000
    case 1152000:
        baud = B1152000;
        break;
#endif
#ifdef B2000000
    case 2000000:
        baud = B2000000;
        break;
#endif
#ifdef B3000000
    case 3000000:
        baud = B3000000;
        break;
#endif
#ifdef B3500000
    case 3500000:
        baud = B3500000;
        break;
#endif
#ifdef B4000000
    case 4000000:
        baud = B4000000;
        break;
#endif
    default:

        return false;
    }

    m_fd = ::open(serialPort.c_str(),O_RDWR | O_NOCTTY | O_SYNC);

    if(m_fd < 0)
    {
        return false;
    }

    struct termios SerialPortSettings;	/* Create the structure                          */
    tcgetattr(m_fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    cfsetispeed(&SerialPortSettings,baud); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,baud); /* Set Write Speed as 9600                       */

    SerialPortSettings.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
    SerialPortSettings.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
    SerialPortSettings.c_cflag &= ~CSIZE;   /* Clears the mask for setting the data size             */
    SerialPortSettings.c_cflag |=  CS8;     /* Set the data bits = 8                                 */
    SerialPortSettings.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */

    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
    SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */

    SerialPortSettings.c_oflag &= ~OPOST;/*No Output Processing*/

    if((tcsetattr(m_fd,TCSANOW,&SerialPortSettings)) != 0)
    {
        //close serial port..
        return close();
    }

	struct timeval tv;
	memset(&tv, 0, sizeof(tv));
	tv.tv_sec =1;

	/* Add the UDP event */
	event_assign(&m_serialEvt,
			m_base.get(),
			m_fd,
			EV_READ|EV_PERSIST | EV_TIMEOUT,
			[](evutil_socket_t fd, short event, void *arg)->void
			{
				hwSerialLinux *_pThis = static_cast<hwSerialLinux*>(arg);
				if(_pThis) _pThis->socketEvt(fd,event);

			},
			static_cast<void*>(this));

	event_add(&m_serialEvt, &tv);

    return true;
}

bool hwSerialLinux::close()
{
    bool rc = false;
    if (m_fd >-1)
    {
        ::close(m_fd);
        rc = true; // it was open.. close
    }

    m_fd = -1;
    return rc;
}

inline
bool hwSerialLinux::addReadCB(const hwSerialLinux::readCallBack_t &handler, int readTimeOutInMS)
{
     std::lock_guard<std::mutex> _lock(m_mutex);

     m_readHandler = handler;
     m_readTimeOutInMS = readTimeOutInMS;
     return true;

}

inline
bool hwSerialLinux::removeReadCB(void)
{
      std::lock_guard<std::mutex> _lock(m_mutex);

     m_readHandler=readCallBack_t();
     m_readTimeOutInMS = -1;
     return true;
}

inline
uint32_t hwSerialLinux::serialTotalRxWaiting()
{
    std::lock_guard<std::mutex> _lock(m_mutex);
    return m_rxBuffer.size();

}
inline
uint32_t hwSerialLinux::serialTotalTxFree()
{
	return 1024;
}
inline
uint8_t hwSerialLinux::serialRead()
{
    std::lock_guard<std::mutex> _lock(m_mutex);
    if (m_rxBuffer.size())
    {
    	uint8_t ch = m_rxBuffer[0];
    	m_rxBuffer.pop_front();
    	return ch;
    }

    return 0;
}

inline
void hwSerialLinux::serialSetBaudRate(int speed)
{
    speed_t baud;
    switch(speed)
    {
    // Do POSIX-specified rates first.
    case 0:
        baud = B0;
        break;
    case 50:
        baud = B50;
        break;
    case 75:
        baud = B75;
        break;
    case 110:
        baud = B110;
        break;
    case 134:
        baud = B134;
        break;
    case 150:
        baud = B150;
        break;
    case 200:
        baud = B200;
        break;
    case 300:
        baud = B300;
        break;
    case 600:
        baud = B600;
        break;
    case 1200:
        baud = B1200;
        break;
    case 1800:
        baud = B1800;
        break;
    case 2400:
        baud = B2400;
        break;
    case 4800:
        baud = B4800;
        break;
    case 9600:
        baud = B9600;
        break;
    case 19200:
        baud = B19200;
        break;
    case 38400:
        baud = B38400;
        break;
// And now the extended ones conditionally.
#ifdef B7200
    case 7200:
        baud = B7200;
        break;
#endif
#ifdef B14400
    case 14400:
        baud = B14400;
        break;
#endif
#ifdef B57600
    case 57600:
        baud = B57600;
        break;
#endif
#ifdef B115200
    case 115200:
        baud = B115200;
        break;
#endif
#ifdef B230400
    case 230400:
        baud = B230400;
        break;
#endif
#ifdef B460800
    case 460800:
        baud = B460800;
        break;
#endif
#ifdef B500000
    case 500000:
        baud = B500000;
        break;
#endif
#ifdef B576000
    case 576000:
        baud = B576000;
        break;
#endif
#ifdef B921600
    case 921600:
        baud = B921600;
        break;
#endif
#ifdef B1000000
    case 1000000:
        baud = B1000000;
        break;
#endif
#ifdef B1152000
    case 1152000:
        baud = B1152000;
        break;
#endif
#ifdef B2000000
    case 2000000:
        baud = B2000000;
        break;
#endif
#ifdef B3000000
    case 3000000:
        baud = B3000000;
        break;
#endif
#ifdef B3500000
    case 3500000:
        baud = B3500000;
        break;
#endif
#ifdef B4000000
    case 4000000:
        baud = B4000000;
        break;
#endif
    default:

    	return;
    }

    struct termios SerialPortSettings;	/* Create the structure                          */
    tcgetattr(m_fd, &SerialPortSettings);	/* Get the current attributes of the Serial port */

    cfsetispeed(&SerialPortSettings,baud); /* Set Read  Speed as 9600                       */
    cfsetospeed(&SerialPortSettings,baud); /* Set Write Speed as 9600                       */

    if((tcsetattr(m_fd,TCSANOW,&SerialPortSettings)) != 0)
    {
        //close serial port..
        close();
    }
}



uint32_t hwSerialLinux::serialWrite(uint8_t ch)
{
    logMsg(logMsgModule_libevent, logMsg_Debug,"%s+",  __PRETTY_FUNCTION__);

	if (m_fd !=-1)
	{
        int bytesWritten = write(m_fd,&ch,sizeof(ch));
        logMsg(logMsgModule_libevent, logMsg_Debug,"%s bytesWritten=%d",  __PRETTY_FUNCTION__,bytesWritten);
		return write(m_fd,&ch,sizeof(ch));
	}

    logMsg(logMsgModule_libevent, logMsg_Debug,"%s",  __PRETTY_FUNCTION__);
	return 0;
}

// eof




