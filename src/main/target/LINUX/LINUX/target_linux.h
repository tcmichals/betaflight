#include <iostream>
#include <memory>
#include <thread>
#include <boost/asio.hpp>  
#include <boost/asio/error.hpp>  


extern boost::asio::io_context  &getIOService(void);
extern bool startWorkerThread(void);