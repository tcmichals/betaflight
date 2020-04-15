
#include <iostream>
#include <memory>
#include <thread>
#include <boost/asio.hpp>  
#include <boost/asio/error.hpp>  
#include <thread>
#include <functional>

static boost::asio::io_context gIOService(1);
static std::unique_ptr<std::thread> gBackgroundThread;

extern bool startFPGA();
boost::asio::io_context  &getIOService()
{
    return gIOService;
}

static void backgroundThread()
{

   boost::asio::signal_set signals(gIOService, SIGINT, SIGTERM);
   signals.async_wait([&](auto, auto){ gIOService.stop(); std::cout<< "stopping" << std::endl; });

    gIOService.run();
}


extern "C" bool startWorkerThread()
{
    gBackgroundThread = std::make_unique<std::thread>(backgroundThread);
    startFPGA();
    return true;
}




//eof



