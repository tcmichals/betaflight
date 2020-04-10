#include <cstdint>
#include <functional>
#include "addressmap.h"
#include "avalonProtocol.h"
#include "transport.h"

#define sysID 0x11223344

class gpioAvalon
{

protected:
	transportAvalon &m_transport;


protected:
	bool rxComplete(int errno, const transportAvalon::array_t &rxBuffer);
	bool txComplete(int errno, size_t bytesSent);
	bool setLEDRegister(uint8_t val, uint32_t led_reg);


public:
	gpioAvalon(transportAvalon &transport);
	virtual ~gpioAvalon();
	bool postLED(uint8_t led);
	bool toggleLED(uint8_t led);
	bool setLED(uint8_t led);
	bool clearLED(uint8_t led);


};

inline 
gpioAvalon::gpioAvalon(transportAvalon &transport): m_transport(transport)
{

}

inline
gpioAvalon::~gpioAvalon()
{
}



inline 
bool gpioAvalon::txComplete(int errno, size_t bytesSent)
{
	return true;

}

inline 	
bool gpioAvalon::rxComplete(int errno, const transportAvalon::array_t &rxBuffer) 
{				
	return true;
}

inline 
bool gpioAvalon::setLEDRegister(uint8_t val, uint32_t led_reg)
{
	avalonProtocol protocol;
	avalonProtocol::array_t tx_packet;
    avalonProtocol::array_t data;

	auto rxCallback = std::bind(&gpioAvalon::rxComplete, this,
							    std::placeholders::_1, 
                                std::placeholders::_2);

	auto txCallback = std::bind(&gpioAvalon::txComplete, this,
							    std::placeholders::_1, 
                                std::placeholders::_2);

    data.push_back(val);
	protocol.transaction_channel_write(tx_packet,
								led_reg, 
								AVALON_CHANNEL, 
                                data,
								Avalon_Trans_Type::WRITE_NON_INCREMENTING);

		m_transport.postTransaction(tx_packet, txCallback, rxCallback);		

	return true;
}

inline
bool gpioAvalon::postLED(uint8_t led)
{
	return setLEDRegister(led, GPIO_LED_START_ADDRESS);
}

inline
bool gpioAvalon::toggleLED(uint8_t led)
{
	return setLEDRegister(led, GPIO_LED_XOR_ADDRESS);
}

inline
bool gpioAvalon::setLED(uint8_t led)
{
	return setLEDRegister(led, GPIO_LED_OR_ADDRESS);
}
inline
bool gpioAvalon::clearLED(uint8_t led)
{
	return setLEDRegister(led, GPIO_LED_CLEAR_ADDRESS);
}