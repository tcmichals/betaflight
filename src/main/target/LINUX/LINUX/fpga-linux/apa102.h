#include <cstdint>
#include <functional>
#include "addressmap.h"
#include "avalonProtocol.h"
#include "transport.h"

class apa102LED
{

public:
    typedef enum
    {
        number_of_leds = 8,
    } numLEDS_t;

    enum class whichLED : int32_t
    {
        LED_1 = 0,
        LED_2,
        LED_3,
        LED_4,
        LED_5,
        LED_6,
        LED_7,
        LED_8
    };

protected:
    std::array<uint32_t, number_of_leds> m_leds;
    transportAvalon &m_transport;

protected:
    bool rxComplete(int errno, const transportAvalon::array_t &rxBuffer);
    bool txComplete(int errno, size_t bytesSent);

public:
    apa102LED(transportAvalon &transport);
    virtual ~apa102LED();
    void set_led(whichLED index = whichLED::LED_1,
                 uint8_t brightness = 0x1f,
                 uint8_t blue = 0xff,
                 uint8_t green = 0xff,
                 uint8_t red = 0xff);

    bool update();
};

inline apa102LED::apa102LED(transportAvalon &transport) : m_transport(transport)
{
    for (int led = static_cast<int>(whichLED::LED_1); led <= static_cast<int>(whichLED::LED_8); ++led)
    {
        set_led(static_cast<whichLED>(led), 0x1F, 0, 0, 0xff);
    }
}

inline apa102LED::~apa102LED()
{
}

inline bool apa102LED::txComplete(int errno, size_t bytesSent)
{
    return true;
}

inline bool apa102LED::rxComplete(int errno, const transportAvalon::array_t &rxBuffer)
{
    return true;
}

inline void apa102LED::set_led(whichLED index,
                               uint8_t brightness,
                               uint8_t blue,
                               uint8_t green,
                               uint8_t red)
{
    uint32_t val = ((brightness | 0xE0) << 24) | (blue << 16) | (green << 8) | red;
    val |= 0xFF000000;
    m_leds[static_cast<size_t>(index)] = val;
}

inline bool apa102LED::update()
{

    avalonProtocol protocol;
    avalonProtocol::array_t tx_packet;
    avalonProtocol::array_t data;

    auto rxCallback = std::bind(&apa102LED::rxComplete, this,
                                std::placeholders::_1,
                                std::placeholders::_2);

    auto txCallback = std::bind(&apa102LED::txComplete, this,
                                std::placeholders::_1,
                                std::placeholders::_2);

    for (auto &val : m_leds)
    {
        data.push_back(val & 0xFF);
        data.push_back((val >> 8) & 0xFF);
        data.push_back((val >> 16) & 0xFF);
        data.push_back((val >> 24) & 0xFF);
    }

    data.push_back(0);
    data.push_back(0);
    data.push_back(0);
    data.push_back(0);
    protocol.transaction_channel_write(tx_packet,
                                       APA102_START_ADDRESS +4,
                                       AVALON_CHANNEL,
                                       data,
                                       Avalon_Trans_Type::WRITE_INCREMENTING);

    m_transport.postTransaction(tx_packet, txCallback, rxCallback);
    return true;
}
