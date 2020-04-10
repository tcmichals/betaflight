#include <cstdint>
#include <functional>
#include "addressmap.h"
#include "avalonProtocol.h"
#include "transport.h"

#define PWM_DECODERS_ 6
#define PWM_OFF_MASK 0x80000000
class pwmDecoderAvalon
{

public:
	typedef std::function<void(uint32_t pwm1,
							   uint32_t pwm2,
							   uint32_t pwm3,
							   uint32_t pwm4,
							   uint32_t pwm5,
							   uint32_t pwm6)>
		updateCalllback_t;

protected:
	std::array<uint32_t, PWM_DECODERS_> m_pwmDecoder;
	transportAvalon &m_transport;
	updateCalllback_t m_callBackFunc;

protected:
	bool rxComplete(int errno, const transportAvalon::array_t &rxBuffer);
	bool txComplete(int errno, size_t bytesSent);

public:
	pwmDecoderAvalon(transportAvalon &transport);
	virtual ~pwmDecoderAvalon();
	bool postRead();
	void updateCalllback(updateCalllback_t callback);
};

inline pwmDecoderAvalon::pwmDecoderAvalon(transportAvalon &transport) : m_transport(transport)
{
}

inline pwmDecoderAvalon::~pwmDecoderAvalon()
{
}

inline void pwmDecoderAvalon::updateCalllback(updateCalllback_t callback)
{
	m_callBackFunc = callback;
}

inline bool pwmDecoderAvalon::txComplete(int errno, size_t bytesSent)
{
	return true;
}

inline bool pwmDecoderAvalon::rxComplete(int errno, const transportAvalon::array_t &rxBuffer)
{
	std::array<uint16_t, PWM_DECODERS_> pwm = {0, 0, 0, 0, 0, 0};

	if (rxBuffer.size() == (sizeof(uint32_t) * PWM_DECODERS_))
	{
		for (size_t indx = 0; indx < PWM_DECODERS_; ++indx)
		{
			m_pwmDecoder[indx] = *(reinterpret_cast<const uint32_t *>(rxBuffer.data()) + indx);
		}
		if (m_callBackFunc)
		{
			m_callBackFunc(m_pwmDecoder[0],
						   m_pwmDecoder[1],
						   m_pwmDecoder[2],
						   m_pwmDecoder[3],
						   m_pwmDecoder[4],
						   m_pwmDecoder[5]);
		}

		return true;
	}
	else
	{
		std::cout << __PRETTY_FUNCTION__ << " ERROR wrong size " << rxBuffer.size() << std::endl;
		return false;
	}
}

inline bool pwmDecoderAvalon::postRead()
{
	avalonProtocol protocol;
	avalonProtocol::array_t tx_packet;

	auto rxCallback = std::bind(&pwmDecoderAvalon::rxComplete, this,
								std::placeholders::_1,
								std::placeholders::_2);

	auto txCallback = std::bind(&pwmDecoderAvalon::txComplete, this,
								std::placeholders::_1,
								std::placeholders::_2);

	protocol.transaction_channel_read(tx_packet,
									  PWM_DECODER_START_ADDRESS,
									  AVALON_CHANNEL,
									  sizeof(uint32_t) * PWM_DECODERS_,
									  Avalon_Trans_Type::READ_INCREMENTING);

	return m_transport.postTransaction(tx_packet, txCallback, rxCallback);
}
