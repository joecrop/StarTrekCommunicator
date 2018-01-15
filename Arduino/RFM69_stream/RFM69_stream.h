#ifndef RFM69_stream_h
#define RFM69_stream_h
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater

class RFM69;

class RFM69_stream : public RFM69 {
  public:
	RFM69_stream(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM);
   	void streamingMode();
	bool returnToPacketMode();
	void dataRate10k();
	void streamingModeRX();
	void stream(uint8_t toAddress, const void* buffer, uint8_t bufferSize);
	void sendStreamByte(uint8_t data);
	uint8_t receiveStreamByte();

};

#endif
