#ifndef RFM69_stream_h
#define RFM69_stream_h
#include <Arduino.h>            // assumes Arduino IDE v1.0 or greater

class RFM69;

class RFM69_stream : public RFM69 {
   public:

      static volatile uint8_t activeBuffer;
      static volatile uint16_t bufferCounter;
      static volatile uint16_t totalSamplesSent;
      static volatile uint8_t DATALEN0;
      static volatile uint8_t DATALEN1;
      static volatile uint8_t STREAMDATA0[RF69_MAX_DATA_LEN];
      static volatile uint8_t STREAMDATA1[RF69_MAX_DATA_LEN];
      // the Cypher is half the length so we can save room for other data to be
      // sent in the cypher packet to differentiate t from a stream packet
      static volatile uint8_t CYPHER[RF69_MAX_DATA_LEN/2];

      RFM69_stream(uint8_t slaveSelectPin=RF69_SPI_CS, uint8_t interruptPin=RF69_IRQ_PIN, bool isRFM69HW=false, uint8_t interruptNum=RF69_IRQ_NUM);
      void dataRate100k();
      void dataRate200k();
      void bufferStreamByte(uint8_t data);
      uint8_t sendStreamBuffer(uint8_t toAddress);
      uint8_t receiveStreamByte();
      void receiveStreamBuffer();
      uint8_t isStreamConnected();
      void select();
      void streamInit();
      void setCypher(uint8_t *c);
      void generateCypher(uint8_t *buf);

};

#endif
