#include <RFM69.h>
#include <RFM69registers.h>
#include <SPI.h>
#include <RFM69_stream.h>

//static uint8_t BYTELEN = 12; //12 bits = 4 bits overhead, 8 bit packet
//static uint8_t BYTESEQ0 = 0;
//static uint8_t BYTESEQ1 = 1;
//static uint8_t BYTESEQ2 = 0;
//static uint8_t BYTESEQ3 = 1;

volatile uint8_t RFM69_stream::activeBuffer;
volatile uint16_t RFM69_stream::bufferCounter;
volatile uint16_t RFM69_stream::totalSamplesSent;
volatile uint8_t RFM69_stream::DATALEN0;
volatile uint8_t RFM69_stream::DATALEN1;
volatile uint8_t RFM69_stream::STREAMDATA0[RF69_MAX_DATA_LEN];
volatile uint8_t RFM69_stream::STREAMDATA1[RF69_MAX_DATA_LEN];
volatile uint8_t RFM69_stream::CYPHER[RF69_MAX_DATA_LEN/2];

// constructor
   RFM69_stream::RFM69_stream(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW, uint8_t interruptNum)
: RFM69(slaveSelectPin, interruptPin, isRFM69HW, interruptNum)
{
   streamInit();
}

void RFM69_stream::dataRate100k()
{
   // write bitrate dependent registers
   writeReg(REG_BITRATEMSB, RF_BITRATEMSB_100000); // 100 kbps
   writeReg(REG_BITRATELSB, RF_BITRATELSB_100000);
   writeReg(REG_FDEVMSB, RF_FDEVMSB_100000); // 100KHz, (FDEV + BitRate / 2 <= 500KHz)
   writeReg(REG_FDEVLSB, RF_FDEVLSB_100000);
   //from table on page 26  (BitRate < 2 * RxBw)
   writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_1); // 250k for FSK
}

void RFM69_stream::dataRate200k()
{
   // write bitrate dependent registers
   writeReg(REG_BITRATEMSB, RF_BITRATEMSB_200000); // 200 kbps
   writeReg(REG_BITRATELSB, RF_BITRATELSB_200000);
   // From page 20: (FDEV + BitRate / 2 <= 500KHz)
   writeReg(REG_FDEVMSB, RF_FDEVMSB_200000); // 200KHz
   writeReg(REG_FDEVLSB, RF_FDEVLSB_200000);
   //from table on page 26: (BitRate < 2 * RxBw)
   writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0); // 500k for FSK
}

// This function looks to see if a packet was just received, if it has it
// transfers the packet to the STREAMDATAx buffers so they can be accessed in
// sequence via receiveStreamByte().
void RFM69_stream::receiveStreamBuffer()
{
   // If the active buffer is 1, we don't want to overwrite it, so we will
   // copy to buffer 0
   if(activeBuffer == 1) {
      if(DATALEN >= RF69_MAX_DATA_LEN) {
         for(int i = 0; i < RF69_MAX_DATA_LEN; i++) {
           STREAMDATA0[i] = DATA[i];
         }
         DATALEN = 0;
         DATALEN0 = 0;
      }
   } else {
      if(DATALEN >= RF69_MAX_DATA_LEN) {
         for(int i = 0; i < RF69_MAX_DATA_LEN; i++) {
            STREAMDATA1[i] = DATA[i];
         }
         DATALEN = 0;
         DATALEN1 = 0;
      }
   }
   bufferCounter = 0;
}

// This function checks to see if we are receieving a regualr stream of packets.
// If we have missed 10 consecutive packets, we assume we are disconnected.
// This functiona should only be called after a connection has been established.
uint8_t RFM69_stream::isStreamConnected()
{
   if(bufferCounter > RF69_MAX_DATA_LEN*10) {
      return 0;
   } else {
      return 1;
   }
}

uint8_t RFM69_stream::receiveStreamByte()
{
   uint8_t data;
   if(bufferCounter++ < RF69_MAX_DATA_LEN) {
      if(activeBuffer == 0) {
         data = STREAMDATA0[DATALEN0] ^ CYPHER[DATALEN0 % (RF69_MAX_DATA_LEN/2)];
         DATALEN0++;
         if(DATALEN0 >= RF69_MAX_DATA_LEN) {
            activeBuffer = 1;
         }
      } else {
         data = STREAMDATA1[DATALEN1] ^ CYPHER[DATALEN1 % (RF69_MAX_DATA_LEN/2)];
         DATALEN1++;
         if(DATALEN1 >= RF69_MAX_DATA_LEN) {
            activeBuffer = 0;
         }
      }
      return data;
   } else {
      // We droped a packet
      if(activeBuffer == 0) {
         return STREAMDATA0[DATALEN0];
      } else {
         return STREAMDATA0[DATALEN1];
      }
   }
}

// add byte to stream buffer
void RFM69_stream::bufferStreamByte(uint8_t data) {
   if(activeBuffer == 0) {
      STREAMDATA0[DATALEN0] = data ^ CYPHER[DATALEN0 % (RF69_MAX_DATA_LEN/2)];
      DATALEN0++;
      if(DATALEN0 == RF69_MAX_DATA_LEN) {
         activeBuffer = 1;
      }
   } else {
      STREAMDATA1[DATALEN1] = data ^ CYPHER[DATALEN1 % (RF69_MAX_DATA_LEN/2)];
      DATALEN1++;
      if(DATALEN1 == RF69_MAX_DATA_LEN) {
         activeBuffer = 0;
      }
   }
}

// Sends buffer to receiver when it is full.
// reutns 0 when the transmission has gon one for too long
uint8_t RFM69_stream::sendStreamBuffer(uint8_t toAddress) {
   uint8_t streamData[RF69_MAX_DATA_LEN];

   // If the active buffer beinf filled is 1, we should send buffer 0 because it
   // should be full
   if(activeBuffer == 1) {
      if(DATALEN0 == RF69_MAX_DATA_LEN) {
         for(int i = 0; i < RF69_MAX_DATA_LEN; i++) {
           streamData[i] = STREAMDATA0[i];
         }
         send(toAddress, streamData, RF69_MAX_DATA_LEN);
         totalSamplesSent++;
         DATALEN0 = 0;
      }
   } else {
      if(DATALEN1 == RF69_MAX_DATA_LEN) {
         for(int i = 0; i < RF69_MAX_DATA_LEN; i++) {
           streamData[i] = STREAMDATA1[i];
         }
         send(toAddress, streamData, RF69_MAX_DATA_LEN);
         totalSamplesSent++;
         DATALEN1 = 0;
      }
   }

   // We limit the total transmission time to 40 seconds = 80us * 61bytes * 8192 packets
   if(totalSamplesSent > 8192) {
      return 0;
   } else {
      return 1;
   }
}

void RFM69_stream::streamInit() {
   activeBuffer = 0;
   bufferCounter = 0;
   totalSamplesSent = 0;
   DATALEN0 = 0;
   DATALEN1 = 0;
}

void RFM69_stream::setCypher(uint8_t *c) {
   for(int i = 0; i < RF69_MAX_DATA_LEN/2; i++) {
      CYPHER[i] = c[i];
   }
}

void RFM69_stream::generateCypher(uint8_t *buf) {
   for(int i = 0; i < RF69_MAX_DATA_LEN/2; i++) {
      CYPHER[i] = random(255);
      buf[i] = CYPHER[i];
   }
}

// select the RFM69 transceiver (save SPI settings, set CS low)
// This is overridden for RFM69 to make the SPI transactions faster
void RFM69_stream::select() {
  noInterrupts();
#if defined (SPCR) && defined (SPSR)
  // save current SPI settings
  _SPCR = SPCR;
  _SPSR = SPSR;
#endif
  // set RFM69 SPI settings
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  // IMPORTANT: This must be disabled to maximize SPI Speed
  //SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2 after SPI stalling in some instances, especially visible on mega1284p when RFM69 and FLASH chip both present
  digitalWrite(_slaveSelectPin, LOW);
}
