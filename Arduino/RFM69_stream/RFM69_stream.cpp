#include <RFM69.h>
#include <RFM69registers.h>
#include <SPI.h>
#include <RFM69_stream.h>

// constructor
RFM69_stream::RFM69_stream(uint8_t slaveSelectPin, uint8_t interruptPin, bool isRFM69HW, uint8_t interruptNum)
	: RFM69(slaveSelectPin, interruptPin, isRFM69HW, interruptNum)
{
}

void RFM69_stream::streamingMode()
{
  //we go into Unlimited Length Packet Mode
  writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF); // FORMAT=0
  writeReg(REG_PAYLOADLENGTH, 0x00); // LENGTH=0
  writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_10); // FSK, packet mode, noise shaping
  //TODO: adjust paRamp register for FSK mode (probably not needed in unlimited length mode)
}

void RFM69_stream::dataRate10k()
{
  // write bitrate dependent registers
  writeReg(REG_BITRATEMSB, RF_BITRATEMSB_100000); // 100 kbps
  writeReg(REG_BITRATELSB, RF_BITRATELSB_100000);
  writeReg(REG_FDEVMSB, RF_FDEVMSB_100000); // 10KHz, (FDEV + BitRate / 2 <= 500KHz)
  writeReg(REG_FDEVLSB, RF_FDEVLSB_100000);
  writeReg(REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_0); // (BitRate < 2 * RxBw)
}

//Works for TX, but not for RX for some reason
bool RFM69_stream::returnToPacketMode()
{
  //return register steeing back to defaults the were written in initilaize function
  writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_VARIABLE | RF_PACKET1_DCFREE_MANCHESTER | RF_PACKET1_CRC_ON | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF);
  writeReg(REG_PAYLOADLENGTH, 66); // in variable length mode: the max frame size, not used in TX
  writeReg(REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_OOK | RF_DATAMODUL_MODULATIONSHAPING_00); // no shaping
  writeReg(REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF);
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);

  //clear FIFO
  while(digitalRead(_interruptPin))
  {
	  readReg(REG_FIFO);
  }

  setHighPower(_isRFM69HW);
  setMode(RF69_MODE_STANDBY);

  unsigned long start = millis();
  uint8_t timeout = 50;

  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis()-start < timeout); // wait for ModeReady
  if (millis()-start >= timeout)
    return false;

}


void RFM69_stream::streamingModeRX()
{

  detachInterrupt(_interruptNum); //we dont' want to do a normal receive

  writeReg(REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF); // FORMAT=0
  writeReg(REG_PAYLOADLENGTH, 0x00); // LENGTH=0

  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)
	writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_10); // set DIO0 to "syncAddress" in receive mode
  setMode(RF69_MODE_RX);
}

uint8_t RFM69_stream::receiveStreamByte()
{
  return readReg(REG_FIFO);
}


void RFM69_stream::stream(uint8_t toAddress, const void* buffer, uint8_t bufferSize)
{
  setMode(RF69_MODE_STANDBY); // turn off receiver to prevent reception while filling fifo
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // wait for ModeReady
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00); // DIO0 is "Packet Sent"
  if (bufferSize > RF69_MAX_DATA_LEN) bufferSize = RF69_MAX_DATA_LEN;

  // control byte
  uint8_t CTLbyte = 0x00;

  // write to FIFO
  select();
  SPI.transfer(REG_FIFO | 0x80);
  SPI.transfer(bufferSize + 3);
  SPI.transfer(toAddress);
  SPI.transfer(_address);
  SPI.transfer(CTLbyte);

  for (uint8_t i = 0; i < bufferSize; i++)
	SPI.transfer(((uint8_t*) buffer)[i]);
  unselect();

  // no need to wait for transmit mode to be ready since its handled by the radio
  setMode(RF69_MODE_TX);

}

// internal function - interrupt gets called when fifo is not full for streaming mode
void RFM69_stream::sendStreamByte(uint8_t data) {
  if(digitalRead(_interruptPin) == 0 )
  {
	// fill another byte into fifo if the transmission is still running
	select();
	SPI.transfer(REG_FIFO | 0x80);
	SPI.transfer(data);//write data into fifo
	unselect();
  }
}

