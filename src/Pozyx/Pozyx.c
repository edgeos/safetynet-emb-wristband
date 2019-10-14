/**
* Pozyx_core.cpp
* --------------
* This file contains the defintion of the core POZYX functions and variables
*
*/
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "Pozyx.h"
#include "Pozyx_definitions.h"
#include "nrf_log.h"

static Pozyx_dev_t *i2c_dev;

/**
 * The interrupt handler for the pozyx device: keeping it uber short!
 */
//void IRQ()
//{
//  _interrupt = 1;
//}

bool waitForFlag(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
//  long timer = millis();
//  int status;

  // stay in this loop until the event interrupt flag is set or until the the timer runs out
//  while(millis()-timer < timeout_ms)
//  {
    // in polling mode, we insert a small delay such that we don't swamp the i2c bus
    //if( _mode == MODE_POLLING ){
      //delay(1);
    //}

//    if( (_interrupt == 1) || (_mode == MODE_POLLING))
//    {
//      _interrupt = 0;

      // Read out the interrupt status register. After reading from this register, pozyx automatically clears the interrupt flags.
//      uint8_t interrupt_status = 0;
//      status = regRead(POZYX_INT_STATUS, &interrupt_status, 1);
//      if((interrupt_status & interrupt_flag) && status == POZYX_SUCCESS)
//      {
        // one of the interrupts we were waiting for arrived!
//        if(interrupt != NULL)
//          *interrupt = interrupt_status;
//        return true;
//      }
//    }
  //}
  // too bad, pozyx didn't respond
  // 1) pozyx can select from two pins to generate interrupts, make sure the correct pin is connected with the attachInterrupt() function.
  // 2) make sure the interrupt we are waiting for is enabled in the POZYX_INT_MASK register)
  return false;
}

bool waitForFlag_safe(uint8_t interrupt_flag, int timeout_ms, uint8_t *interrupt)
{
//  int tmp = _mode;
//  _mode = MODE_POLLING;
//  bool result = waitForFlag(interrupt_flag, timeout_ms, interrupt);
//  _mode = tmp;
//  return result;
}

int Pozyx_init(Pozyx_dev_t *i2c){

  int status = POZYX_SUCCESS;
  bool print_result = true;

  // check if the mode parameter is valid
  //if((mode != MODE_POLLING) && (mode != MODE_INTERRUPT))
//    return POZYX_FAILURE;

  // check if the pin is valid
//  if((interrupt_pin != 0) && (interrupt_pin != 1))
    //return POZYX_FAILURE;

//  Wire.begin();

  // wait a bit until the pozyx board is up and running
//  delay(250);

  //_mode = mode;

  uint8_t whoami, selftest;
  uint8_t regs[3];
  regs[2] = 0x12;

  i2c_dev = i2c;

  // we read out the first 3 register values: who_am_i, firmware_version and harware version, respectively.
  if(regRead(POZYX_WHO_AM_I, regs, 3) != NRF_SUCCESS){
    return POZYX_FAILURE;
  }
  whoami = regs[0];
  i2c_dev->firmware_ver = regs[1];
  i2c_dev->hardware_ver = regs[2];

  if(true){
    NRF_LOG_INFO("WhoAmI: 0x%X", whoami);
    //Serial.print("WhoAmI: 0x");
    //Serial.println(whoami, HEX);
    NRF_LOG_INFO("FW ver.: v%d.%d", (i2c_dev->firmware_ver&0xF0)>>4, (i2c_dev->firmware_ver&0x0F));
    //Serial.print("FW ver.: v");
    //Serial.print((_fw_version&0xF0)>>4);
    //Serial.print(".");
    //Serial.print((_fw_version&0x0F));
    if(i2c_dev->firmware_ver < 0x10)
      NRF_LOG_INFO("please upgrade");
      //Serial.print(" (please upgrade)");
    //Serial.print("\nHW ver.: ");
    NRF_LOG_INFO("HW ver.: %d", i2c_dev->hardware_ver);
    //Serial.println(_hw_version);
  }
  // verify if the whoami is correct
  if(whoami != 0x43) {
    // possibly the pozyx is not connected right. Also make sure the jumper of the boot pins is present.
//    status = POZYX_FAILURE;
  }

  // readout the selftest registers to validate the proper functioning of pozyx
  if(regRead(POZYX_ST_RESULT, &selftest, 1) != NRF_SUCCESS){
    return POZYX_FAILURE;
  }

  if(print_result){
    NRF_LOG_INFO("selftest: 0x%X", selftest);
    //Serial.print("selftest: 0b");
    //Serial.println(selftest, BIN);
  }

  if((i2c_dev->hardware_ver & POZYX_TYPE) == POZYX_TAG)
  {
    // check if the uwb, pressure sensor, accelerometer, magnetometer and gyroscope are working
    if(selftest != 0b00111111) {
      status = POZYX_FAILURE;
    }
  }else if((i2c_dev->hardware_ver & POZYX_TYPE) == POZYX_ANCHOR)
  {
    // check if the uwb transceiver and pressure sensor are working
    if(selftest != 0b00110000) {
      status = POZYX_FAILURE;
    }
    return status;
  }

//  if(_mode == MODE_INTERRUPT){
    // set the function that must be called upon an interrupt
    // put your main code here, to run repeatedly:
#if defined(__SAMD21G18A__) || defined(__ATSAMD21G18A__)
    // Arduino Tian
    int tian_interrupt_pin = interrupt_pin;
    attachInterrupt(interrupt_pin+2, IRQ, RISING);
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    // Arduino UNO, Mega
    attachInterrupt(interrupt_pin, IRQ, RISING);
#else
//    Serial.println("This is not a board supported by Pozyx, interrupts may not work");
    //attachInterrupt(interrupt_pin, IRQ, RISING);
#endif

    // use interrupt as provided and initiate the interrupt mask
//    uint8_t int_mask = interrupts;
//    configInterruptPin(5+interrupt_pin, PIN_MODE_PUSHPULL, PIN_ACTIVE_LOW, 0);

//    if (regWrite(POZYX_INT_MASK, &int_mask, 1) == POZYX_FAILURE){
//      return POZYX_FAILURE;
//    }
//  }

  // all done
//  delay(POZYX_DELAY_LOCAL_WRITE);
  return status;
}

/**
  * Reads a number of bytes from the specified pozyx register address using I2C
  */
int regRead(uint8_t reg_address, uint8_t *pData, int size)
{
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c reads in smaller chunks

  if(!IS_REG_READABLE(reg_address))
    return POZYX_FAILURE;

  if(i2c_dev == NULL) return POZYX_FAILURE;
  if(pData == NULL) return POZYX_FAILURE;
  if(size <= 0) return POZYX_FAILURE;

  int offset = 0;
  int status = 1;
  uint8_t reg = reg_address;

  while(size-offset > BUFFER_LENGTH)
  {
    //status &= i2cWriteRead(reg, 1, pData+offset, BUFFER_LENGTH);
    status = i2c_dev->read(POZYX_I2C_ADDRESS, reg, pData+offset, BUFFER_LENGTH);
    offset += BUFFER_LENGTH;
    reg += BUFFER_LENGTH;
  }
  status = i2c_dev->read(POZYX_I2C_ADDRESS, reg, pData+offset, size-offset);

  return status;
}

/**
  * Writes a number of bytes to the specified pozyx register address using I2C
  */
int regWrite(uint8_t reg_address, uint8_t *pData, int size)
{
  // BUFFER_LENGTH is defined in wire.h, it limits the maximum amount of bytes that can be transmitted/received with i2c in one go
  // because of this, we may have to split up the i2c writes in smaller chunks

  if(!IS_REG_WRITABLE(reg_address))
    return POZYX_FAILURE;

  if(i2c_dev == NULL) return POZYX_FAILURE;
  if(pData == NULL) return POZYX_FAILURE;
  if(size <= 0) return POZYX_FAILURE;

  int offset = 0;
  int status = 1;
  uint8_t reg = reg_address;

  while(size-offset > BUFFER_LENGTH)
  {
    status &= i2c_dev->write(POZYX_I2C_ADDRESS, reg, pData+offset, BUFFER_LENGTH);
    offset += BUFFER_LENGTH;
    reg += BUFFER_LENGTH;
  }
  status &= i2c_dev->write(POZYX_I2C_ADDRESS, reg, pData+offset, size-offset);

  return status;
}

/**
  * Call a register function using i2c with given parameters, the data from the function is stored in pData
  */
int regFunction(uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  assert(BUFFER_LENGTH >= size+1);           // Arduino-specific code for the i2c
  assert(BUFFER_LENGTH >= param_size+1);     // Arduino-specific code for the i2c
  if(i2c_dev == NULL) return POZYX_FAILURE;
  if(params == NULL) return POZYX_FAILURE;
  if(param_size == 0) return POZYX_FAILURE;

  if(!IS_FUNCTIONCALL(reg_address))
    return POZYX_FAILURE;

  uint8_t status;

  // this feels a bit clumsy with all these memcpy's
//  uint8_t write_data[param_size+1];
//  write_data[0] = reg_address;
//  memcpy(write_data+1, params, param_size);
  uint8_t read_data[size+1];

  // first write some data with i2c and then read some data
  status = i2c_dev->write(POZYX_I2C_ADDRESS, reg_address, params, param_size);
  if(pData != NULL && size > 0) {
    status = i2c_dev->read(POZYX_I2C_ADDRESS, reg_address, read_data, size+1);
    if(status)
      return POZYX_FAILURE;
    memcpy(pData, read_data+1, size);
  }

  // the first byte that a function returns is always it's success indicator, so we simply pass this through
  return read_data[0];
}


/**
 * Wirelessly write a number of bytes to a specified register address on a remote Pozyx device using UWB.
 */
int remoteRegWrite(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_WRITABLE(reg_address))      return POZYX_FAILURE;    // the register is not writable
  if(size > MAX_BUF_SIZE-1)              return POZYX_FAILURE;    // trying to write too much data

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;              // the first byte is the register address we want to start writing to.
  memcpy(tmp_data+2, pData, size);         // the remaining bytes are the data bytes to be written starting at the register address.
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, size+2, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x04;    // flag to indicate a register write

  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  if (waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 100, &int_status)){
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      // An error occured during positioning.
      // Please read out the register POZYX_ERRORCODE to obtain more information about the error
      return POZYX_FAILURE;
    }else{
      return POZYX_SUCCESS;
    }
  }else{
    return POZYX_TIMEOUT;
  }

  return status;
}

/**
 * Wirelessly read a number of bytes from a specified register address on a remote Pozyx device using UWB.
 */
int remoteRegRead(uint16_t destination, uint8_t reg_address, uint8_t *pData, int size)
{
  // some checks
  if(!IS_REG_READABLE(reg_address))      return POZYX_FAILURE;        // the register is not readable
  if(size > MAX_BUF_SIZE)                return POZYX_FAILURE;        // trying to read too much data
  if(destination == 0)                   return POZYX_FAILURE;        // remote read not allowed in broadcast mode

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[3];
  tmp_data[0] = 0;                  // the offset in the TX buffer
  tmp_data[1] = reg_address;        // the first byte is the register address we want to start reading from
  tmp_data[2] = size;               // the number of bytes to read starting from the register address
  status = regFunction(POZYX_TX_DATA, (uint8_t *)&tmp_data, 3, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x02;    // flag to indicate a register read

  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // wait up to x ms to receive a response
  if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
  {
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      // An error occured during positioning.
      // Please read out the register POZYX_ERRORCODE to obtain more information about the error
      return POZYX_FAILURE;
    }else{
      // we received a response, now get some information about the response
      uint8_t rx_info[3]= {0,0,0};
      regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
      uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
      uint8_t data_len = rx_info[2];

      if( remote_network_id == destination && data_len == size)
      {
        status = readRXBufferData(pData, size);
        return status;
      }else{
        return POZYX_FAILURE;
      }
    }

  }else{
    // timeout
    return POZYX_TIMEOUT;
  }
}

/*
 * Wirelessly call a register function with given parameters on a remote Pozyx device using UWB, the data from the function is stored in pData
 */
int remoteRegFunction(uint16_t destination, uint8_t reg_address, uint8_t *params, int param_size, uint8_t *pData, int size)
{
  // some checks
  if(!IS_FUNCTIONCALL(reg_address))      return POZYX_FAILURE;        // the register is not a function

  int status = 0;

  // first prepare the packet to send
  uint8_t tmp_data[param_size+2];
  tmp_data[0] = 0;
  tmp_data[1] = reg_address;                // the first byte is the function register address we want to call.
  memcpy(tmp_data+2, params, param_size);   // the remaining bytes are the parameter bytes for the function.
  status = regFunction(POZYX_TX_DATA, tmp_data, param_size+2, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
  {
    return status;
  }

  // send the packet
  uint8_t tx_params[3];
  tx_params[0] = (uint8_t)destination;
  tx_params[1] = (uint8_t)(destination>>8);
  tx_params[2] = 0x08;    // flag to indicate a register function call
  uint8_t int_status = 0;
  regRead(POZYX_INT_STATUS, &int_status, 1);      // first clear out the interrupt status register by reading from it
  status = regFunction(POZYX_TX_SEND, tx_params, 3, NULL, 0);

  // stop if POZYX_TX_SEND returned an error.
  if(status == POZYX_FAILURE){
    return status;
  }

  // wait up to x ms to receive a response
  if(waitForFlag_safe(POZYX_INT_STATUS_FUNC | POZYX_INT_STATUS_ERR, 1000, &int_status))
  {
    if((int_status & POZYX_INT_STATUS_ERR) == POZYX_INT_STATUS_ERR)
    {
      return POZYX_FAILURE;
    }else
    {
      // we received a response, now get some information about the response
      uint8_t rx_info[3];
      regRead(POZYX_RX_NETWORK_ID, rx_info, 3);
      uint16_t remote_network_id = rx_info[0] + ((uint16_t)rx_info[1]<<8);
      uint8_t data_len = rx_info[2];

      if( remote_network_id == destination && data_len == size+1)
      {
        uint8_t return_data[size+1];

        status = readRXBufferData(return_data, size+1);

        if(status == POZYX_FAILURE){
          // debug information
          return status;
        }

        memcpy(pData, return_data+1, size);

        return return_data[0];
      }else{
        return POZYX_FAILURE;
      }
    }

  }else{
    // timeout
    return POZYX_TIMEOUT;
  }
}

int writeTXBufferData(uint8_t data[], int size, int offset)
{
  if (offset + size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }

  int status = 1;
  uint8_t max_bytes = BUFFER_LENGTH-1;
  uint8_t params[BUFFER_LENGTH];
  uint8_t i_offset = 0;

  while(size-i_offset > max_bytes) {
    params[0] = offset + i_offset;      // the offset
    memcpy(params+1, data+i_offset, max_bytes);
    status &= regFunction(POZYX_TX_DATA, params, max_bytes, NULL, 0);
    i_offset += max_bytes;
  }
  params[0] = offset + i_offset;      // the offset
  memcpy(params+1, data+i_offset, size-i_offset);
  status &= regFunction(POZYX_TX_DATA, params, size-i_offset, NULL, 0);

  return status;
}

int readRXBufferData(uint8_t* pData, int size)
{
  if (size > MAX_BUF_SIZE){
    return POZYX_FAILURE;
  }

  int status;
  uint8_t params[2];
  uint8_t max_bytes = BUFFER_LENGTH-1;
  uint8_t offset = 0;

  while(size-offset > max_bytes) {
    params[0] = offset;      // the offset
    params[1] = max_bytes;      // the number of bytes to read
    status = regFunction(POZYX_RX_DATA, params, 2, pData+params[0], params[1]);
    offset += max_bytes;
  }
  params[0] = offset;      // the offset
  params[1] = size - offset;      // the number of bytes to read
  status = regFunction(POZYX_RX_DATA, params, 2, pData+params[0], params[1]);

  return status;
}

int sendTXBufferData(uint16_t destination)
{
  int status;

  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);
//  delay(POZYX_DELAY_LOCAL_FUNCTION);

  return status;
}


/*
 * This function sends some data bytes to the destination
 */
int sendData(uint16_t destination, uint8_t *pData, int size)
{
  if(size > MAX_BUF_SIZE)          return POZYX_FAILURE;        // trying to send too much data

  uint8_t status = 0;

  uint8_t tmp_data[size+1];
  tmp_data[0] = 0;                        // the first byte is the offset byte.
  memcpy(tmp_data+1, pData, size);

  // set the TX buffer
  status = regFunction(POZYX_TX_DATA, tmp_data, size+1, NULL, 0);

  // stop if POZYX_TX_DATA returned an error.
  if(status == POZYX_FAILURE)
    return status;

  // send the packet
  uint8_t params[3];
  params[0] = (uint8_t)destination;
  params[1] = (uint8_t)(destination>>8);
  params[2] = 0x06;    // flag to indicate we're just sending data
  status = regFunction(POZYX_TX_SEND, (uint8_t *)&params, 3, NULL, 0);

  return status;
}


