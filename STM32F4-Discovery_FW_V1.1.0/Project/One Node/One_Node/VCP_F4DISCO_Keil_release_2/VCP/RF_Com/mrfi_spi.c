/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Radios: CC1101
 *   SPI interface code.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "mrfi_spi.h"
#include "mrfi_board_defs.h"
#define int8 int8_t
/* ------------------------------------------------------------------------------------------------
 *                                            Defines
 * ------------------------------------------------------------------------------------------------
 */
#define DUMMY_BYTE                  0xDB

#define READ_BIt                    0x80
#define BURST_BIT                   0x40


/* ------------------------------------------------------------------------------------------------
 *                                            Macros
 * ------------------------------------------------------------------------------------------------
 */
#define MRFI_SPI_TURN_CHIP_SELECT_ON()        MRFI_SPI_DRIVE_CSN_LOW()
#define MRFI_SPI_TURN_CHIP_SELECT_OFF()       MRFI_SPI_DRIVE_CSN_HIGH()
#define MRFI_SPI_CHIP_SELECT_IS_OFF()         MRFI_SPI_CSN_IS_HIGH()


/* ------------------------------------------------------------------------------------------------
 *                                       Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static uint8 spiRegAccess(uint8 addrByte, uint8 writeValue);
static void spiBurstFifoAccess(uint8 addrByte, uint8 * pData, uint8 len);


/**************************************************************************************************
 * @fn          mrfiSpiInit
 *
 * @brief       Initialize SPI.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiInit(void)
{
  /* configure all SPI related pins */
  MRFI_SPI_CONFIG_CSN_PIN_AS_OUTPUT();
  MRFI_SPI_CONFIG_SCLK_PIN_AS_OUTPUT();
  MRFI_SPI_CONFIG_SI_PIN_AS_OUTPUT();
  MRFI_SPI_CONFIG_SO_PIN_AS_INPUT();

  /* set CSn to default high level */
  MRFI_SPI_DRIVE_CSN_HIGH();
  
  /* initialize the SPI registers */
  //MRFI_SPI_INIT();
} 


/**************************************************************************************************
 * @fn          mrfiSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio.  Returns status byte read during transfer
 *              of strobe command.
 *
 * @param       addr - address of register to strobe
 *
 * @return      status byte of radio
 **************************************************************************************************
 */
int8 mrfiSpiCmdStrobe(uint8 addr)
{
  uint8 statusByte;
  mrfiSpiIState_t s;

  if(!((addr >= 0x30) && (addr <= 0x3D)))  /* invalid address */
	return(-1);

  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);

  /* turn chip select "off" and then "on" to clear any current SPI access */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();

  /* send the command strobe, wait for SPI access to complete */
  MRFI_SPI_WRITE_BYTE(addr);
  MRFI_SPI_WAIT_DONE();

  /* read the radio status byte returned by the command strobe */
  statusByte = MRFI_SPI_READ_BYTE();

  /* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_EXIT_CRITICAL_SECTION(s);

  /* return the status byte */
  return(statusByte);
}


/**************************************************************************************************
 * @fn          mrfiSpiReadReg
 *
 * @brief       Read value from radio register.
 *
 * @param       addr - address of register
 *
 * @return      register value
 **************************************************************************************************
 */
uint8 mrfiSpiReadReg(uint8 addr)
{
    
  /*
   *  The burst bit is set to allow access to read-only status registers.
   *  This does not affect normal register reads.
   */
  return( spiRegAccess(addr | BURST_BIT | READ_BIt, DUMMY_BYTE) );
}


/**************************************************************************************************
 * @fn          mrfiSpiWriteReg
 *
 * @brief       Write value to radio register.
 *
 * @param       addr  - address of register
 * @param       value - register value to write
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiWriteReg(uint8 addr, uint8 value)
{                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
  spiRegAccess(addr, value);
}


/*=================================================================================================
 * @fn          spiRegAccess
 *
 * @brief       This function performs a read or write.  The
 *              calling code must configure the read/write bit of the register's address byte.
 *              This bit is set or cleared based on the type of access.
 *
 * @param       regAddrByte - address byte of register; the read/write bit already configured
 *
 * @return      register value
 *=================================================================================================
 */
static uint8 spiRegAccess(uint8 addrByte, uint8 writeValue)
{
  uint8 readValue;
  mrfiSpiIState_t s;

  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);

  /* turn chip select "off" and then "on" to clear any current SPI access */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();

  /* send register address byte, the read/write bit is already configured */
  MRFI_SPI_WRITE_BYTE(addrByte);
  MRFI_SPI_WAIT_DONE();

  /*
   *  Send the byte value to write.  If this operation is a read, this value
   *  is not used and is just dummy data.  Wait for SPI access to complete.
   */
  MRFI_SPI_WRITE_BYTE(writeValue);
  MRFI_SPI_WAIT_DONE();

  /*
   *  If this is a read operation, SPI data register now contains the register
   *  value which will be returned.  For a read operation, it contains junk info
   *  that is not used.
   */
  readValue = MRFI_SPI_READ_BYTE();

  /* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_EXIT_CRITICAL_SECTION(s);

  /* return the register value */
  return(readValue);
}


/**************************************************************************************************
 * @fn          mrfiSpiWriteTxFifo
 *
 * @brief       Write data to radio transmit FIFO.
 *
 * @param       pData - pointer for storing write data
 * @param       len   - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiWriteTxFifo(uint8 * pData, uint8 len)
{
  spiBurstFifoAccess(TXFIFO | BURST_BIT, pData, len);
}


/**************************************************************************************************
 * @fn          macSpiReadRxFifo
 *
 * @brief       Read data from radio receive FIFO.
 *
 * @param       pData - pointer for storing read data
 * @param       len   - length of data in bytes
 *
 * @return      none
 **************************************************************************************************
 */
void mrfiSpiReadRxFifo(uint8 * pData, uint8 len)
{
  spiBurstFifoAccess(RXFIFO | BURST_BIT | READ_BIt, pData, len);
}


/*=================================================================================================
 * @fn          spiBurstFifoAccess
 *
 * @brief       Burst mode access used for reading or writing to radio FIFOs.
 *
 *              For more efficient interrupt latency, this function does not keep interrupts
 *              disabled for its entire execution.  It is designed to recover if an interrupt
 *              occurs that accesses SPI.  See comments in code for further details.
 *
 * @param       addrByte - first byte written to SPI, contains address and mode bits
 * @param       pData    - pointer to data to read or write
 * @param       len      - length of data in bytes
 *
 * @return      none
 *=================================================================================================
 */
static void spiBurstFifoAccess(uint8 addrByte, uint8 * pData, uint8 len)
{
  mrfiSpiIState_t s;

  /* disable interrupts that use SPI */
  MRFI_SPI_ENTER_CRITICAL_SECTION(s);

  /* turn chip select "off" and then "on" to clear any current SPI access */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_TURN_CHIP_SELECT_ON();

  /*-------------------------------------------------------------------------------
   *  Main loop.  If the SPI access is interrupted, execution comes back to
   *  the start of this loop.  Loop exits when nothing left to transfer.
   */
  do
  {
    /* send FIFO access command byte, wait for SPI access to complete */
    MRFI_SPI_WRITE_BYTE(addrByte);
    MRFI_SPI_WAIT_DONE();

    /*-------------------------------------------------------------------------------
     *  Inner loop.  This loop executes as long as the SPI access is not interrupted.
     *  Loop completes when nothing left to transfer.
     */
    do
    {
      MRFI_SPI_WRITE_BYTE(*pData);
        
      /*-------------------------------------------------------------------------------
       *  Use idle time.  Perform increment/decrement operations before pending on
       *  completion of SPI access.
       *
       *  Decrement the length counter.  Wait for SPI access to complete.
       */
      len--;
      MRFI_SPI_WAIT_DONE();

      /*-------------------------------------------------------------------------------
       *  SPI data register holds data just read.  If this is a read operation,
       *  store the value into memory.
       */
      if (addrByte & READ_BIt)
      {
        *pData = MRFI_SPI_READ_BYTE();
      }

      /*-------------------------------------------------------------------------------
       *  At least one byte of data has transferred.  Briefly enable (and then disable)
       *  interrupts that can call SPI functions.  This provides a window for any timing
       *  critical interrupts that might be pending.
       *
       *  To improve latency, take care of pointer increment within the interrupt
       *  enabled window.
       */
      MRFI_SPI_EXIT_CRITICAL_SECTION(s);
      pData++;
      MRFI_SPI_ENTER_CRITICAL_SECTION(s);

      /*-------------------------------------------------------------------------------
       *  If chip select is "off" the SPI access was interrupted (all SPI access
       *  functions leave chip select in the "off" state).  In this case, turn
       *  back on chip select and break to the main loop.  The main loop will
       *  pick up where the access was interrupted.
       */
      if (MRFI_SPI_CHIP_SELECT_IS_OFF())
      {
        MRFI_SPI_TURN_CHIP_SELECT_ON();
        break;
      }

    /*-------------------------------------------------------------------------------
     */
    } while (len); /* inner loop */
  } while (len);   /* main loop */

  /* turn off chip select; enable interrupts that call SPI functions */
  MRFI_SPI_TURN_CHIP_SELECT_OFF();
  MRFI_SPI_EXIT_CRITICAL_SECTION(s);
}


/**************************************************************************************************
*/
