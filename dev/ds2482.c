/*** FILEHEADER ****************************************************************
 *
 *    FILENAME:    ds2482.c
 *    DATE:        04.08.2011
 *    AUTHOR:      Christian Stadler
 *    CC2538 port: Kiril Petrov /28.10.2014/
 *
 *    DESCRIPTION: DS2482-800 1-Wire I2C bridge driver.
 *                 Based on DALLAS APPLICATION NOTE 3684:
 *                 How to Use the DS2482 I²C 1-Wire® Master
 *
 ******************************************************************************/

#include "contiki.h"
#include "i2c.h"
#include "ds2482.h"

#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define POLL_LIMIT      16

// DS2482 state
unsigned char I2C_address;
int c1WS, cSPU, cPPM, cAPU;
int short_detected;

// Search state
int LastDiscrepancy;
int LastFamilyDiscrepancy;
int LastDeviceFlag;
unsigned char crc8;

/*---------------------------------------------------------------------------*/
struct i2c_msg {
	uint8_t addr;	/* slave addres					*/
	uint8_t flags;
#define I2C_M_RD	0x0001  /* read data, from slave to master	*/
#define I2C_M_RECV_LEN	0x0400  /* length will be first received byte	*/
	uint8_t len;	/* msg length					*/
	uint8_t *buf;	/* pointer to msg data				*/
};

/*******************************************************************************
 * FUNCTION:   OneWire_AddrDevice
 * PURPOSE:    Addresses a single or all devices on the 1-wire bus.
 *
 * INPUT:      nAddrMethod       use ONEWIRE_CMD_MATCHROM to select a single
 *                               device or ONEWIRE_CMD_SKIPROM to select all
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
void OneWire_AddrDevice(uint8_t nAddrMethod)
{
    uint8_t i;

    if (nAddrMethod == ONEWIRE_CMD_MATCHROM)
    {
        OWWriteByte(ONEWIRE_CMD_MATCHROM);    /* address single devices on bus */
        for (i = 0; i < ONEWIRE_ROM_LENGTH; i ++)
        {
            OWWriteByte(ROM_NO[i]);
        }
    }
    else
    {
        OWWriteByte(ONEWIRE_CMD_SKIPROM);     /* address all devices on bus */
    }
}


/*******************************************************************************
 * FUNCTION:   OneWire_CheckCRC
 * PURPOSE:    Checks if CRC is ok (ROM Code or Scratch PAD RAM)
 *
 * INPUT:      buf                  buffer to be checked for proper CRC
 *             buflen               buffer length
 * OUTPUT:     OneWire_nRomAddr_au8[]       ROM code of the first device
 * RETURN:     bool                 TRUE if there are more devices on the 1-wire
 *                                  bus, FALSE otherwise
 ******************************************************************************/
bool OneWire_CheckCRC(uint8_t *buf, uint8_t buflen)
{
    uint8_t shift_reg = 0;
    uint8_t data_bit;
    uint8_t sr_lsb;
    uint8_t fb_bit;
    uint8_t i;
    uint8_t j;
    bool retval;

    for (i=0; i<buflen; i++)    /* for each byte */
    {
        for (j=0; j<8; j++)     /* for each bit */
        {
            data_bit = (buf[i] >> j) & 0x01;
            sr_lsb = shift_reg & 0x01;
            fb_bit = (data_bit ^ sr_lsb) & 0x01;
            shift_reg = shift_reg >> 1;
            if (fb_bit)
            {
                shift_reg = shift_reg ^ 0x8c;
            }
        }
    }

    retval = (shift_reg == 0) ? true : false;

    return (retval);
}


//--------------------------------------------------------------------------
// DS2428 Detect routine that sets the I2C address and then performs a
// device reset followed by writing the configuration byte to default values:
//   1-Wire speed (c1WS) = standard (0)
//   Strong pullup (cSPU) = off (0)
//   Presence pulse masking (cPPM) = off (0)
//   Active pullup (cAPU) = on (CONFIG_APU = 0x01)
//
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//
int DS2482_detect(unsigned char addr)
{
   i2c_init(I2C_CONF_SDA_PORT,
           I2C_CONF_SDA_PIN,
           I2C_CONF_SCL_PORT,
           I2C_CONF_SCL_PIN,
           I2C_SCL_NORMAL_BUS_SPEED);

   // set global address
   I2C_address = addr;

   // reset the DS2482 ON selected address
   if (!DS2482_reset()) {
      PRINTF("%s: Unable to reset ds2482, exit\n", __FUNCTION__);
      return false;
   }

   // write the default configuration setup
   if (!DS2482_write_config(CFG_APU))
      return false;

   return true;
}


//--------------------------------------------------------------------------
// Perform a device reset on the DS2482
//
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//
int DS2482_reset()
{
   unsigned char status;

   // Device Reset
   //   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
   //  [] indicates from slave
   //  SS status byte to read to verify state

   i2c_single_send(I2C_address, CMD_DRST);
   i2c_single_receive(I2C_address, &status);

   // check for failure due to incorrect read back of status
   return ((status & 0xF7) == 0x10);
}


//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration
// options are provided in the lower nibble of the provided config byte.
// The uppper nibble in bitwise inverted when written to the DS2482.
//
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//
int DS2482_write_config(unsigned char config)
{
   unsigned char read_config;
   uint8_t temp[2] = { CMD_WCFG, config | (~config << 4) };

   // Write configuration (Case A)
   //   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
   //  [] indicates from slave
   //  CF configuration byte to write
   i2c_burst_send(I2C_address, temp, 2);
   i2c_single_receive(I2C_address, &read_config);

   // check for failure due to incorrect read back
   if (config != read_config)
   {
      // handle error
      // ...
      DS2482_reset();

      return false;
   }

   return true;
}

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#define EIO		5      /* I/O error */

/*---------------------------------------------------------------------------*/
int i2c_transfer(struct i2c_msg *msgs, int num)
{
	int ret, i;
#ifdef DEBUG
	for (ret = 0; ret < num; ret++) {
		PRINTF("%s: [%d] %c, addr=0x%02x, "
		"len=%d%s\n", __func__, ret, (msgs[ret].flags & I2C_M_RD)
		? 'R' : 'W', msgs[ret].addr, msgs[ret].len,
		(msgs[ret].flags & I2C_M_RECV_LEN) ? "+" : "");
	}
#endif

	for (i = 0; i < num; i++) {
		i2c_master_enable();
		if ((msgs[i].flags & I2C_M_RD))
			ret = i2c_burst_receive(msgs[i].addr, msgs[i].buf, msgs[i].len);
		else
			ret = i2c_burst_send(msgs[i].addr, msgs[i].buf, msgs[i].len);
		if (ret != 0)
			break;
	}
	if (ret == 0)
		ret = num;
	return ret;
}

/*---------------------------------------------------------------------------*/
int ds2482_getreg(uint8_t regaddr, uint8_t *val)
{
	int stat;
	struct i2c_msg msgs[] = {
		{
			.addr = DS2482_ADDRESS,
			.flags = 0,
			.len = 1,
			.buf = &regaddr,
		},
		{
			.addr = DS2482_ADDRESS,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
        };

	stat = i2c_transfer(msgs, ARRAY_SIZE(msgs));

	if (stat < 0)
		PRINTF("ds2482: I2C read error: %d\n", stat);
	else if (stat != ARRAY_SIZE(msgs)) {
		PRINTF("ds2482: I2C read N mismatch: %d\n", stat);
		stat = -EIO;
	} else
		stat = 0;

	return stat;
}

/*---------------------------------------------------------------------------*/
int ds2482_setreg(uint8_t regaddr, uint8_t val)
{
	int stat;
	uint8_t txbuf[2] = { regaddr, val };
	struct i2c_msg msgs[] = {
		{
			.addr = DS2482_ADDRESS,
			.flags = 0,
			.len = 2,
			.buf = txbuf,
		}
	};

	stat = i2c_transfer(msgs, ARRAY_SIZE(msgs));

	if (stat < 0)
		PRINTF("ds2482: I2C send error: %d\n", stat);
	else if (stat != ARRAY_SIZE(msgs)) {
		PRINTF("ds2482: I2C send N mismatch: %d\n", stat);
		stat = -EIO;
	} else
		stat = 0;

	return stat;
}

//--------------------------------------------------------------------------
//
int DS2482_set_pullup(void)
{
	/* note: it seems like both SPU and APU have to be set! */
	return DS2482_write_config(CFG_APU | CFG_SPU);
}

//--------------------------------------------------------------------------
// Select the 1-Wire channel on a DS2482-800.
//
// Returns: TRUE if channel selected
//          FALSE device not detected or failure to perform select
//
int DS2482_channel_select(int channel)
{
   unsigned char ch, ch_read, check;
   uint8_t temp[2] = { CMD_CHSL, 0 };

   // Channel Select (Case A)
   //   S AD,0 [A] CHSL [A] CC [A] Sr AD,1 [A] [RR] A\ P
   //  [] indicates from slave
   //  CC channel value
   //  RR channel read back
   switch (channel)
   {
      default: case 0: ch = 0xF0; ch_read = 0xB8; break;
      case 1: ch = 0xE1; ch_read = 0xB1; break;
      case 2: ch = 0xD2; ch_read = 0xAA; break;
      case 3: ch = 0xC3; ch_read = 0xA3; break;
      case 4: ch = 0xB4; ch_read = 0x9C; break;
      case 5: ch = 0xA5; ch_read = 0x95; break;
      case 6: ch = 0x96; ch_read = 0x8E; break;
      case 7: ch = 0x87; ch_read = 0x87; break;
   };
   temp[1] = ch;
   i2c_burst_send(I2C_address, temp, 2);
   i2c_single_receive(I2C_address, &check);

   // check for failure due to incorrect read back of channel
   return (check == ch_read);
}


//--------------------------------------------------------------------------
// Reset all of the devices on the 1-Wire Net and return the result.
//
// Returns: TRUE(1):  presence pulse(s) detected, device(s) reset
//          FALSE(0): no presence pulses detected
//
int OWReset(void)
{
   unsigned char status;
   int poll_count = 0;

   // 1-Wire reset (Case B)
   //   S AD,0 [A] 1WRS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                   \--------/
   //                       Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   i2c_single_send(I2C_address, CMD_1WRS);

   // loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
   do
   {
      i2c_single_receive(I2C_address, &status);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return false;
   }

   // check for short condition
   if (status & STATUS_SD)
      short_detected = true;
   else
      short_detected = false;

   // check for presence detect
   if (status & STATUS_PPD)
      return true;
   else
      return false;
}


//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net.
// The parameter 'sendbit' least significant bit is used.
//
// 'sendbit' - 1 bit to send (least significant byte)
//
void OWWriteBit(unsigned char sendbit)
{
   OWTouchBit(sendbit);
}


//--------------------------------------------------------------------------
// Reads 1 bit of communication from the 1-Wire Net and returns the
// result
//
// Returns:  1 bit read from 1-Wire Net
//
unsigned char OWReadBit(void)
{
   return OWTouchBit(0x01);
}


//--------------------------------------------------------------------------
// Send 1 bit of communication to the 1-Wire Net and return the
// result 1 bit read from the 1-Wire Net. The parameter 'sendbit'
// least significant bit is used and the least significant bit
// of the result is the return bit.
//
// 'sendbit' - the least significant bit is the bit to send
//
// Returns: 0:   0 bit read from sendbit
//          1:   1 bit read from sendbit
//
unsigned char OWTouchBit(unsigned char sendbit)
{
   unsigned char status;
   int poll_count = 0;
   uint8_t temp[2] = { CMD_1WSB, sendbit ? 0x80 : 0x00 };

   // 1-Wire bit (Case B)
   //   S AD,0 [A] 1WSB [A] BB [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                          \--------/
   //                           Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  BB indicates byte containing bit value in msbit
   i2c_burst_send(I2C_address, temp, 2);

   // loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
   do
   {
      i2c_single_receive(I2C_address, &status);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return 0;
   }

   // return bit state
   if (status & STATUS_SBR)
      return 1;
   else
      return 0;
}


//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and verify that the
// 8 bits read from the 1-Wire Net are the same (write operation).
// The parameter 'sendbyte' least significant 8 bits are used.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  TRUE: bytes written and echo was the same
//           FALSE: echo was not the same
//
void OWWriteByte(unsigned char sendbyte)
{
   unsigned char status;
   int poll_count = 0;
   uint8_t temp[2] = { CMD_1WWB, sendbyte };

   // 1-Wire Write Byte (Case B)
   //   S AD,0 [A] 1WWB [A] DD [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                          \--------/
   //                             Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  DD data to write
   i2c_burst_send(I2C_address, temp, 2);

   // loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
   do
   {
      i2c_single_receive(I2C_address, &status);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
   }
}

//--------------------------------------------------------------------------
// Send 8 bits of read communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net.
//
// Returns:  8 bits read from 1-Wire Net
//
unsigned char OWReadByte(void)
{
   unsigned char data, status;
   int poll_count = 0;
   uint8_t temp[2] = { CMD_SRP, 0xE1 };

   /* 1-Wire Read Bytes (Case C)
   //   S AD,0 [A] 1WRB [A] Sr AD,1 [A] [Status] A [Status] A\
   //                                   \--------/
   //                     Repeat until 1WB bit has changed to 0
   //   Sr AD,0 [A] SRP [A] E1 [A] Sr AD,1 [A] DD A\ P
   //
   //  [] indicates from slave
   //  DD data read
   */
   i2c_single_send(I2C_address, CMD_1WRB);

   // loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
   do
   {
      i2c_single_receive(I2C_address, &status);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return 0;
   }

   i2c_burst_send(I2C_address, temp, 2);
   i2c_single_receive(I2C_address, &data);

   return data;
}


//--------------------------------------------------------------------------
// The 'OWBlock' transfers a block of data to and from the
// 1-Wire Net. The result is returned in the same buffer.
//
// 'tran_buf' - pointer to a block of unsigned
//              chars of length 'tran_len' that will be sent
//              to the 1-Wire Net
// 'tran_len' - length in bytes to transfer
//
void OWBlock(unsigned char *tran_buf, int tran_len)
{
   int i;

   for (i = 0; i < tran_len; i++)
      tran_buf[i] = OWTouchByte(tran_buf[i]);
}


//--------------------------------------------------------------------------
// Send 8 bits of communication to the 1-Wire Net and return the
// result 8 bits read from the 1-Wire Net. The parameter 'sendbyte'
// least significant 8 bits are used and the least significant 8 bits
// of the result are the return byte.
//
// 'sendbyte' - 8 bits to send (least significant byte)
//
// Returns:  8 bits read from sendbyte
//
unsigned char OWTouchByte(unsigned char sendbyte)
{
   if (sendbyte == 0xFF)
      return OWReadByte();
   else
   {
      OWWriteByte(sendbyte);
      return sendbyte;
   }
}


//--------------------------------------------------------------------------
// Find the 'first' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : no device present
//
int OWFirst()
{
   // reset the search state
   LastDiscrepancy = 0;
   LastDeviceFlag = false;
   LastFamilyDiscrepancy = 0;

   return OWSearch();
}


//--------------------------------------------------------------------------
// Find the 'next' devices on the 1-Wire network
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
int OWNext()
{
   // leave the search state alone
   return OWSearch();
}


//--------------------------------------------------------------------------
// The 'OWSearch' function does a general search. This function
// continues from the previous search state. The search state
// can be reset by using the 'OWFirst' function.
// This function contains one parameter 'alarm_only'.
// When 'alarm_only' is TRUE (1) the find alarm command
// 0xEC is sent instead of the normal search command 0xF0.
// Using the find alarm command 0xEC will limit the search to only
// 1-Wire devices that are in an 'alarm' state.
//
// Returns:   TRUE (1) : when a 1-Wire device was found and its
//                       Serial Number placed in the global ROM
//            FALSE (0): when no new device was found.  Either the
//                       last search was the last device or there
//                       are no devices on the 1-Wire Net.
//
int OWSearch()
{
   int id_bit_number;
   int last_zero, rom_byte_number, search_result;
   int id_bit, cmp_id_bit;
   unsigned char rom_byte_mask, search_direction, status;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;
   crc8 = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!OWReset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      OWWriteByte(0xF0);

      // loop to do the search
      do
      {
         // if this discrepancy if before the Last Discrepancy
         // on a previous next then pick the same as last time
         if (id_bit_number < LastDiscrepancy)
         {
            if ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0)
               search_direction = 1;
            else
               search_direction = 0;
         }
         else
         {
            // if equal to last pick 1, if not then pick 0
            if (id_bit_number == LastDiscrepancy)
               search_direction = 1;
            else
               search_direction = 0;
         }

         // Perform a triple operation on the DS2482 which will perform
         // 2 read bits and 1 write bit
         status = DS2482_search_triplet(search_direction);

         // check bit results in status byte
         id_bit = ((status & STATUS_SBR) == STATUS_SBR);
         cmp_id_bit = ((status & STATUS_TSB) == STATUS_TSB);
         search_direction =
             ((status & STATUS_DIR) == STATUS_DIR) ? 1 : 0;

         // check for no devices on 1-Wire
         if ((id_bit) && (cmp_id_bit))
            break;
         else
         {
            if ((!id_bit) && (!cmp_id_bit) && (search_direction == 0))
            {
               last_zero = id_bit_number;

               // check for Last discrepancy in family
               if (last_zero < 9)
                  LastFamilyDiscrepancy = last_zero;
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
               ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
               ROM_NO[rom_byte_number] &= (uint8_t)~rom_byte_mask;

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number
            // and reset mask
            if (rom_byte_mask == 0)
            {
               calc_crc8(ROM_NO[rom_byte_number]);  // accumulate the CRC
               rom_byte_number++;
               rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!((id_bit_number < 65) || (crc8 != 0)))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag
         // search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = true;

         search_result = true;
      }
   }

   // if no device found then reset counters so next
   // 'search' will be like a first

   if (!search_result || (ROM_NO[0] == 0))
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   }

   return search_result;
}


//--------------------------------------------------------------------------
// Verify the device with the ROM number in ROM_NO buffer is present.
// Return TRUE  : device verified present
//        FALSE : device not present
//
int OWVerify()
{
   unsigned char rom_backup[8];
   int i,rslt,ld_backup,ldf_backup,lfd_backup;

   // keep a backup copy of the current state
   for (i = 0; i < 8; i++)
      rom_backup[i] = ROM_NO[i];
   ld_backup = LastDiscrepancy;
   ldf_backup = LastDeviceFlag;
   lfd_backup = LastFamilyDiscrepancy;

   // set search to find the same device
   LastDiscrepancy = 64;
   LastDeviceFlag = false;

   if (OWSearch())
   {
      // check if same device found
      rslt = true;
      for (i = 0; i < 8; i++)
      {
         if (rom_backup[i] != ROM_NO[i])
         {
            rslt = false;
            break;
         }
      }
   }
   else
     rslt = false;

   // restore the search state
   for (i = 0; i < 8; i++)
      ROM_NO[i] = rom_backup[i];
   LastDiscrepancy = ld_backup;
   LastDeviceFlag = ldf_backup;
   LastFamilyDiscrepancy = lfd_backup;

   // return the result of the verify
   return rslt;
}


//--------------------------------------------------------------------------
// Setup the search to find the device type 'family_code' on the next call
// to OWNext() if it is present.
//
void OWTargetSetup(unsigned char family_code)
{
   int i;

   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = false;
}


//--------------------------------------------------------------------------
// Setup the search to skip the current device type on the next call
// to OWNext().
//
void OWFamilySkipSetup()
{
   // set the Last discrepancy to last family discrepancy
   LastDiscrepancy = LastFamilyDiscrepancy;
   LastFamilyDiscrepancy = 0;

   // check for end of list
   if (LastDiscrepancy == 0)
      LastDeviceFlag = true;
}


//--------------------------------------------------------------------------
// Use the DS2482 help command '1-Wire triplet' to perform one bit of a
//1-Wire search.
//This command does two read bits and one write bit. The write bit
// is either the default direction (all device have same bit) or in case of
// a discrepancy, the 'search_direction' parameter is used.
//
// Returns – The DS2482 status byte result from the triplet command
//
unsigned char DS2482_search_triplet(int search_direction)
{
   unsigned char status;
   int poll_count = 0;
   uint8_t temp[2] = { CMD_1WT, search_direction ? 0x80 : 0x00 };

   // 1-Wire Triplet (Case B)
   //   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                         \--------/
   //                           Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  SS indicates byte containing search direction bit value in msbit
   i2c_burst_send(I2C_address, temp, 2);

   // loop checking 1WB bit for completion of 1-Wire operation
   // abort if poll limit reached
   do
   {
      i2c_single_receive(I2C_address, &status);
   }
   while ((status & STATUS_1WB) && (poll_count++ < POLL_LIMIT));

   // check for failure due to poll limit reached
   if (poll_count >= POLL_LIMIT)
   {
      // handle error
      // ...
      DS2482_reset();
      return 0;
   }

   // return status byte
   return status;
}


//--------------------------------------------------------------------------
// Calculate the CRC8 of the byte value provided with the current
// global 'crc8' value.
// Returns current global crc8 value
//
unsigned char calc_crc8(unsigned char data)
{
   int i;

   // See Application Note 27
   crc8 = crc8 ^ data;
   for (i = 0; i < 8; ++i)
   {
      if (crc8 & 1)
         crc8 = (crc8 >> 1) ^ 0x8c;
      else
         crc8 = (crc8 >> 1);
   }

   return crc8;
}


