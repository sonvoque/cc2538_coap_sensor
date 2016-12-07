/*
 *
 */
#ifndef __DS2482_H__
#define __DS2482_H__

#include <stdbool.h>

#define ONEWIRE_CMD_SEARCHROM     0xF0
#define ONEWIRE_CMD_READROM       0x33
#define ONEWIRE_CMD_MATCHROM      0x55
#define ONEWIRE_CMD_SKIPROM       0xCC
#define ONEWIRE_CMD_ALARMSEARCH   0xEC

/*
 * Configure Register bit definitions
 * The top 4 bits always read 0.
 * To write, the top nibble must be the 1's compl. of the low nibble.
 */
#define CFG_APU		(1 << 0)	/* active pull-up */
#define CFG_PPM		(1 << 1)	/* presence pulse masking */
#define CFG_SPU		(1 << 2)	/* strong pull-up */
#define CFG_1WS		(1 << 3)	/* 1-wire speed */

#define CMD_DRST        0xF0
#define CMD_WCFG        0xD2
#define CMD_CHSL        0xC3
#define CMD_1WRS        0xB4
#define CMD_1WSB        0x87
#define CMD_1WWB        0xA5
#define CMD_1WRB        0x96
#define CMD_SRP         0xE1
#define CMD_1WT         0x78

#define STATUS_1WB      (1 << 0)
#define STATUS_PPD      (1 << 1)
#define STATUS_SD       (1 << 2)
#define STATUS_LL       (1 << 3)
#define STATUS_RST      (1 << 4)
#define STATUS_SBR      (1 << 5)
#define STATUS_TSB      (1 << 6)
#define STATUS_DIR      (1 << 7)

#define ONEWIRE_ROM_LENGTH        8
#define DS2482_NUM_CHANNELS       8
#define ONEWIRE_SCRATCHPAD_LENGTH 9

#define DS2482_ADDRESS  0x18

unsigned char ROM_NO[ONEWIRE_ROM_LENGTH];
unsigned char read_result[ONEWIRE_SCRATCHPAD_LENGTH];

int DS2482_detect(unsigned char addr);
int DS2482_reset();
int DS2482_write_config(unsigned char config);
int DS2482_channel_select(int channel);
int DS2482_set_pullup(void);

int OWReset(void);
void OWWriteBit(unsigned char sendbit);
unsigned char OWReadBit(void);
unsigned char OWTouchBit(unsigned char sendbit);
void OWWriteByte(unsigned char sendbyte);
unsigned char OWReadByte(void);
void OWBlock(unsigned char *tran_buf, int tran_len);
unsigned char OWTouchByte(unsigned char sendbyte);
int OWFirst();
int OWNext();
int OWSearch();
unsigned char DS2482_search_triplet(int search_direction);
unsigned char calc_crc8(unsigned char data);
void OWTargetSetup(unsigned char family_code);
void OWFamilySkipSetup();
int OWVerify();

#endif /* __DS2482_H__ */
