#ifndef __LR_H__
#define __LR_H__
#include <stdint.h>


// Like Intel, Arduino is little endian

#define ZB_RX_MAX_PAYLOAD   76
#define LR_LIST_LEN_IR      4
#define LR_LIST_LEN_CFG     2

#define LR_MSG_TYPE_IR      'i'
#define LR_MSG_TYPE_CFG     'c'
#define LR_MSG_VERSION_1    '1'

typedef char LrStructId[2]; // Should always be 'LR'

typedef struct {
    uint8_t proto;
    uint8_t nbits;
    uint8_t repeats;
    uint8_t repeatDelay;
    uint64_t command;
} 
__attribute__((packed))
IrCmd;

typedef struct {
    char name[16];
    union {
        char cvalue[16];
        uint32_t ivalue0;
        uint32_t ivalue1;
        uint32_t ivalue2;
        uint32_t ivalue3;
    } v;
}
__attribute__((packed))
ConfigCmd;

typedef struct {
    LrStructId id;
    uint8_t version;
    uint8_t msgType;
    union {
        IrCmd ir[LR_LIST_LEN_IR];
        ConfigCmd cfg[LR_LIST_LEN_CFG];
    } cmds;
}  
__attribute__((packed))
LrMsg
;

typedef union {
    char data_buf[ZB_RX_MAX_PAYLOAD];
    LrMsg lrMsg;
} 
__attribute__((packed))
ZbMsg;


void sony(long command, int nbits);
void nec(long command, int nbits);

void handleLrMsg(LrMsg *lrMsg, char *txMsg, size_t txMsgLen);

#endif

