#ifndef __LR_H__
#define __LR_H__
#include <stdint.h>

#define ZB_RX_MAX_PAYLOAD    76

#define LR_DATA_BUF_LEN    (ZB_RX_MAX_PAYLOAD - sizeof(LrStructId) - sizeof(uint8_t))

#define LR_MSG_TYPE_IR_CMD 'i'
#define LR_MSG_TYPE_CONFIG 'c'
#define IR_COMMAND_LIST_LEN 4

// Like Intel, Arduino is little endian

typedef char LrStructId[3];

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
    uint8_t msgType;
    union {
        IrCmd ir_cmds[IR_COMMAND_LIST_LEN];
        ConfigCmd config_cmds[2];
        char data_buf[LR_DATA_BUF_LEN];
    } d;
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

#endif
