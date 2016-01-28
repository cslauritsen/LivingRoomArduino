#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "lr.h"
#include <Arduino.h>


void handleLrMsg(LrMsg *lrMsg, char *txMsg, size_t txMsgLen) {
    IrCmd *irCmd = NULL;
    if (0 == memcmp("LR", &lrMsg->id, sizeof(LrStructId))) {
        switch (lrMsg->version) {
            case LR_MSG_VERSION_1:
                switch(lrMsg->msgType) {
                    case LR_MSG_TYPE_IR:
                        for (int i=0; i < LR_LIST_LEN_IR && lrMsg->cmds.ir[i].proto; i++) {
                            irCmd = &lrMsg->cmds.ir[i];
                            switch(irCmd->proto) {
                                case 'S':  // Sony 
                                    for (int r=0; r < irCmd->repeats; r++) {
                                        sony(irCmd->command, irCmd->nbits);
                                        if (irCmd->repeatDelay) {
                                            delay(irCmd->repeatDelay);
                                        }
                                    }
                                    snprintf(txMsg, txMsgLen, "sony-cmd:0x%lx\n", irCmd->command);
                                    break;

                                case 'N':  // NEC
                                    for (int r=0; r < irCmd->repeats; r++) {
                                        nec(irCmd->command, irCmd->nbits);
                                        if (irCmd->repeatDelay) {
                                            delay(irCmd->repeatDelay);
                                        }
                                    }
                                    snprintf(txMsg, txMsgLen, "sony-cmd:0x%lx\n", irCmd->command);
                                    break;

                                default:
                                    break;
                            }
                        }
                        break;
                    case LR_MSG_TYPE_CFG:
                        break;
                    default:
                        strncat(txMsg, "cmd-status:UNKCMD\n", txMsgLen);
                        break;
                }
                strncat(txMsg, "cmd-status:OK\n", txMsgLen);
                break;
            default:
                strncat(txMsg, "cmd-status:UNKVER\n", txMsgLen);
                break;
        }
    }
    else {
        snprintf(txMsg, txMsgLen, "cmd-status:%s\n", "BADMSG");
    }
}

