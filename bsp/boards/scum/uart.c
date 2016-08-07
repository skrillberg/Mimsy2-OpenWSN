/**
\brief SCuM-specific definition of the "uart" bsp module.

\author Tengfei Chang <tengfei.chang@inria.fr>, August 2016.
*/

#include "memory_map.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "uart.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   uart_tx_cbt txCb;
   uart_rx_cbt rxCb;
} uart_vars_t;

uart_vars_t uart_vars;

//=========================== prototypes ======================================

//=========================== public ==========================================

void uart_init() {
    // reset local variables
    memset(&uart_vars,0,sizeof(uart_vars_t));
}

void uart_setCallbacks(uart_tx_cbt txCb, uart_rx_cbt rxCb) {
    uart_vars.txCb = txCb;
    uart_vars.rxCb = rxCb;
}

void    uart_writeByte(uint8_t byteToWrite){
    UART_REG__TX_DATA = byteToWrite;
    // there is no txdone interruption, call the handler directly
    uart_tx_isr();
}

void    uart_enableInterrupts(){ 
}

uint8_t uart_readByte(){
    return UART_REG__RX_DATA;
}

//=========================== interrupt handlers ==============================

kick_scheduler_t uart_tx_isr() {
    uart_vars.txCb();
    return DO_NOT_KICK_SCHEDULER;
}

kick_scheduler_t uart_rx_isr() {
    uart_vars.rxCb();
    return DO_NOT_KICK_SCHEDULER;
}