#ifndef _MODBUS_SLAVE_H_
#define _MODBUS_SLAVE_H_

#include "scidrv.h" //SCI_RX_BUFFER_SIZE
#include "hal_obj.h" // System Include

//
// DO NOT MODIFY THESE LINES.
//
#define CPU_RATE         16.667L   // for a 60MHz CPU clock speed (SYSCLKOUT)
#define DELAY_US(A)      usDelay(((((long double) A * 1000.0L) /\
                         (long double)CPU_RATE) - 9.0L) / 5.0L)

typedef enum
{
    MODBUS_IDLE              = 0x00,
    MODBUS_TX_RESPONSE_BUSY  = 0x01,
    MODBUS_TX_ERROR_BUSY     = 0x02,
    MODBUS_RX_REQUEST_BUSY   = 0x03,
    MODBUS_RX_RESPONSE_BUSY  = 0x04,
    MODBUS_TX_WAIT_T35       = 0x05
}MB_STATE_E;

// Modbus slave Variables
typedef struct
{
    GPIO_Number_e tx_enable_pin;
    uint16_t frame_buf[SCI_RX_BUFFER_SIZE];
    uint16_t bufsize;
    uint16_t holdreg_size; // size of the register array
    uint16_t framesize; //size of frame to be transmitted
    uint16_t byteTxed; // bytes transmitted for current frame
    uint16_t *reg; // user array address
    uint16_t id;
    uint16_t fc;
    uint16_t err_count;
    uint32_t pass_count;
    MB_STATE_E state;
} MB_T;

extern MB_T mb;
extern uint32_t Cputimer1IntCount;
extern uint32_t Cputimer2IntCount;

// function definitions
uint16_t modbus_update();
void mbSlaveInit(uint16_t id, GPIO_Number_e enpin, uint16_t regsize, uint16_t *reg);

#endif /* _MODBUS_SLAVE_H_*/
