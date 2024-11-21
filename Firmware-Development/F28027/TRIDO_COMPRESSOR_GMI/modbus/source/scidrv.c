#include "scidrv.h"
#include "modbus_user.h"
#include "modbus_slave.h"
#include "gpio.h"

#ifdef FLASH
#pragma CODE_SECTION(sciaRxFifoIsr,"ramfuncs");
#endif

extern HAL_Obj hal;

__interrupt void sciaTxFifoIsr(void);
__interrupt void sciaRxFifoIsr(void);

void scia_init(void);
void scia_fifo_init(void);

void scia_setup()
{
    // Initialize SCIA GPIO
    GPIO_setPullUp(hal.gpioHandle, GPIO_Number_28, GPIO_PullUp_Enable);
    GPIO_setPullUp(hal.gpioHandle, GPIO_Number_29, GPIO_PullUp_Disable);
    GPIO_setQualification(hal.gpioHandle, GPIO_Number_28, GPIO_Qual_ASync);
    GPIO_setMode(hal.gpioHandle, GPIO_Number_28, GPIO_28_Mode_SCIRXDA);
    GPIO_setMode(hal.gpioHandle, GPIO_Number_29, GPIO_29_Mode_SCITXDA);

    // Interrupts are re-mapped to ISR functions found within this file.
    EALLOW; // Allow write to EALLOW protected registers
    ((PIE_Obj*) hal.pieHandle)->SCIRXINTA = &sciaRxFifoIsr;
    ((PIE_Obj*) hal.pieHandle)->SCITXINTA = &sciaTxFifoIsr;
    EDIS; // Disallow write to EALLOW protected registers

    // Register interrupt handlers in the PIE vector table
    PIE_registerPieIntHandler(hal.pieHandle, PIE_GroupNumber_9, PIE_SubGroupNumber_1, (PIE_IntVec_t) & sciaRxFifoIsr);
    PIE_registerPieIntHandler(hal.pieHandle, PIE_GroupNumber_9, PIE_SubGroupNumber_2, (PIE_IntVec_t) & sciaTxFifoIsr);

    scia_init();        // Init SCI-A
    scia_fifo_init();   // Init SCI-A Fifos

    PIE_enableInt(hal.pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_SCIARX); // Enable interrupts
    PIE_enableInt(hal.pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_SCIATX);

    CPU_enableInt(hal.cpuHandle, CPU_IntNumber_9);

    mb.bufsize = 0; // Fix the SCI hardware reset Tx Int bug
}

void scia_xmit(uint16_t dat)
{
    while (SCI_getTxFifoStatus(hal.sciAHandle) == 4){} //wait if FIFO is full
    hal.sciAHandle->SCITXBUF = dat;
}

uint32_t SciTxIntcount = 0;
__interrupt void sciaTxFifoIsr(void)
{
    if (SCI_getTxFifoStatus(hal.sciAHandle) < 4)
    {
        if (mb.framesize > 0)
        {
            if (mb.byteTxed < mb.framesize)
            {
                hal.sciAHandle->SCITXBUF = mb.frame_buf[mb.byteTxed++];
            }
            else
            {
                mb.byteTxed = 0;
                mb.framesize = 0;
            }
        }
    }
    SciTxIntcount++;

    SCI_clearTxFifoInt(hal.sciAHandle); // Clear SCI Interrupt flag
    PIE_clearInt(hal.pieHandle, PIE_GroupNumber_9); // Issue PIE ACK
    return;
}

__interrupt void sciaRxFifoIsr(void)
{
    to.t15Count = Cputimer1IntCount; // restart timeout counter
    to.t15TimeoutFlag = 0;

    mb.frame_buf[mb.bufsize++] = hal.sciAHandle->SCIRXBUF; // Read data

    if (mb.bufsize == SCI_RX_BUFFER_SIZE)
    {
        mb.bufsize = 0; // Reset if buffer is overflow
    }

    SCI_clearRxFifoOvf(hal.sciAHandle); // Clear Overflow flag
    SCI_clearRxFifoInt(hal.sciAHandle);  // Clear Interrupt flag
    PIE_clearInt(hal.pieHandle, PIE_GroupNumber_9); // Issue PIE ack
    return;
}

void scia_init()
{
    CLK_enableSciaClock(hal.clkHandle);

    // Default after reset: No loopback, async mode, idle-line protocol
    SCI_disableParity(hal.sciAHandle); // No parity
    SCI_setNumStopBits(hal.sciAHandle, SCI_NumStopBits_One); //Stop bit 1
    SCI_setCharLength(hal.sciAHandle, SCI_CharLength_8_Bits); // 8 bits

    // Default after reset: Enable internal SCICLK, Disable RX ERR, SLEEP, TXWAKE
    SCI_enableTx(hal.sciAHandle); // Enable Tx
    SCI_enableRx(hal.sciAHandle); // Enable Rx
    SCI_enableTxInt(hal.sciAHandle); // Enable Tx Int
    SCI_enableRxInt(hal.sciAHandle); // Enable Rx Int

    SCI_setBaudRate(hal.sciAHandle, MODBUS_BAUDRATE);

    SCI_enable(hal.sciAHandle); //Enable the SCI module

    return;
}

void scia_fifo_init()
{
    SCI_enableTxFifoEnh(hal.sciAHandle); // Enable Tx FIFO
    SCI_resetTxFifo(hal.sciAHandle); // Tx FIFO reset
    SCI_clearTxFifoInt(hal.sciAHandle); // Tx FIFO Int flag reset
    SCI_resetChannels(hal.sciAHandle); //
    SCI_setTxFifoIntLevel(hal.sciAHandle, SCI_FifoLevel_Empty);
    SCI_disableTxFifoInt(hal.sciAHandle); // Disable Tx FIFO Int
    //SCI_enableTxFifoInt(hal.sciAHandle); // Enable Tx FIFO Int

    SCI_resetRxFifo(hal.sciAHandle);  // Rx FIFO reset
    SCI_clearRxFifoInt(hal.sciAHandle); // Rx FIFO Int flag reset
    SCI_setRxFifoIntLevel(hal.sciAHandle, SCI_FifoLevel_1_Word); // Interrupt occurs at 1 FFST
    SCI_enableRxFifoInt(hal.sciAHandle); // Enable Rx FIFO Int

    return;
}

//===========================================================================
// No more.
//===========================================================================
