//20210822
#include "serial.h"

SCI_Handle    sciAHandle;
//SCI_Handle    sciBHandle;
serial_s serial;


interrupt void SCIRXINTAISR(void){
    while(SCI_getRxFifoStatus(sciAHandle) != SCI_FifoStatus_Empty){//loop until receive buffer is empty check SciRegs.SCIRXST.bit.RXRDY when not using FIFO
        serial.receivedByte = (char)SCI_getDataFIFO(sciAHandle);
//        serial_receiveHandle(serial.receivedByte);
    }
    if(SCI_getRxFifoOVF(sciAHandle)){
        SCI_clearRxFifoOvf(sciAHandle);
    }
    SCI_clearRxFifoInt(sciAHandle);
    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_9);
}


interrupt void SCITXINTAISR(void){//disabled
//    while(SCI_getTxFifoStatus(sciAHandle) != SCI_FifoStatus_4_Words)){
//        SCI_putDataNonBlocking(sciAHandle, 'b');
//    }
//    serial.txReady = true;
//    SCI_clearTxFifoInt(sciAHandle);
    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_9);
}

//interrupt void SCIRXINTBISR(void){
//    while(SCI_getRxFifoStatus(sciBHandle) != SCI_FifoStatus_Empty){//loop until receive buffer is empty check SciRegs.SCIRXST.bit.RXRDY when not using FIFO
//        serial.receivedByte = (char)SCI_getDataFIFO(sciBHandle);
//        serial_receiveHandle(serial.receivedByte);
//    }
//    if(SCI_getRxFifoOVF(sciBHandle)){
//        SCI_clearRxFifoOvf(sciBHandle);
//    }
//    SCI_clearRxFifoInt(sciBHandle);
//    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_9);
//}
//
//
//interrupt void SCITXINTBISR(void){//disabled
////    while(SCI_getTxFifoStatus(sciAHandle) != SCI_FifoStatus_4_Words)){
////        SCI_putDataNonBlocking(sciAHandle, 'b');
////    }
////    serial.txReady = true;
////    SCI_clearTxFifoInt(sciAHandle);
//    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_9);
//}


void serial_init(HAL_Handle handle){
    HAL_Obj *obj = (HAL_Obj *)handle;

    //SCI A
    // initialize the SCI handles
    sciAHandle = SCI_init((void *)SCIA_BASE_ADDR,sizeof(SCI_Obj));
    SCI_resetChannels(sciAHandle);//SCIRST. SCI FIFO can resume transmit or receive. SCIRST should be 1 even for Autobaud logic to work.
    SCI_reset(sciAHandle);// Relinquish SCI after system reset, does not affect SCI configuration, only reset state/flag
    SCI_enableTx(sciAHandle);
    SCI_enableRx(sciAHandle);
    //8-N-1
    SCI_disableParity(sciAHandle);
    SCI_setNumStopBits(sciAHandle,SCI_NumStopBits_One);
    SCI_setCharLength(sciAHandle,SCI_CharLength_8_Bits);
    SCI_setMode(sciAHandle,SCI_Mode_IdleLine);
    // set baud rate
    SCI_setBaudRate(sciAHandle,SCI_BaudRate_9_6_kBaud);
    SCI_setPriority(sciAHandle,SCI_Priority_FreeRun);
    //FIFO
    SCI_enableFifo(sciAHandle);//SCIFFENA. FIFO enhancements are enabled
    //TX FIFO
    SCI_setTxDelay(sciAHandle, 0);//3ms = 29
    SCI_clearTxFifoInt(sciAHandle);//TXFFINT CLR = 1. Write 1 to clear TXFFINT flag in bit 7
    SCI_setTxFifoIntLevel(sciAHandle,SCI_FifoLevel_Empty);
//    SCI_enableTxFifoInt(sciAHandle);//enable TX FIFO interrupt
    //RX FIFO
    SCI_clearRxFifoInt(sciAHandle);//RXFFINT CLR = 1. Write 1 to clear RXFFINT flag in bit 7
    SCI_setRxFifoIntLevel(sciAHandle,SCI_FifoLevel_4_Words);//16bytes
    SCI_enableRxFifoInt(sciAHandle);//enable RX FIFO interrupt
    //configure isr vector. SCIRXINTA is INT9.1, SCITXINTA is INT9.2. pie->SCIRXINTA = &sciARxISR;
    PIE_registerPieIntHandler(obj->pieHandle, PIE_GroupNumber_9, PIE_SubGroupNumber_1, &SCIRXINTAISR);
    PIE_registerPieIntHandler(obj->pieHandle, PIE_GroupNumber_9, PIE_SubGroupNumber_2, &SCITXINTAISR);
    // enable the PIE interrupts associated with the SCI interrupts
    PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIARX);
    PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIATX);
//    SCI_enableRxInt(sciAHandle);
    // enable the cpu interrupt for SCI interrupts
    CPU_enableInt(obj->cpuHandle,CPU_IntNumber_9);
    //enable sci
    SCI_resetRxFifo(sciAHandle);//Re-enable receive FIFO operation
    SCI_resetTxFifo(sciAHandle);//Re-enable transmit FIFO operation
    SCI_enable(sciAHandle);// Relinquish SCI after system reset, does not affect SCI configuration, only reset state/flag


//    //SCI B
//    // initialize the SCI handles
//    sciBHandle = SCI_init((void *)SCIB_BASE_ADDR,sizeof(SCI_Obj));
//    SCI_resetChannels(sciBHandle);//SCIRST. SCI FIFO can resume transmit or receive. SCIRST should be 1 even for Autobaud logic to work.
//    SCI_reset(sciBHandle);// Relinquish SCI after system reset, does not affect SCI configuration, only reset state/flag
//    SCI_enableTx(sciBHandle);
//    SCI_enableRx(sciBHandle);
//    //8-N-1
//    SCI_disableParity(sciBHandle);
//    SCI_setNumStopBits(sciBHandle,SCI_NumStopBits_One);
//    SCI_setCharLength(sciBHandle,SCI_CharLength_8_Bits);
//    SCI_setMode(sciBHandle,SCI_Mode_IdleLine);
//    // set baud rate
//    SCI_setBaudRate(sciBHandle,SCI_BaudRate_19_2_kBaud);
//    SCI_setPriority(sciBHandle,SCI_Priority_FreeRun);
//    //FIFO
//    SCI_enableFifo(sciBHandle);//SCIFFENA. FIFO enhancements are enabled
//    //TX FIFO
//    SCI_setTxDelay(sciBHandle, 0);//3ms = 29
//    SCI_clearTxFifoInt(sciBHandle);//TXFFINT CLR = 1. Write 1 to clear TXFFINT flag in bit 7
//    SCI_setTxFifoIntLevel(sciBHandle,SCI_FifoLevel_Empty);
////    SCI_enableTxFifoInt(sciBHandle);//enable TX FIFO interrupt
//    //RX FIFO
//    SCI_clearRxFifoInt(sciBHandle);//RXFFINT CLR = 1. Write 1 to clear RXFFINT flag in bit 7
//    SCI_setRxFifoIntLevel(sciBHandle,SCI_FifoLevel_4_Words);//16bytes
//    SCI_enableRxFifoInt(sciBHandle);//enable RX FIFO interrupt
//    //configure isr vector. SCIRXINTB is INT9.3, SCITXINTB is INT9.4. pie->SCIRXINTB = &sciBRxISR;
//    PIE_registerPieIntHandler(obj->pieHandle, PIE_GroupNumber_9, PIE_SubGroupNumber_3, &SCIRXINTBISR);
//    PIE_registerPieIntHandler(obj->pieHandle, PIE_GroupNumber_9, PIE_SubGroupNumber_4, &SCITXINTBISR);
//    // enable the PIE interrupts associated with the SCI interrupts
//    PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIBRX);
//    PIE_enableInt(obj->pieHandle,PIE_GroupNumber_9,PIE_InterruptSource_SCIBTX);
////    SCI_enableRxInt(sciBHandle);
//    // enable the cpu interrupt for SCI interrupts
//    CPU_enableInt(obj->cpuHandle,CPU_IntNumber_9);
//    //enable sci
//    SCI_resetRxFifo(sciBHandle);//Re-enable receive FIFO operation
//    SCI_resetTxFifo(sciBHandle);//Re-enable transmit FIFO operation
//    SCI_enable(sciBHandle);// Relinquish SCI after system reset, does not affect SCI configuration, only reset state/flag


    //object initialization
    serial.txcq = construct_circularQueue();//member copy of circular queue, slower. TODO: change to input pointer of cq

    return;
}

void serial_loop(void){
    //SCI A
    //transmit the bytes from fifo
    while (!serial.txcq.empty(&serial.txcq) && SCI_txReady(sciAHandle)){
        sciAHandle->SCITXBUF = serial.txcq.deQueue(&serial.txcq);
    }
    //receive the bytes from fifo, which was not handle by rxfifo interrupt
    while(!SCI_getRxFifoInt(sciAHandle) && (SCI_getRxFifoStatus(sciAHandle) != SCI_FifoStatus_Empty)){
        serial.receivedByte = (char)SCI_getDataFIFO(sciAHandle);
//        serial_receiveHandle(serial.receivedByte);
    }

//    //SCI B
//    while (!serial.txcq.empty(&serial.txcq) && SCI_txReady(sciBHandle)){
//        sciBHandle->SCITXBUF = serial.txcq.deQueue(&serial.txcq);
//    }
//    //receive the bytes from fifo, which was not handle by rxfifo interrupt
//    while(!SCI_getRxFifoInt(sciBHandle) && (SCI_getRxFifoStatus(sciBHandle) != SCI_FifoStatus_Empty)){
//        serial.receivedByte = (char)SCI_getDataFIFO(sciBHandle);
//        serial_receiveHandle(serial.receivedByte);
//    }
}

void serial_xmit(char a){
    //transmit a byte, input a byte to software fifo
    serial.txcq.enQueue(a, &serial.txcq);
}

void serial_msg(char * msg){
    // input a c-string to software fifo
    int i;
    for(i = 0;msg[i] != '\0';++i)
    {
        serial_xmit(msg[i]);
    }
}

int serial_trafficMax(){
    return serial.txcq.trafficMax;
}

void serial_print(const char *_format, ...){
    va_list args;//create argument list
    va_start(args, _format);//retrieve the additional arguments after parameter _format
    vsnprintf(serial.cStringBuffer, sizeof(serial.cStringBuffer), _format, args);
    va_end(args);//delete argument list
    serial_msg(serial.cStringBuffer);
}


