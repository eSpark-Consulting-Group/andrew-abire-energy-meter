#include "modbus_user.h"
#include "modbus_slave.h"
#include "main.h"
#include "scidrv.h"
#include "math.h"
#include "gpio.h"
#include "timer.h"

#ifdef FLASH
#pragma DATA_SECTION(mb, "modbus_data");
#endif

#define T35_COUNT        35UL    //35 count * 50us = 1750us
#define T15_COUNT        25UL    //25 count * 50us = 750us

uint16_t calCRC16(uint16_t *pBuf, uint16_t len);
void sendError(uint16_t exception);
void sendResponse(uint16_t bufferSize);

MB_T mb;
uint32_t Cputimer1IntCount = 0;
uint32_t Cputimer2IntCount = 0;
uint32_t msTimer1Count = 0;
extern bool overcurrentFlag;
extern bool warncurrentFlag;
extern union FAULT_T fault;

// Modbus errors
enum
{
    //MODBUS Standard define
    ILLEGAL_FUNCTION     = 0x01,
    ILLEGAL_DATA_ADDRESS = 0x02,
    ILLEGAL_DATA_VALUE   = 0x03,
    //User define
    CRC_ERROR            = 0xE2,
    SERVER_ID_MISMATCH   = 0xE4,
    PACKET_LENGTH_ERROR  = 0xE5,
    PARAMETER_COUNT_ERROR = 0xE6,
};

void currentVdc()
{
    _iq value = (_iq)ADC_readResult(hal.adcHandle, ADC_ResultNumber_7); // divide by 2^numAdcBits = 2^12
    value = _IQ12mpy(value, hal.voltage_sf);
    _iq rawVdc = _IQmpy(value, _IQ(gUserVar.USER_IQ_FULL_SCALE_VOLTAGE_V / 1000.0));
    float DcVoltages = (_IQtoF(rawVdc) * 10000.0L * VOLTAGE_CORRECTION_FACTOR)/10.0;

    if ((DcVoltages * 2) > stableInputVoltage)
        spikeCounter++;
}

__interrupt void cpu_timer1_isr(void)
{
    if(Cputimer1IntCount - to.commCount >= 60000) // 1 sec = 20000 count * 50us
    {
      to.commTimeoutFlag = true;
      to.commCount = Cputimer1IntCount;
    }


    if (Cputimer1IntCount - to.hearbeatCount >= 20000UL) // 1 sec = 20000 count * 50us
    {
        mbmsg.heartbeat_out++;
        to.hearbeatCount = Cputimer1IntCount;
    }

    if (Cputimer1IntCount - to.t15Count >= T15_COUNT) // stop when there is a T15_COUNT gap
    {
        to.t15TimeoutFlag = 1;
        to.t15Count = Cputimer1IntCount;
    }
    else
    {
        to.t15TimeoutFlag = 0;
    }

    if ((stableInputVoltage > mbmsg.min_dcv_input/10.0) && (Cputimer1IntCount - msTimer1Count) >= 1000UL) // 50ms
    {
        currentVdc();
        msTimer1Count = Cputimer1IntCount;
    }

    Cputimer1IntCount++;
}

__interrupt void cpu_timer2_isr(void)
{
    if (mb.state == MODBUS_IDLE)
    {
        GPIO_setLow(hal.gpioHandle, mb.tx_enable_pin); //RX mode
    }
    else if (mb.state == MODBUS_TX_WAIT_T35)
    {
        if (Cputimer1IntCount - to.t35Count > T35_COUNT)
        {
            GPIO_setLow(hal.gpioHandle, mb.tx_enable_pin); //RX mode
            mb.state = MODBUS_IDLE;
        }
    }
    else if (mb.state == MODBUS_TX_RESPONSE_BUSY || mb.state == MODBUS_TX_ERROR_BUSY)
    {
        GPIO_setHigh(hal.gpioHandle, mb.tx_enable_pin); //TX mode
        sendResponse(mb.framesize);
    }

    Cputimer2IntCount++;
}

// Initialize of modbus
void mbSlaveInit(uint16_t id, GPIO_Number_e enpin, uint16_t regsize, uint16_t *reg)
{
    mb.id = id;
    mb.tx_enable_pin = enpin;
    mb.holdreg_size = regsize;
    mb.reg = reg;

    scia_setup();

    // Initialize MAX3485 EN PIN
    GPIO_setPullUp(hal.gpioHandle, mb.tx_enable_pin, GPIO_PullUp_Enable);
    GPIO_setMode(hal.gpioHandle, mb.tx_enable_pin, GPIO_0_Mode_GeneralPurpose); //GPIO_x_Mode_GeneralPurpose = 0
    GPIO_setDirection(hal.gpioHandle, mb.tx_enable_pin, GPIO_Direction_Output);
    GPIO_setLow(hal.gpioHandle, mb.tx_enable_pin);

    // Init var
    mb.err_count = 0;
    mb.pass_count = 0;
    mb.bufsize = 0;
    mb.framesize = 0;
    mb.byteTxed = 0;
    mb.state = MODBUS_IDLE;

    // Setup Timer 1 for timeout management
    EALLOW; // Allow write to EALLOW protected registers
    PIE_registerSystemIntHandler(hal.pieHandle, PIE_SystemInterrupts_TINT1, (PIE_IntVec_t) & cpu_timer1_isr);
    EDIS; // Disallow write to EALLOW protected registers

    TIMER_stop(hal.timerHandle[1]);
    TIMER_setPeriod(hal.timerHandle[1], 60 * 50); //50us

    TIMER_setPreScaler(hal.timerHandle[1], 0);
    TIMER_reload(hal.timerHandle[1]);
    TIMER_setEmulationMode(hal.timerHandle[1], TIMER_EmulationMode_StopAfterNextDecrement);
    TIMER_enableInt(hal.timerHandle[1]);

    TIMER_start(hal.timerHandle[1]);
    CPU_enableInt(hal.cpuHandle, CPU_IntNumber_13);

    // Setup Timer 2 for modbus loop
    EALLOW; // Allow write to EALLOW protected registers
    PIE_registerSystemIntHandler(hal.pieHandle, PIE_SystemInterrupts_TINT2, (PIE_IntVec_t) & cpu_timer2_isr);
    EDIS; // Disallow write to EALLOW protected registers

    TIMER_stop(hal.timerHandle[2]);
    TIMER_setPeriod(hal.timerHandle[2], 60 * 10UL); //10us

    TIMER_setPreScaler(hal.timerHandle[2], 0);
    TIMER_reload(hal.timerHandle[2]);
    TIMER_setEmulationMode(hal.timerHandle[2], TIMER_EmulationMode_StopAfterNextDecrement);
    TIMER_enableInt(hal.timerHandle[2]);

    TIMER_start(hal.timerHandle[2]);
    CPU_enableInt(hal.cpuHandle, CPU_IntNumber_14);

    to.t15Count = Cputimer1IntCount;
}

uint16_t modbus_update()
{
    uint16_t id;
    uint16_t crc;
    uint16_t start_addr;
    uint16_t total_registers;
    uint16_t max_data;
    uint16_t index;
    uint16_t address;
    uint16_t total_bytes;
    uint16_t frame_size;

    if (to.t15TimeoutFlag && mb.state == MODBUS_IDLE) // Seems we got a frame
    {
        // sufficient length is 8 bytes for FC03 & FC16
        if (mb.bufsize > 7)
        {
            id = mb.frame_buf[0];

            if (mb.id == id)  // if the received ID matches the server id, continue
            {
                // get CRC bytes
                crc = (mb.frame_buf[mb.bufsize - 2] << 8 | mb.frame_buf[mb.bufsize - 1]);
                // if the calculated CRC matches the received CRC, continue
                if (calCRC16(mb.frame_buf, mb.bufsize - 2) == crc)
                {
                    mb.fc = mb.frame_buf[1];
                    // get starting address
                    start_addr = (mb.frame_buf[2] << 8 | mb.frame_buf[3]);
                    // get number of register bytes
                    total_registers = (mb.frame_buf[4] << 8 | mb.frame_buf[5]);

                    max_data = start_addr + total_registers;

                    switch (mb.fc)
                    {
                    case 3: // FC03 - Read Holding Registers
                        if (start_addr < mb.holdreg_size) // check exception 2 ILLEGAL DATA ADDRESS
                        {
                            if (max_data <= mb.holdreg_size) // check exception 3 ILLEGAL DATA VALUE
                            {
                                total_bytes = total_registers * 2;
                                mb.frame_buf[0] = mb.id;
                                mb.frame_buf[1] = mb.fc;
                                mb.frame_buf[2] = total_bytes;
                                address = 3; // PDU starts at the 4th byte
                                // Fill data
                                for (index = start_addr; index < max_data; index++)
                                {
                                    mb.frame_buf[address++] = mb.reg[index] >> 8;
                                    mb.frame_buf[address++] = mb.reg[index] & 0xFF;
                                }
                                // ID, FC, total_bytes, (dataLo + dataHi)*number of registers, (crcLo + crcHi)
                                frame_size = 5 + total_bytes;
                                mb.framesize = frame_size;
                                // Fill CRC
                                crc = calCRC16(mb.frame_buf, frame_size - 2);
                                mb.frame_buf[frame_size - 2] = crc >> 8;
                                mb.frame_buf[frame_size - 1] = crc & 0xFF;
                                // Send response
                                mb.state = MODBUS_TX_RESPONSE_BUSY;
                                mb.pass_count++;
                            }
                            else
                            {
                                sendError(ILLEGAL_DATA_VALUE); // exception 3 ILLEGAL DATA VALUE
                            }
                        }
                        else
                        {
                            sendError(ILLEGAL_DATA_ADDRESS); // exception 2 ILLEGAL DATA ADDRESS
                        }
                        break;
                    case 6: // FC06 - Write Single Holding Register
                        if (start_addr < mb.holdreg_size) // check exception 2 ILLEGAL DATA ADDRESS
                        {
                            mb.reg[start_addr] = ((mb.frame_buf[4] << 8) | mb.frame_buf[5]); // the 4th and 5th elements in frame_buf is the 16 bit data value
                            // ID, FC, (addrHi + addrLo), (dataHi + dataLo), (crcLo + crcHi)
                            frame_size = 8;
                            mb.framesize = frame_size;
                            // only the first 6 bytes are used for CRC calculation
                            crc = calCRC16(mb.frame_buf, frame_size - 2);
                            mb.frame_buf[6] = crc >> 8;
                            mb.frame_buf[7] = crc & 0xFF;
                            // FC16 response echo the first 6 bytes from the request + 2 CRC bytes
                            mb.state = MODBUS_TX_RESPONSE_BUSY;
                            mb.pass_count++;
                        }
                        else
                        {
                            sendError(ILLEGAL_DATA_ADDRESS); // exception 2 ILLEGAL DATA ADDRESS
                        }
                        break;
                    case 16: // FC16 - Write Multiple Holding Registers
                        // Check if the received number of bytes matches the calculated bytes minus the request bytes.
                        // ID + FC + (2 * address bytes) + (2 * no of register bytes) + byte count + (2 * CRC bytes) = 9 bytes
                        if (mb.frame_buf[6] == (mb.bufsize - 9))
                        {
                            if (start_addr < mb.holdreg_size) // check exception 2 ILLEGAL DATA ADDRESS
                            {
                                if (max_data <= mb.holdreg_size) // check exception 3 ILLEGAL DATA VALUE
                                {
                                    address = 7; // start at the 8th byte in the frame_buf
                                    for (index = start_addr; index < max_data; index++)
                                    {
                                        mb.reg[index] = ((mb.frame_buf[address] << 8) | mb.frame_buf[address + 1]);
                                        address += 2;
                                    }
                                    frame_size = 8;
                                    mb.framesize = frame_size;
                                    // only the first 6 bytes are used for CRC calculation
                                    crc = calCRC16(mb.frame_buf, frame_size - 2);
                                    mb.frame_buf[6] = crc >> 8;
                                    mb.frame_buf[7] = crc & 0xFF;
                                    // FC16 response echo the first 6 bytes from the request + 2 CRC bytes
                                    mb.state = MODBUS_TX_RESPONSE_BUSY;
                                    mb.pass_count++;
                                }
                                else
                                {
                                    sendError(ILLEGAL_DATA_VALUE); // exception 3 ILLEGAL DATA VALUE
                                }
                            }
                            else
                            {
                                sendError(ILLEGAL_DATA_ADDRESS); // exception 2 ILLEGAL DATA ADDRESS
                            }
                        }
                        else
                        {
                            sendError(PARAMETER_COUNT_ERROR); // exception e6 PACKET_LENGTH_ERROR
                        }
                        break;
                    default:
                        sendError(ILLEGAL_FUNCTION); // exception 1 ILLEGAL FUNCTION
                        break;
                    }//switch
                }
                else
                {
                    sendError(CRC_ERROR); // exception e2 CRC ERROR
                }
            } // slave id not match
            else
            {
                sendError(SERVER_ID_MISMATCH); // exception e4 SERVER ID MISMATCH
            }
        }
        else if (mb.bufsize > 0)
        {
            sendError(PACKET_LENGTH_ERROR); // exception e5 PACKET_LENGTH_ERROR
        }
        mb.bufsize = 0;
    }

    return mb.err_count;
}

// Look up table stored in Flash for CRC16
static const uint16_t crcHighBytes[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
                                         0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                                         0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
                                         0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
                                         0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
                                         0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
                                         0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
                                         0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
                                         0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
                                         0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

// Look up table stored in Flash for CRC16
static const uint16_t crcLowBytes[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA,
                                        0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15,
                                        0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35,
                                        0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
                                        0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
                                        0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
                                        0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7,
                                        0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
                                        0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
                                        0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

// Calculate CRC 16
uint16_t calCRC16(uint16_t *pBuf, uint16_t len)
{
    uint16_t crc_high = 0xFF;
    uint16_t crc_low = 0xFF;
    int index;

    while (len--)
    {
        index = crc_low ^ *(pBuf++);
        crc_low = (uint16_t) (crc_high ^ crcHighBytes[index]);
        crc_high = crcLowBytes[index];
    }
    // Reversed byte order.
    return (uint16_t) (crc_low << 8 | crc_high);
}

//t15 t35 calculation
//serial 8N1 = start bit + data byte + stop bit = 10bits
//baudrate = 115200
//frame/sec = 115200/10 = 11520 bytes/s
//1 frame = 1/11520 = 86.8us
//t35 = 3.5 chars = 3.5 frames = 3.5 * 86.8us = 304us
//t15 = 1.5 chars = 1.5 frames = 1.5 * 86.8us = 130us
//MAX3485 EN->driver delay max 1us
//to reduce CPU load, if baud rates >19200, t35 = 1750us, t15 = 750us

// Send response message to a good request
void sendResponse(uint16_t bufferSize)
{
    uint16_t i;

    for (i = 0; i < bufferSize; i++)
    {
        SCI_putDataBlocking(hal.sciAHandle, mb.frame_buf[i]);
        //hal.sciAHandle->SCITXBUF = mb.frame_buf[i++];
    }

    to.t35Count = Cputimer1IntCount;
    mb.state = MODBUS_TX_WAIT_T35;
}

// Send error message to a bad request
void sendError(uint16_t exception)
{
    uint16_t crc;

    // ID, FC + 0x80, exception code, 2 bytes CRC
    mb.frame_buf[0] = mb.id;
    mb.frame_buf[1] = (mb.fc | 0x80); // set MSB bit high, informs the master of an exception
    mb.frame_buf[2] = exception;
    crc = calCRC16(mb.frame_buf, 3); // ID, FC|0x80, exception code
    mb.frame_buf[3] = crc >> 8;
    mb.frame_buf[4] = crc & 0xFF;
    mb.framesize = 5; // exception response is always 5 bytes
    // each call to sendError() will increment the err_count
    mb.err_count++;
    mb.state = MODBUS_TX_ERROR_BUSY;
}

//===========================================================================
// No more.
//===========================================================================
