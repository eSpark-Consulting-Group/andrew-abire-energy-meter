#include "main.h"
#include "userio.h"
#include "modbus_user.h"

GPIO_Handle myGpio;
extern MODBUS_MSG_T mbmsg;

void InitGpios()
{
    //
    // Init DI1 as an input
    //
    GPIO_setPullUp(myGpio, DI1_PIN, GPIO_PullUp_Enable);
    GPIO_setMode(myGpio, DI1_PIN, DI1_MODE);
    GPIO_setDirection(myGpio, DI1_PIN, GPIO_Direction_Input);
    //
    // Init DI2 as an input
    //
    GPIO_setPullUp(myGpio, DI2_PIN, GPIO_PullUp_Enable);
    GPIO_setMode(myGpio, DI2_PIN, DI2_MODE);
    GPIO_setDirection(myGpio, DI2_PIN, GPIO_Direction_Input);
    //
    // Init DI3 as an input
    //
    GPIO_setPullUp(myGpio, DI3_PIN, GPIO_PullUp_Enable);
    GPIO_setMode(myGpio, DI3_PIN, DI3_MODE);
    GPIO_setDirection(myGpio, DI3_PIN, GPIO_Direction_Input);
    //
    // Init DI4 as an input
    //
    GPIO_setPullUp(myGpio, DI4_PIN, GPIO_PullUp_Enable);
    GPIO_setMode(myGpio, DI4_PIN, DI4_MODE);
    GPIO_setDirection(myGpio, DI4_PIN, GPIO_Direction_Input);
    //
    //Init DO1 as an output and set the optocoupler output to High Z
    //
    myGpio = GPIO_init((void*) GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    GPIO_setPullUp(myGpio, DO1_PIN, GPIO_PullUp_Disable);
    GPIO_setHigh(myGpio, DO1_PIN); //GPIO High = Opto High Z
    GPIO_setMode(myGpio, DO1_PIN, DO1_MODE);
    GPIO_setDirection(myGpio, DO1_PIN, GPIO_Direction_Output);
    //
    //Init DO2 as an output and set the optocoupler output to High Z
    //
    myGpio = GPIO_init((void*) GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    GPIO_setPullUp(myGpio, DO2_PIN, GPIO_PullUp_Disable);
    GPIO_setHigh(myGpio, DO2_PIN); //GPIO High = Opto High Z
    GPIO_setMode(myGpio, DO2_PIN, DO2_MODE);
    GPIO_setDirection(myGpio, DO2_PIN, GPIO_Direction_Output);
    //
    //Init DO3 as an output and set the optocoupler output to High Z
    //
    myGpio = GPIO_init((void*) GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    GPIO_setPullUp(myGpio, DO3_PIN, GPIO_PullUp_Disable);
    GPIO_setHigh(myGpio, DO3_PIN); //GPIO High = Opto High Z
    GPIO_setMode(myGpio, DO3_PIN, DO3_MODE);
    GPIO_setDirection(myGpio, DO3_PIN, GPIO_Direction_Output);
    //
    //Init DO3 as an output and set the optocoupler output to High Z
    //
    myGpio = GPIO_init((void*) GPIO_BASE_ADDR, sizeof(GPIO_Obj));
    GPIO_setPullUp(myGpio, DO4_PIN, GPIO_PullUp_Disable);
    GPIO_setHigh(myGpio, DO4_PIN); //GPIO High = Opto High Z
    GPIO_setMode(myGpio, DO4_PIN, DO4_MODE);
    GPIO_setDirection(myGpio, DO4_PIN, GPIO_Direction_Output);
}

void readIOstates()
{
    //DI1
    if (GPIO_read(myGpio, DI1_PIN))
    {
        mbmsg.io.bit.DI1 = false;
        stat.bit.onoff = false;
    }
    else
    {
        mbmsg.io.bit.DI1 = true;
        stat.bit.onoff = true;
    }
    //DI2
    if (GPIO_read(myGpio, DI2_PIN))
    {
        mbmsg.io.bit.DI2 = false;
        stat.bit.dir = DIR_CCW;
    }
    else
    {
        mbmsg.io.bit.DI2 = true;
        stat.bit.dir = DIR_CW;
    }
    //DI3
    if (GPIO_read(myGpio, DI3_PIN))
    {
        mbmsg.io.bit.DI3 = false;
    }
    else
    {
        mbmsg.io.bit.DI3 = true;
    }
    //DI4
    if (GPIO_read(myGpio, DI4_PIN))
    {
        mbmsg.io.bit.DI4 = false;
    }
    else
    {
        mbmsg.io.bit.DI4 = true;
    }
}

void setDigitalOutput(void)
{
    mbmsg.io.bit.DO1 ? DO1_OPEN() : DO1_CLOSE();
    mbmsg.io.bit.DO2 ? DO2_OPEN() : DO2_CLOSE();
    mbmsg.io.bit.DO3 ? DO3_OPEN() : DO3_CLOSE();
    mbmsg.io.bit.DO4 ? DO4_OPEN() : DO4_CLOSE();
}

//===========================================================================
// No more.
//===========================================================================

