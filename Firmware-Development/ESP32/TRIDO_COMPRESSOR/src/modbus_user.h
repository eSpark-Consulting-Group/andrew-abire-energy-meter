#ifndef MODBUS_USER_H
#define MODBUS_USER_H

// Here 
// MO means output from master
// SO means output from slave
// MI means input to master
// SI means input to slave
enum
{ 
  FC03_SOMI               = (uint32_t)0,
  FC16_MOSI               = (uint32_t)1
};

#endif
