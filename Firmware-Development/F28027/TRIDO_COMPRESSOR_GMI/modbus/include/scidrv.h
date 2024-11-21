#ifndef _SCIDRV_H_
#define _SCIDRV_H_

#include "../../include/hal_obj.h" // System Include

#define SCI_RX_BUFFER_SIZE 64

void scia_setup(void);
void scia_xmit(uint16_t dat);
void scia_fifo_init(void);
void scia_init(void);

#endif /* _SCIDRV_H_ */
