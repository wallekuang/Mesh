

#ifndef __RECEIVER_H__
#define __RECEIVER_H__



#include "types.h"


typedef void (*RECEIVER_DONE_CALLBACK)(MOBLEUINT8 command, 
																MOBLEUINT8 const *data, MOBLEUINT32 length);



void receiver_init(RECEIVER_DONE_CALLBACK cb);


MOBLEBOOL receiver_update(MOBLE_ADDRESS peer_addr, 
                                     MOBLE_ADDRESS dst_peer, 
                                     MOBLEUINT8 command, 
                                     MOBLEUINT8 const *data, 
                                     MOBLEUINT32 length, 
                                     MOBLEBOOL response);
















#endif
