/*****************************************************************************
File Name:    sender.h
Description:   
History:	
Date                Author                   Description
2019-10-11         Lucien                    Creat
****************************************************************************/
#ifndef __SENDER_H__
#define __SENDER_H__

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "dcache.h"
#include "types.h"

#include "double_list.h"



typedef void *(*PUBLISH_DONE_CALLBACK)(MOBLE_RESULT sta,void* param);


typedef void (*PUBLISH_WAY_CALLBACK)(uint8_t *pdata, uint8_t length, MOBLEBOOL response);

void sender_init(PUBLISH_WAY_CALLBACK cb);


MOBLE_RESULT sender_packet_list_send(uint8_t *pdata, uint16_t len, PUBLISH_DONE_CALLBACK cb, void *param);


MOBLE_RESULT sender_easy_publish(uint8_t *pdata, uint16_t len);



void sender_update_OnRespon_frame(MOBLEUINT8 command, 
                                     MOBLEUINT8 const *pRxData, 
                                     MOBLEUINT32 dataLength, 
                                     MOBLEBOOL response);
void sender_tick(void);





#endif

