/*****************************************************************************
File Name:    packet_common.h
Description:   
History:	
Date                Author                   Description
2019-10-12         Lucien                    Creat
****************************************************************************/
#ifndef PACKET_COMMON_H__
#define PACKET_COMMON_H__


#define FRAME_WINDOW						(7)
#define FRAME_SIZE							(8)


#if (PACKET_NUM_BYTES==1)
		#define SENDER_MAX_PACKET_LEN  (220)
#else
		#define SENDER_MAX_PACKET_LEN  (4096)
#endif



/* 数据包字最大数量 所占字节数  如果为1                 最大可以发送0xFF ，为2 最大可以发送0xFFFF                不支持其他值*/
#define PACKET_NUM_BYTES				(1)







#endif




