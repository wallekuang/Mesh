/*****************************************************************************
File Name:    app_control.c
Description:   
History:	
Date                Author                   Description
2019-10-12         Lucien                    Creat
****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "app_control.h"
#include "receiver.h"
#include "types.h"
#include "mesh_cfg_usr.h"
#include "mesh_cfg.h"
#include "sender.h"
#include "vendor.h"
#include "common.h"

#define PACKET_SIMMULATION     0

static void app_receiver_done_even(MOBLEUINT8 command, 
																MOBLEUINT8 const *data, MOBLEUINT32 length)
{
		TRACE_I(TF_VENDOR,"%s \r\n", __func__);
		for(int i=0;i<length;i++){
				TRACE_I(TF_VENDOR,"%x ", data[i]);
		}
		TRACE_I(TF_VENDOR,"\n");
}



void app_control_tick(void)
{
		sender_tick();
}


static void *send_done_even(MOBLE_RESULT sta,void* param) 
{
		TRACE_I(TF_VENDOR,"sta:%d %s \r\n", sta, __func__);

		static int count = 100;
		count--;
		if(count > 0){
				//app_control_test();
		}
		
		return NULL;
}

void app_control_test(void)
{
		uint8_t temp[200];
		for(int i=0;i<sizeof(temp);i++)
			temp[i] = i;
		sender_packet_list_send(temp,sizeof(temp),send_done_even,NULL);
}


#if !(PACKET_SIMMULATION)
static void app_publish_even(uint8_t *pdata, uint8_t length, MOBLEBOOL response)
{
		MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
		MOBLE_ADDRESS srcAdd = BluenrgMesh_GetAddress();
		uint8_t elementNumber = Bluenrg_GetElementNumber();
		uint16_t publishAddress = BluenrgMesh_GetPublishAddress(elementNumber, VENDORMODEL_STMICRO_ID1);
		TRACE_I(TF_VENDOR,"srcAddress:%x publishAddress:%x \n", srcAdd, publishAddress);

		result = BluenrgMesh_SetRemotePublication(VENDORMODEL_STMICRO_ID1, srcAdd,
																							APPLI_THROUGHPUT_CMD, 
																							pdata, length,
																							response, MOBLE_TRUE);

		 if(result){
			 TRACE_I(TF_VENDOR,"Publication Error \r\n");
 }																
}
#endif

#if PACKET_SIMMULATION
static uint8_t mem_temp[8];
static void app_publish_simulation_even(uint8_t *pdata, uint8_t length, MOBLEBOOL response)
{

#if 1
		TRACE_I(TF_VENDOR,"%s \n", __func__);
		for(int i=0;i<length;i++)
		{
				TRACE_I(TF_VENDOR, " %x ", pdata[i]);
		}
#endif
		

		memcpy(mem_temp,pdata,length);


		MOBLEBOOL ret = receiver_update(0x00,0x00,0x00,pdata,length,response);
		
		if(response && ret){
			static int count = 0;
			count++;
			if(count %5 == 0)
				return;
			sender_update_OnRespon_frame(APPLI_THROUGHPUT_CMD, mem_temp, length, response);
		}
}
#endif


void app_control_init(void)
{
		dlist_cache_init();
		receiver_init(app_receiver_done_even);
		
#if PACKET_SIMMULATION
		sender_init(app_publish_simulation_even);
		app_control_test();
		app_control_test();
		app_control_test();
#else
		sender_init(app_publish_even);
#endif
}



