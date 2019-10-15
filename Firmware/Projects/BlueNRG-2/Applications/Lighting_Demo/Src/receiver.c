/*****************************************************************************
File Name:    receiver.c
Description:   
History:	
Date                Author                   Description
2019-10-11         Lucien                    Creat
****************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "double_list.h"
#include "receiver.h"
#include "packet_common.h"
#include "dcache.h"
#include "mesh_cfg_usr.h"
#include "mesh_cfg.h"


struct packet_t{
		uint16_t length;
		uint8_t	 *body;
};


static struct dl_list s_receive_cache;

static struct packet_t s_packet;
static RECEIVER_DONE_CALLBACK s_receiver_done_cb;


void receiver_init(RECEIVER_DONE_CALLBACK cb)
{
		s_receiver_done_cb = cb;
		dl_list_init(&s_receive_cache);
}

			
static void receiver_reinit(uint16_t packet_len, MOBLEBOOL clean_flag)
{
		if(clean_flag){
				/* clearn cache */
        struct dlist_cache *item;
        struct dlist_cache *next;
        dl_list_for_each_safe(item, next, &s_receive_cache, struct dlist_cache, node) {
            dl_list_del(&item->node);
            dlist_cache_free(item);
      	}

				/* apply dynamic memory */
				if(s_packet.body != NULL){
						free(s_packet.body);
						s_packet.body = NULL;
				}
				s_packet.length = packet_len;
				s_packet.body = (uint8_t*)malloc(packet_len);
				memset(s_packet.body,0,packet_len);
				if(s_packet.body == NULL){
						TRACE_I(TF_VENDOR,"Error, dynamic memory lack \r\n");
				}
				
		}

		dl_list_init(&s_receive_cache);		
}

static void receiver_frame_add_cache(struct dlist_cache *new)
{
		MOBLEBOOL flag = MOBLE_FALSE;

		struct dlist_cache *item;
		dl_list_for_each(item, &s_receive_cache, struct dlist_cache, node) {
        if (item->offset > new->offset) {
            flag = MOBLE_TRUE;
            dl_list_add(item->node.prev, &new->node);
						//break;
        }
    }

		if (!flag) {
        dl_list_add_tail(&s_receive_cache, &new->node);
    } 
		
		/* ºÏ²¢ */
    item = dl_list_prev(&s_receive_cache, new, struct dlist_cache, node);
    if (item && item->offset + item->length  == new->offset) {
        dl_list_del(&item->node);
        new->offset  = item->offset;
        new->length += item->length;
        dlist_cache_free(item);
				//print_list(&s_receive_cache);
    }
    
    item = dl_list_next(&s_receive_cache, new, struct dlist_cache, node);
    if (item && new->offset + new->length == item->offset) {
				
        dl_list_del(&item->node);
        new->length  += item->length;
        dlist_cache_free(item);
    }
}

MOBLEBOOL receiver_update(MOBLE_ADDRESS peer_addr, 
                                     MOBLE_ADDRESS dst_peer, 
                                     MOBLEUINT8 command, 
                                     MOBLEUINT8 const *data, 
                                     MOBLEUINT32 length, 
                                     MOBLEBOOL response)
{
		MOBLEBOOL ret = MOBLE_FALSE;
		if((length > FRAME_SIZE) || (data == NULL))
			return ret;

		printf("%s \n",__func__);
		/* get the first byte*/
		uint8_t seq = data[0]>>3;
		uint8_t len = data[0] & 0x07;

		printf("seq: %d  len:%d \n",seq,len);
		/* check the length */
		if((length<2) || ((len+1) != length))
			return ret;

		uint16_t fill_offset = seq*(FRAME_SIZE-1);
		uint16_t fill_len = len;
		uint16_t data_offset = 1;
		if(seq == 0){
				uint16_t frame_nums = data[1]/FRAME_SIZE;
				uint16_t packet_len = (frame_nums)*(FRAME_SIZE-1);
				packet_len += (data[1] % FRAME_SIZE) -2;
				receiver_reinit(packet_len, MOBLE_TRUE);
				fill_len -= 1;
				data_offset += 1;
		}
		else{
				fill_offset -= 1;
		}

		printf("dlist_cache_length:%d\n",dl_list_len(&s_receive_cache));
		/* check if It is old packet */
    struct dlist_cache *item;
    dl_list_for_each(item, &s_receive_cache, struct dlist_cache, node) {
        uint32_t end = item->offset + item->length;
				printf("fill_offset:%d  \n",fill_offset);
				printf("item->offset:%d  end:%d \n",item->offset, end);
        if (fill_offset >= item->offset && fill_offset < end) {
            if (fill_offset + fill_len <= end) {
								ret = MOBLE_TRUE;
                return ret;
            } else {
								ret = MOBLE_FALSE;
                return ret;
            }
        }
    }


		struct dlist_cache *new = dlist_cache_new();
		
		// if It have not enough cache  return
		if (NULL == new) {
				ret = MOBLE_FALSE;
				TRACE_I(TF_VENDOR,"have not enough cache \n");
				return ret;
		}
		
		new->length = fill_len;
		new->offset = fill_offset;
		receiver_frame_add_cache(new);
		
		memcpy(&s_packet.body[fill_offset],&data[data_offset],fill_len);

		 /* completed send, nofity application  */
    if (dl_list_len(&s_receive_cache) == 1) {
				ret = MOBLE_TRUE;
				printf("new->length:%d \n", new->length);
        if (new->offset == 0 && new->length >= s_packet.length) {  
            if(s_receiver_done_cb != NULL){
								s_receiver_done_cb(command, s_packet.body, s_packet.length);
								if(s_packet.body != NULL){
										free(s_packet.body);
										s_packet.body = NULL;
								}
						}
        }
    }
		return ret;

}


















