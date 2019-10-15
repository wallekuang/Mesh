/*****************************************************************************
File Name:    sender.c
Description:   
History:	
Date                Author                   Description
2019-10-11         Lucien                    Creat
****************************************************************************/
#include "sender.h"
#include "bluenrg_mesh.h"
#include "mesh_cfg_usr.h"
#include "mesh_cfg.h"
#include "bluenrg1_stack.h"
#include "packet_common.h"

#define RESEND_TIMEOUT					(1000)										// 超时重发时间
#define TIME_OUT_COUNT						(10)							      // 超时重发次数，当超时次数超过时，发送失败


enum{
		SENDER_NOMAL,
		SENDER_NEED_TO_RESEND,
};


struct packet_send_t{
		uint16_t length;
		uint8_t	 *body;
		PUBLISH_DONE_CALLBACK cb;
		void *param;
};


struct fifo_sender{
		struct dl_list node;
			
		uint32_t start_timestamp;		/* 用户记录发送起始时间戳 */
		uint16_t frame_nums;				/* 用户计算整个数据包的需要的总帧数 */
		struct packet_send_t ins;		/* 发送应用层相关的数据 记录需要发送的数据和发送完的回调等 */
};


/* 一次可以同时缓存这么多条应用数据包  */ 
static struct fifo_sender __packet_memory[3];
/* 应用数据包已使用列表 */
static struct dl_list s_packet_list;
/* 应用数据包未使用列表 */
static struct dl_list s_packet_unuse;

/* 已发送的数据帧            
应答方式:默认是按照每次间隔FRAME_WINDOW 帧需要应答一次 
首尾帧需要应答 应答数据不OK 则需要重发当前window的所有帧 */
/* window的起始点       -1: no start yet */ 
static int s_window_slide = -1; 

static PUBLISH_WAY_CALLBACK s_publish_way_cb = NULL;

void sender_init(PUBLISH_WAY_CALLBACK cb)
{
		dl_list_init(&s_packet_list);
		dl_list_init(&s_packet_unuse);

		for (int i = 0; i < ARRAY_LEN(__packet_memory); ++i) {
        dl_list_add(&s_packet_unuse, &__packet_memory[i].node);
    }
		
		s_publish_way_cb = cb;
		//dl_list_init(&s_checked_cache);
}

static struct fifo_sender * sender_get_last_sender(void)
{
		struct fifo_sender *item = NULL;
		if (!dl_list_empty(&s_packet_list)) {
    		item = dl_list_last(&s_packet_list, struct fifo_sender, node);
    }
		return item;
}

static void sender_safe_clean_current_current_packet(void)
{
		struct fifo_sender *item = sender_get_last_sender();
		if(item != NULL){
				if(item->ins.body != NULL){
						free(item->ins.body);
						memset(&item->ins,0,sizeof(struct packet_send_t));
				}

				dl_list_del(&item->node);
				dl_list_add_tail(&s_packet_unuse, &item->node);
		}
		s_window_slide = -1;
}

/*  for application call to send data
@input
		-pdata: application data need to send
		-len:		application data length need to send
		-cb:		callback of application want to when finish send. if application don't want to callback, just set cb to NULL
		-param: param for when callback cb if application don't want to callback, just set param to NULL

*/
MOBLE_RESULT sender_packet_list_send(uint8_t *pdata, uint16_t len, PUBLISH_DONE_CALLBACK cb, void *param)
{
		/* count the number need to send   + FRAME num * 1byte */
		uint16_t frame_nums = ((len + PACKET_NUM_BYTES)/(FRAME_SIZE-1));
		if(!(((len + PACKET_NUM_BYTES)%(FRAME_SIZE-1)) == 0))
				frame_nums += 1;
		uint16_t send_len = frame_nums*PACKET_NUM_BYTES + (len+1);
		/* check  param of input */
		if( (pdata == NULL) || (len == 0) || (len) >= SENDER_MAX_PACKET_LEN){
				return MOBLE_RESULT_INVALIDARG;
		}
		
		/* check the unuse list if is avalible for using */
		struct fifo_sender *item = NULL;
		if (!dl_list_empty(&s_packet_unuse)) {
    		item = dl_list_last(&s_packet_unuse, struct fifo_sender, node);
    }
		else{
				return MOBLE_RESULT_FAIL;
		}

		if(item == NULL){
				return MOBLE_RESULT_FAIL;
		}
				
		/* check if have enough memmory to send  */
		uint8_t *p = (uint8_t*)malloc(send_len);
		if(p == NULL){
				return MOBLE_RESULT_OUTOFMEMORY;
		}
	
		// remove node from unuse list
		dl_list_del(&item->node);
		item->ins.body = p;
		uint16_t pos = 0;	// 帧偏移
		uint16_t user_pos = 0; 
		for(int seq=0;seq<frame_nums;seq++)
		{
				/* 第一帧有用位置比其他帧少一个字节                 因为第一帧的第二个字节用户存放 总数据包长度 */
				/* 所有帧的第一字节用户存放pos和len */
				uint8_t available_len = (seq != 0)? (FRAME_SIZE -1):(FRAME_SIZE -2);
				uint8_t frame_len = ((len - user_pos) > available_len) ? available_len:(len-user_pos);
				// fill frame seq and len [seq:len]<---->[5:3]
				item->ins.body[pos] = (seq << 3) + ((seq == 0)? (frame_len+1) : frame_len);
				pos += 1;
				if(seq == 0){
						item->ins.body[pos] = send_len;
						pos += 1;
				}
				// fill frame data
				memcpy(&item->ins.body[pos], pdata+user_pos, frame_len);
				user_pos += frame_len;
				pos += frame_len;
		}
		
		//
		item->ins.length = send_len;
		item->ins.cb = cb;
		item->ins.param = param;
		item->frame_nums = frame_nums;
		// add  node to tail
		dl_list_add_tail(&s_packet_list, &item->node);
		return MOBLE_RESULT_SUCCESS;
}


MOBLE_RESULT sender_easy_publish(uint8_t *pdata, uint16_t len)
{
		return sender_packet_list_send(pdata,len,NULL,NULL);
}

/*
		publish frames:  for frame_num  will be get the respon form peer device in time, if no that will be timeout

*/
static void sender_publish_frames(int window_start, struct fifo_sender *item, uint16_t frame_num)
{
		if(frame_num == 0 || window_start < 0)
			return;

		struct packet_send_t *pins = &item->ins;
		uint8_t stemp[FRAME_SIZE];
		uint8_t slen;
		MOBLEBOOL response = MOBLE_FALSE;
		
		/* record the timestamp of each first frame send*/

		item->start_timestamp = HAL_VTimerGetCurrentTime_sysT32();
		//printf("item->start_timestamp: %d \n",item->start_timestamp);

		uint16_t pos = 0;
		for(int i=0;i<frame_num;i++){
			pos = (window_start+i)*FRAME_SIZE;
			slen = ((pins->length-pos) > FRAME_SIZE) ? FRAME_SIZE : (pins->length-pos);
			memcpy(stemp, &(pins->body[pos]), slen);
		  /* if it is the last frame, ask to respense */
			if( (i+1) == frame_num)
				response = MOBLE_TRUE;

			if(s_publish_way_cb != NULL){
				 s_publish_way_cb(stemp,slen,response);
			}
		}
}


/*  */
static uint8_t sender_packet_check_sender_state(int window_start, struct fifo_sender *item, uint16_t frame_num)
{
		uint32_t cur_timestamp = HAL_VTimerGetCurrentTime_sysT32();
		uint32_t elapsed_ms = (uint32_t)(abs(cur_timestamp-item->start_timestamp) * 2.4414)/1000;
		//printf("elapsed_ms:%d \n",elapsed_ms);
		
		if(elapsed_ms > RESEND_TIMEOUT)
				return SENDER_NEED_TO_RESEND;
		else
				return SENDER_NOMAL;
}

static uint16_t sender_packet_count_frame_num(struct fifo_sender *item)
{
		uint16_t frame_num = 0;
		if(s_window_slide < 0){
				frame_num = 1;
		}
		else if(s_window_slide == 0){
				frame_num = 1;
		}else{
				uint16_t left = (item->frame_nums-s_window_slide);
				frame_num = (left > FRAME_WINDOW) ? FRAME_WINDOW : left;
		}

		return frame_num;
}

static void sender_packet_schedule(struct fifo_sender *item, int window_slide)
{
		if( (item == NULL) || (item->ins.length == 0) || (item->ins.body == NULL) )
				return;

		uint8_t need_to_send = MOBLE_FALSE;
		static uint8_t resend_count = 0;
		// start slide
		if(window_slide < 0){
				need_to_send = MOBLE_TRUE;
				s_window_slide = 0;
				resend_count = 0;
		}
		else{
				if(s_window_slide != window_slide){
						need_to_send = MOBLE_TRUE;
						s_window_slide = window_slide;
						resend_count = 0;
				}
		}

		uint16_t frame_num = sender_packet_count_frame_num(item);
		if(!need_to_send){
				/* check sender state */
				if(sender_packet_check_sender_state(s_window_slide, item,frame_num) == SENDER_NEED_TO_RESEND){
						resend_count++;
						if(resend_count > TIME_OUT_COUNT){
								if(item->ins.cb != NULL){
										item->ins.cb(MOBLE_RESULT_FAIL,item->ins.param);
								}
								sender_safe_clean_current_current_packet();
								resend_count = 0;
						}
						else{
								need_to_send = MOBLE_TRUE;
						}
				}
		}

		if(need_to_send)
				sender_publish_frames(s_window_slide, item, frame_num);
}


static void sender_window_slide(MOBLEUINT8 const *pRxData, MOBLEUINT32 dataLength)
{
		struct fifo_sender * item = sender_get_last_sender();
		if(item == NULL || s_window_slide<0)
			return;

		// 最后一帧数据为应答数据 检查应答数据是否相等
		uint16_t frame_num = sender_packet_count_frame_num(item);
		if(frame_num == 0)
			return;
		uint16_t pos = (s_window_slide+frame_num-1)*FRAME_SIZE;
		// 检查长度
		uint32_t slen = ((item->ins.length-pos) > FRAME_SIZE) ? FRAME_SIZE : (item->ins.length-pos);
		if(slen != dataLength)
			return;

		if(memcmp(pRxData,&item->ins.body[pos],slen) != 0)
			return;
			
	 /* completed send, nofity application  */
		uint16_t checked_len = pos + dataLength;
    if (checked_len  >=  (item->ins.length) ) {  
        if(item->ins.cb != NULL){
						item->ins.cb(MOBLE_RESULT_SUCCESS, item->ins.param);
						sender_safe_clean_current_current_packet();
				}
				return;
    }

		sender_packet_schedule(item, s_window_slide + frame_num);
		
}

void sender_update_OnRespon_frame(MOBLEUINT8 command, 
                                     MOBLEUINT8 const *pRxData, 
                                     MOBLEUINT32 dataLength, 
                                     MOBLEBOOL response)
{
		if((dataLength > FRAME_SIZE) || (pRxData == NULL))
			return;
		
		/* get the first byte*/
//		uint8_t seq = (pRxData[0]>>3);
		uint8_t len = pRxData[0] & 0x07;

		/* check the length */
		if((dataLength<2) || ((len+1) != dataLength))
			return;
		
		sender_window_slide(pRxData,dataLength);
}


void sender_tick(void)
{
		/* check the packet list need to send */
		struct fifo_sender *item = sender_get_last_sender();
		if(item != NULL){
				sender_packet_schedule(item, s_window_slide);
		}

}












