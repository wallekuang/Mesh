/*****************************************************************************
File Name:    dcache.h
Description:   
History:	
Date                Author                   Description
2019-10-11         Lucien                    Creat
****************************************************************************/
#ifndef __DCACHE_H__
#define __DCACHE_H__

#include <stdint.h>
#include <stdlib.h>

#include "double_list.h"


struct dlist_cache {
    struct dl_list node;
    uint16_t offset;
    uint16_t length;
};


void dlist_cache_init(void);
struct dlist_cache *dlist_cache_new(void);
void dlist_cache_free(struct dlist_cache *cache);

uint32_t dlist_cache_length(void);


#endif

