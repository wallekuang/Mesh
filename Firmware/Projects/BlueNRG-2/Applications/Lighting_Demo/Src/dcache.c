/*****************************************************************************
File Name:    dcache.c
Description:   
History:	
Date                Author                   Description
2019-10-11         Lucien                    Creat
****************************************************************************/
#include "dcache.h"

static struct dlist_cache __cache_memory[24];

static struct dl_list s_cache_list;



void dlist_cache_init(void)
{
    dl_list_init(&s_cache_list);

    for (int i = 0; i < ARRAY_LEN(__cache_memory); i++) {
        dl_list_add(&s_cache_list, &__cache_memory[i].node);
    }
}

struct dlist_cache *dlist_cache_new(void)
{
    struct dlist_cache *item = dl_list_first(&s_cache_list, struct dlist_cache, node);
    if (item) {
        dl_list_del(&item->node);
    }
    
    return item;
}

void dlist_cache_free(struct dlist_cache *cache)
{
    dl_list_add(&s_cache_list, &cache->node);
}

uint32_t dlist_cache_length(void)
{
		return dl_list_len(&s_cache_list);
}



