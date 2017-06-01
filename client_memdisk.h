#ifndef CLIENT_MEMDISK_H
#define CLIENT_MEMDISK_H

#include <stdint.h>
#include "../rn1-host/mapping.h"

// Disk access; file name is generated and the page is stored/read.
int read_map_page(world_t* w, int pagex, int pagey);

// Allocates memory for a page and reads page from disk; if it doesn't exist, the new page is zeroed out
int load_map_page(world_t* w, int pagex, int pagey);

// Writes the map page to disk and frees the memory, setting the page pointer to 0.
int unload_map_page(world_t* w, int pagex, int pagey);

void load_all_pages_on_disk(world_t* w);


#endif
