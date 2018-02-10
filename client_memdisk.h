/*
	PULUROBOT RN1-CLIENT  Stand-alone GUI client prototype

	(c) 2017-2018 Pulu Robotics and other contributors
	Maintainer: Antti Alhonen <antti.alhonen@iki.fi>

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License version 2, as 
	published by the Free Software Foundation.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	GNU General Public License version 2 is supplied in file LICENSING.


*/

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
