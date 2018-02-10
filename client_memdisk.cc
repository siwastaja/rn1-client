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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <dirent.h>

#include "../rn1-host/mapping.h"
#include "client_memdisk.h"

extern uint32_t robot_id;

int read_map_page(world_t* w, int pagex, int pagey)
{
	char fname[1024];
	sprintf(fname, "%08x_%u_%u_%u.map", robot_id, w->id, pagex, pagey);

	printf("Info: Attempting to read map page %s\n", fname);

	FILE *f = fopen(fname, "r");
	if(!f)
	{
		if(errno == ENOENT)
			return 2;
		fprintf(stderr, "Error %d opening %s for read\n", errno, fname);
		return 1;
	}

	int ret;
	if( (ret = fread(w->pages[pagex][pagey], sizeof(map_page_t), 1, f)) != 1)
	{
		printf("Error: Reading map data failed, fread returned %d. feof=%d, ferror=%d\n", ret, feof(f), ferror(f));
	}

	fclose(f);
	return 0;
}

int load_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
		printf("Info: reloading already allocated map page %d,%d\n", pagex, pagey);
	}
	else
	{
		printf("Info: Allocating mem for page %d,%d\n", pagex, pagey);
		w->pages[pagex][pagey] = (map_page_t*)malloc(sizeof(map_page_t));
	}

	int ret = read_map_page(w, pagex, pagey);
	if(ret)
	{
		printf("Error: Reading map file failed. Initializing empty map\n");
		memset(w->pages[pagex][pagey], 0, sizeof(map_page_t));
		return 1;
	}
	return 0;
}

int unload_map_page(world_t* w, int pagex, int pagey)
{
	if(w->pages[pagex][pagey])
	{
		printf("Info: Freeing mem for page %d,%d\n", pagex, pagey);
		free(w->pages[pagex][pagey]);
		w->pages[pagex][pagey] = 0;
	}
	else
	{
		printf("Warn: Trying to unload a map page which is already free.\n");
	}
	return 0;
}

int unload_map_pages(world_t* w, int cur_pagex, int cur_pagey)
{
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(w->pages[x][y] && (abs(cur_pagex - x) > 2 || abs(cur_pagey - y) > 2))
			{
				unload_map_page(w, x, y);
			}

		}
	}
	return 0;
}

void load_all_pages_on_disk(world_t* w)
{
	DIR *d;
	struct dirent *dir;
	d = opendir(".");

	if(d)
	{
		while( (dir = readdir(d)) != 0)
		{
			uint32_t f_robot_id = 0;
			int f_world_id = 0, f_pagex = 0, f_pagey = 0;

			if(sscanf(dir->d_name, "%08x_%u_%u_%u.map", &f_robot_id, &f_world_id, &f_pagex, &f_pagey) == 4)
			{
				if(f_robot_id == robot_id && f_world_id == w->id)
				{
					load_map_page(w, f_pagex, f_pagey);
				}
			}

		}

		closedir(d);
	}
}

