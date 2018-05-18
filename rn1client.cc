//#define TOF_DEV

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



	This program takes direct TCP connection to the robot:
	./rn1client robot_hostname robot_port

	Needs code quality improvement. I haven't been able to decide whether this is 
	a prototype-to-be-replaced, or a maintained application. It works nevertheless :-).

	Library dependencies:
	SFML (at least 2.4.2 works; SFML tends to have slight compatibility breaks every now and then)

*/


#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <errno.h>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>

#include "../rn1-host/mapping.h"
#include "../rn1-host/datatypes.h"
#include "../rn1-brain/comm.h"
#include "client_memdisk.h"
#include "uthash.h"
#include "utlist.h"
#include "sfml_gui.h"

#define I16FROMBUF(b_, s_)  ( ((uint16_t)b_[(s_)+0]<<8) | ((uint16_t)b_[(s_)+1]<<0) )
#define I32FROMBUF(b_, s_)  ( ((uint32_t)b_[(s_)+0]<<24) | ((uint32_t)b_[(s_)+1]<<16) | ((uint32_t)b_[(s_)+2]<<8) | ((uint32_t)b_[(s_)+3]<<0) )
#define I32TOBUF(i_, b_, s_) {(b_)[(s_)] = ((i_)>>24)&0xff; (b_)[(s_)+1] = ((i_)>>16)&0xff; (b_)[(s_)+2] = ((i_)>>8)&0xff; (b_)[(s_)+3] = ((i_)>>0)&0xff; }
#define I16TOBUF(i_, b_, s_) {(b_)[(s_)] = ((i_)>>8)&0xff; (b_)[(s_)+1] = ((i_)>>0)&0xff; }


world_t world;


int pict_id, pict_bpp, pict_xs, pict_ys, dbg_boost;
uint8_t pict_data[1000000];

char status_text[2000];

uint32_t robot_id = 0xacdcabba;
int cur_speed_limit = 45;

sf::Font arial;

int screen_x = 1200;
int screen_y = 700;

double click_x, click_y;

double origin_x = 0;
double origin_y = 0;

double cur_angle = 0.0;
double cur_x = 0.0;
double cur_y = 0.0;

int show_dbgpoint, dbgpoint_x, dbgpoint_y, dbgpoint_r, dbgpoint_g, dbgpoint_b;

int num_pers_dbgpoints;
int pers_dbgpoint_x[100], pers_dbgpoint_y[100], pers_dbgpoint_r[100], pers_dbgpoint_g[100], pers_dbgpoint_b[100];


double route_start_x, route_start_y;

typedef enum {MODE_INVALID = -1, MODE_ROUTE = 0, MODE_MANUAL_FWD, MODE_MANUAL_BACK, MODE_FORCE_FWD, MODE_FORCE_BACK, MODE_POSE, MODE_ADDCONSTRAINT, MODE_REMCONSTRAINT} click_mode_t;
click_mode_t click_mode;

double dest_x, dest_y;
click_mode_t dest_type = MODE_INVALID;

double robot_xs = 480.0;
double robot_ys = 524.0;
double lidar_xoffs = 120.0;
double lidar_yoffs = 0.0;

int charging;
int charge_finished;
float bat_voltage;
float cha_voltage;
int bat_percentage;

int cur_cmd_status;
void print_cur_cmd_status(sf::RenderWindow& win)
{
	char tbu[1000];
	sf::Text t;
	t.setFont(arial);
	switch(cur_cmd_status)
	{
		case 55: sprintf(tbu, "Working on: Direct (manual) move"); break;
		case 56: sprintf(tbu, "Working on: Routefinding"); break;
		case 57: sprintf(tbu, "Working on: Finding the charger"); break;
		default: sprintf(tbu, "State bar"); break;
	}

	t.setString(tbu);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(200,255,200,200));
	t.setPosition(10, screen_y-28);
	win.draw(t);
	t.setFillColor(sf::Color(0,100,0,255));
	t.setPosition(8, screen_y-30);
	win.draw(t);
}

const char* click_mode_names[8] =
{
	"click to find route",
	"click to go directly (forward drive)",
	"click to go directly (REVERSE drive)",
	"click to force the robot (forward drive)",
	"click to force the robot (REVERSE drive)",
	"click to rotate the pose",
	"click to add a forbidden place",
	"click to remove a forbidden place"
};

const sf::Color click_mode_colors[8] = {
sf::Color(110, 255, 110, 190),
sf::Color(235, 235, 110, 190),
sf::Color(235, 235, 110, 190),
sf::Color(255, 110, 110, 190),
sf::Color(255, 110, 110, 190),
sf::Color(110, 200, 200, 190),
sf::Color(255, 110, 190, 190),
sf::Color(255, 110, 190, 190)
};

static int rsync_running = 0;

static char *rsync_argv[4];

void init_rsync_argv()
{
	rsync_argv[0] = (char*)malloc(100);
	strcpy(rsync_argv[0], "/bin/bash");

	rsync_argv[1] = (char*)malloc(100);
	strcpy(rsync_argv[1], "map_sync.sh");

	rsync_argv[2] = (char*)malloc(1024);

	rsync_argv[3] = NULL;
}

void deinit_rsync_argv()
{
	free(rsync_argv[0]);
	free(rsync_argv[1]);
	free(rsync_argv[2]);
}

pid_t my_pid;
static void run_map_rsync()
{
	if(rsync_running)
	{
		printf("rsync still running\n");
		return;
	}

	if((my_pid = fork()) == 0)
	{
		if((execve(rsync_argv[0], (char **)rsync_argv , NULL)) == -1)
		{
			printf("run_map_rsync(): execve failed\n");
		}
	}
	else
	{
		rsync_running = 1;
	}
}

static int poll_map_rsync()
{
	if(!rsync_running)
		return -998;

	int status = 0;
	if(waitpid(my_pid , &status , WNOHANG) == 0)
		return -999;

	rsync_running = 0;
	printf("rsync returned %d\n", status);
	return status;
}


void page_coords(int mm_x, int mm_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int unit_x = mm_x / MAP_UNIT_W;
	int unit_y = mm_y / MAP_UNIT_W;
	unit_x += MAP_MIDDLE_UNIT;
	unit_y += MAP_MIDDLE_UNIT;
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}

void unit_coords(int mm_x, int mm_y, int* unit_x, int* unit_y)
{
	int unit_x_t = mm_x / MAP_UNIT_W;
	int unit_y_t = mm_y / MAP_UNIT_W;
	unit_x_t += MAP_MIDDLE_UNIT;
	unit_y_t += MAP_MIDDLE_UNIT;

	*unit_x = unit_x_t;
	*unit_y = unit_y_t;
}

void mm_from_unit_coords(int unit_x, int unit_y, int* mm_x, int* mm_y)
{
	unit_x -= MAP_MIDDLE_UNIT;
	unit_y -= MAP_MIDDLE_UNIT;

	*mm_x = unit_x * MAP_UNIT_W;
	*mm_y = unit_y * MAP_UNIT_W;
}

void page_coords_from_unit_coords(int unit_x, int unit_y, int* pageidx_x, int* pageidx_y, int* pageoffs_x, int* pageoffs_y)
{
	int page_x = unit_x / MAP_PAGE_W;
	int page_y = unit_y / MAP_PAGE_W;
	int offs_x = unit_x - page_x*MAP_PAGE_W;
	int offs_y = unit_y - page_y*MAP_PAGE_W;

	*pageidx_x = page_x;
	*pageidx_y = page_y;
	*pageoffs_x = offs_x;
	*pageoffs_y = offs_y;
}


double mm_per_pixel = 10.0;


typedef struct
{
	int x;
	int y;
} xy_t;

typedef struct route_unit_T
{
	xy_t loc;
	int backmode;
	route_unit_T* prev;
	route_unit_T* next;
} route_unit_t;


#define sq(x) ((x)*(x))

void dev_draw_circle(sf::RenderWindow& win, int unit_x, int unit_y, int r, int g, int b, int dir)
{
	int x_mm, y_mm;
	mm_from_unit_coords(unit_x, unit_y, &x_mm, &y_mm);

	sf::CircleShape circ(14.0/mm_per_pixel);
	circ.setOrigin(14.0/mm_per_pixel, 14.0/mm_per_pixel);
	circ.setFillColor(sf::Color(r,g,b));

	if(dir==-123)
	{
		circ.setFillColor(sf::Color::Transparent);
		circ.setOutlineThickness(2.0);
		circ.setOutlineColor(sf::Color(r,g,b));

	}

	circ.setPosition((x_mm+origin_x+MAP_UNIT_W/2)/mm_per_pixel,(y_mm+origin_y+MAP_UNIT_W/2)/mm_per_pixel);
	win.draw(circ);

	if(dir >= 0)
	{
		sf::ConvexShape arrow(3);
		arrow.setPoint(0, sf::Vector2f(0,-12.0/mm_per_pixel));
		arrow.setPoint(1, sf::Vector2f(0,12.0/mm_per_pixel));
		arrow.setPoint(2, sf::Vector2f(12.0/mm_per_pixel,0));

		arrow.setOrigin(0,0);

		arrow.setFillColor(sf::Color((r/2),(g/2),(b/2)));

		arrow.setRotation((float)dir*(360.0/32.0));
		arrow.setPosition((x_mm+origin_x+MAP_UNIT_W/2)/mm_per_pixel,(y_mm+origin_y+MAP_UNIT_W/2)/mm_per_pixel);
		win.draw(arrow);
	}

}

void draw_map(sf::RenderWindow& win);

#define TODEG(x) ((360.0*x)/(2.0*M_PI))


route_unit_t *some_route = NULL;
route_unit_t *p_cur_step = NULL;

void clear_route(route_unit_t **route)
{
	route_unit_t *elt, *tmp;
	DL_FOREACH_SAFE(*route,elt,tmp)
	{
		DL_DELETE(*route,elt);
		free(elt);
	}
	*route = NULL;
}

void draw_route_mm(sf::RenderWindow& win, route_unit_t **route)
{
	if(!route)
		return;

	route_unit_t *rt;
	route_unit_t *prev;
	int first = 1;
	DL_FOREACH(*route, rt)
	{
		float x1, x2, y1, y2;

		if(first)
		{
			x1 = (route_start_x+origin_x)/mm_per_pixel;
			y1 = (route_start_y+origin_y)/mm_per_pixel;
			first = 0;
		}
		else
		{
			x1 = (prev->loc.x+origin_x)/mm_per_pixel;
			y1 = (prev->loc.y+origin_y)/mm_per_pixel;
		}

		x2 = (rt->loc.x+origin_x)/mm_per_pixel;
		y2 = (rt->loc.y+origin_y)/mm_per_pixel;
		sf::RectangleShape rect(sf::Vector2f( sqrt(pow(x2-x1,2)+pow(y2-y1,2)), 6.0));
		rect.setOrigin(0, 3.0);
		rect.setPosition(x1, y1);
		rect.setRotation(atan2(y2-y1,x2-x1)*180.0/M_PI);
		rect.setFillColor(rt->backmode?sf::Color(180,0,0,170):sf::Color(0,180,0,170));

		win.draw(rect);

		prev = rt;
	}
}

//#define WALL_LEVEL(i) ((int)(i).num_obstacles*4)
#define WALL_LEVEL(i) ((int)(i).num_obstacles*2)

void draw_page(sf::RenderWindow& win, map_page_t* page, int startx, int starty)
{
	if(!page)
		return;

	static uint8_t pixels[MAP_PAGE_W*MAP_PAGE_W*4];

	for(int x = 0; x < MAP_PAGE_W; x++)
	{
		for(int y = 0; y < MAP_PAGE_W; y++)
		{
			if(page->units[x][y].constraints & CONSTRAINT_FORBIDDEN)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 255;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 110;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 190;
				pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
			}
			else
			{
//				int alpha = (30*(int)page->units[x][y].num_seen)/3 + (255/3);
//				int alpha = (2*(int)page->units[x][y].num_seen) + (255/6);
				int alpha = (8*(int)page->units[x][y].num_seen) + (255/2);
				if(alpha > 255) alpha=255;
				if(page->units[x][y].result & UNIT_DBG)
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
				}
/*				else if(page->units[x][y].result & UNIT_ITEM)
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+2] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
				}*/
				else if(page->units[x][y].result & UNIT_INVISIBLE_WALL)
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 200;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+3] = 255; //alpha;
				}
				else if(page->units[x][y].num_obstacles)
				{
					int lvl = WALL_LEVEL(page->units[x][y]);
					if(lvl > 170) lvl = 170;
					int color = 170 - lvl;
					if(!(page->units[x][y].result & UNIT_WALL))
					{
						color = 255;
					}

					pixels[4*(y*MAP_PAGE_W+x)+0] = color;
					pixels[4*(y*MAP_PAGE_W+x)+1] = color;
					pixels[4*(y*MAP_PAGE_W+x)+2] = color;
					pixels[4*(y*MAP_PAGE_W+x)+3] = alpha;
				}
				else if(page->units[x][y].result & UNIT_MAPPED)
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 240 - sqrt(page->units[x][y].num_visited*150);
					pixels[4*(y*MAP_PAGE_W+x)+2] = 190;
					pixels[4*(y*MAP_PAGE_W+x)+3] = alpha;
				}
				else
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 230;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 230;
					pixels[4*(y*MAP_PAGE_W+x)+2] = 230;
					pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
				}

#define PIX_MINUS(what_, how_) do{ int t_ = (what_) - (how_); if(t_ < 0) t_ = 0; (what_) = t_;} while(0);
#define PIX_PLUS(what_, how_) do{ int t_ = (what_) + (how_); if(t_ > 255) t_ = 255; (what_) = t_;} while(0);

				if(!(page->units[x][y].result & UNIT_INVISIBLE_WALL))
				{
					if(page->units[x][y].result & UNIT_3D_WALL)
					{
//						pixels[4*(y*MAP_PAGE_W+x)+0] >>= 1;
//						pixels[4*(y*MAP_PAGE_W+x)+1] >>= 0;
//						pixels[4*(y*MAP_PAGE_W+x)+2] >>= 1;
						PIX_PLUS(pixels[4*(y*MAP_PAGE_W+x)+0], 30);
						PIX_PLUS(pixels[4*(y*MAP_PAGE_W+x)+1], 30);
						PIX_MINUS(pixels[4*(y*MAP_PAGE_W+x)+2], 60);

//						int a = pixels[4*(y*MAP_PAGE_W+x)+3]<<1; if(a>255) a=255;
//						pixels[4*(y*MAP_PAGE_W+x)+3] = a;
					}
					else if(page->units[x][y].result & UNIT_DROP)
					{
//						pixels[4*(y*MAP_PAGE_W+x)+0] >>= 0;
//						pixels[4*(y*MAP_PAGE_W+x)+1] >>= 1;
//						pixels[4*(y*MAP_PAGE_W+x)+2] >>= 0;
						PIX_PLUS(pixels[4*(y*MAP_PAGE_W+x)+0], 20);
						PIX_MINUS(pixels[4*(y*MAP_PAGE_W+x)+1], 60);
						PIX_PLUS(pixels[4*(y*MAP_PAGE_W+x)+2], 30);
//						int a = pixels[4*(y*MAP_PAGE_W+x)+3]<<1; if(a>255) a=255;
//						pixels[4*(y*MAP_PAGE_W+x)+3] = a;
					}
					else if(page->units[x][y].result & UNIT_ITEM)
					{
//						pixels[4*(y*MAP_PAGE_W+x)+0] >>= 0;
//						pixels[4*(y*MAP_PAGE_W+x)+1] >>= 0;
//						pixels[4*(y*MAP_PAGE_W+x)+2] >>= 2;
						PIX_PLUS(pixels[4*(y*MAP_PAGE_W+x)+0], 30);
						PIX_MINUS(pixels[4*(y*MAP_PAGE_W+x)+1], 30);
						PIX_MINUS(pixels[4*(y*MAP_PAGE_W+x)+2], 30);
//						int a = pixels[4*(y*MAP_PAGE_W+x)+3]<<1; if(a>255) a=255;
//						pixels[4*(y*MAP_PAGE_W+x)+3] = a;
					}
				}
			}
		}
	}

	float scale = ((float)MAP_PAGE_W_MM/mm_per_pixel)/256.0f;

	sf::Texture t;
	t.create(256, 256);
	t.setSmooth(false);
	t.update(pixels);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setPosition((startx)/mm_per_pixel, (starty)/mm_per_pixel);
	sprite.setScale(sf::Vector2f(scale, scale));
	win.draw(sprite);

	sf::RectangleShape b1(sf::Vector2f(MAP_PAGE_W_MM/mm_per_pixel, 1));
	b1.setPosition((startx+0*MAP_UNIT_W)/mm_per_pixel, (starty+0*MAP_UNIT_W)/mm_per_pixel);
	b1.setFillColor(sf::Color(0,0,0,64));
	win.draw(b1);

	sf::RectangleShape b2(sf::Vector2f(MAP_PAGE_W_MM/mm_per_pixel, 1));
	b2.setPosition((startx+0*MAP_UNIT_W)/mm_per_pixel, (starty+256*MAP_UNIT_W)/mm_per_pixel);
	b2.setFillColor(sf::Color(0,0,0,64));
	win.draw(b2);

	sf::RectangleShape b3(sf::Vector2f(1, MAP_PAGE_W_MM/mm_per_pixel));
	b3.setPosition((startx+0*MAP_UNIT_W)/mm_per_pixel, (starty+0*MAP_UNIT_W)/mm_per_pixel);
	b3.setFillColor(sf::Color(0,0,0,64));
	win.draw(b3);

	sf::RectangleShape b4(sf::Vector2f(1, MAP_PAGE_W_MM/mm_per_pixel));
	b4.setPosition((startx+256*MAP_UNIT_W)/mm_per_pixel, (starty+0*MAP_UNIT_W)/mm_per_pixel);
	b4.setFillColor(sf::Color(0,0,0,64));
	win.draw(b4);

}

int32_t hwdbg[10];

void draw_hwdbg(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[500];
	t.setFont(arial);
	t.setCharacterSize(11);
	t.setFillColor(sf::Color(0,0,0,190));
	for(int i = 0; i<10; i++)
	{
		sprintf(buf, "dbg[%2i] = %11d (%08x)", i, hwdbg[i], hwdbg[i]);
		t.setString(buf);
		t.setPosition(10,screen_y-170-30 + 15*i);
		win.draw(t);
	}
}

void draw_bat_status(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[256];
	t.setFont(arial);

	sprintf(buf, "BATT %2.2f V (%d%%)", bat_voltage, bat_percentage);
	t.setString(buf);
	t.setCharacterSize(18);
	float vlevel = (float)bat_percentage/100.0;
	int r = (1.0-vlevel)*250.0;
	int g = vlevel*250.0;
	if(r > 250) r = 250; if(r<0) r=0;
	if(g > 250) g = 250; if(g<0) g=0;
	t.setFillColor(sf::Color(r,g,0));
	t.setPosition(screen_x-180,screen_y-45);
	win.draw(t);

	sprintf(buf, "charger input %2.2f V", cha_voltage);
	t.setString(buf);
	t.setCharacterSize(14);
	if(cha_voltage < 1.0)
		t.setFillColor(sf::Color(50,50,255));
	else if(cha_voltage < 22.0 || cha_voltage > 27.0)
		t.setFillColor(sf::Color(255,0,0));
	else
		t.setFillColor(sf::Color(0,255,0));

	t.setPosition(screen_x-180,screen_y-20);
	win.draw(t);

	if(charging)
	{
		t.setString("charging");
		t.setCharacterSize(16);
		t.setFillColor(sf::Color(200,110,0));
		t.setPosition(screen_x-130,screen_y-65);
		win.draw(t);
	}

	if(charge_finished)
	{
		t.setString("charge finished");
		t.setCharacterSize(16);
		t.setFillColor(sf::Color(10,200,50));
		t.setPosition(screen_x-160,screen_y-65);
		win.draw(t);
	}
}

int state_is_unsynchronized;

void draw_texts(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[256];
	t.setFont(arial);

	const int bot_box_xs = 400;
	const int bot_box_ys = 63;
	sf::RectangleShape rect(sf::Vector2f( bot_box_xs, bot_box_ys));
	rect.setPosition(screen_x/2 - bot_box_xs/2, screen_y-bot_box_ys-10-30);
	rect.setFillColor(sf::Color(255,255,255,160));
	win.draw(rect);


	sprintf(buf, "robot: x=%d  y=%d  mm  (ang=%.1f deg)", (int)cur_x, (int)cur_y, cur_angle);
	t.setString(buf);
	t.setCharacterSize(17);
	t.setFillColor(sf::Color(0,0,0,160));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-51-30);
	win.draw(t);

	sprintf(buf, "cursor: x=%d  y=%d  mm", (int)click_x, (int)click_y);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0, 120));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-30-30);
	win.draw(t);

	if(state_is_unsynchronized)
		sprintf(buf, "Robot state is unsynchronized...");
	else if(rsync_running)
		sprintf(buf, "Syncing maps...");
	else
		sprintf(buf, "%s", click_mode_names[click_mode]);
	t.setString(buf);
	t.setCharacterSize(17);
	t.setFillColor(sf::Color(0,0,0,130));
	t.setPosition(screen_x/2-bot_box_xs/2+11,screen_y-72-30);
	win.draw(t);
	t.setFillColor(rsync_running?sf::Color(0,190,20,200):click_mode_colors[click_mode]);
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-73-30);
	win.draw(t);

	const float dbg_point_r = 35.0;
	if(show_dbgpoint)
	{


		sf::CircleShape circ(2.0*dbg_point_r/mm_per_pixel);
		circ.setOrigin(2.0*dbg_point_r/mm_per_pixel, 2.0*dbg_point_r/mm_per_pixel);
		circ.setFillColor(sf::Color(dbgpoint_r,dbgpoint_g,dbgpoint_b, 180));
		circ.setOutlineColor(sf::Color(255,255,255,255));

		circ.setPosition((dbgpoint_x+origin_x)/mm_per_pixel,(dbgpoint_y+origin_y)/mm_per_pixel);
		win.draw(circ);
	}

	for(int i = 0; i < num_pers_dbgpoints; i++)
	{

		sf::CircleShape circ(dbg_point_r/mm_per_pixel);
		circ.setOrigin(dbg_point_r/mm_per_pixel, dbg_point_r/mm_per_pixel);
		circ.setFillColor(sf::Color(pers_dbgpoint_r[i],pers_dbgpoint_g[i],pers_dbgpoint_b[i], 120));
		circ.setOutlineColor(sf::Color(0,0,0,150));

		circ.setPosition((pers_dbgpoint_x[i]+origin_x)/mm_per_pixel,(pers_dbgpoint_y[i]+origin_y)/mm_per_pixel);
		win.draw(circ);

	}

}

void draw_picture(sf::RenderWindow& win)
{
	if(pict_id < 0 || pict_xs < 1 || pict_ys < 1 || pict_xs > 500 || pict_ys > 500)
	{
		return;
	}

	static uint8_t pixels[500*500*4];
/*	if(pict_bpp == 1)
	{
		for(int i=0; i<pict_xs*pict_ys; i++)
		{
			pixels[4*i+0] = pict_data[i];
			pixels[4*i+1] = pict_data[i];
			pixels[4*i+2] = pict_data[i];
			pixels[4*i+3] = 255;
		}
	}
*/

	sf::Vector2i m = sf::Mouse::getPosition(win);


	float scale = 8.0;

	sf::Texture t;
	t.create(pict_xs, pict_ys);
	t.setSmooth(false);
	sf::Sprite sprite;

	float mx = m.x;
	float my = m.y;

	int pic_x = 15, pic_y;
#ifdef TOF_DEV
	if(pict_id==100)
	{
		pic_y = 15+scale*0*pict_ys+10;
		mx -= 15; my -= 15+scale*pict_ys+10;
	}
	else if(pict_id==101 || pict_id==110)
	{
		pic_y = 15+scale*1*pict_ys+20;
		mx -= 15; my -= 15+2*scale*pict_ys+20;
	}
	else
	{
#endif
		pic_y = 15;
		mx -= 15; my -= 15;
#ifdef TOF_DEV
	}
#endif
	sprite.setPosition(pic_x, pic_y);

	mx /= scale;
	my /= scale;

	int process_as_dist = 0;

	if(pict_id == 1 || pict_id == 2 || pict_id == 3 || pict_id == 4) // ignore map
	{
		for(int i=0; i<pict_xs*pict_ys; i++)
		{
			if(pict_data[i])
			{
				pixels[4*i+0] = 255;
				pixels[4*i+1] = 128;
				pixels[4*i+2] = 50;
			}
			else
			{
				pixels[4*i+0] = 0;
				pixels[4*i+1] = 0;
				pixels[4*i+2] = 0;
			}
			pixels[4*i+3] = 255;
		}
	}
	else if(pict_id == 101 || pict_id == 5 || pict_id == 7 || pict_id == 9) // amplitudes
	{
		for(int i=0; i<pict_xs*pict_ys; i++)
		{
			int val = pict_data[i];
			if(dbg_boost) { val*=4; if(val>255) val=255;}
			pixels[4*i+0] = val;
			pixels[4*i+1] = val;
			pixels[4*i+2] = val;
			pixels[4*i+3] = 255;
		}
	}
	else if(pict_id == 100 || pict_id == 6 || pict_id == 8 || pict_id == 10 || pict_id == 110) // distances
	{
		process_as_dist = 1;
		for(int i=0; i<pict_xs*pict_ys; i++)
		{

			int dist = ((uint16_t*)pict_data)[i];
			int r, g, b;

			if(dist == 0)
			{
				r = 0; g = 0; b = 50;
			}
			else if(dist > 6000)
			{
				r = 200; g = 200; b = 200;
			}
			else
			{
				float blue_dist=5000;
				float percolor = blue_dist/3.0;

				float mm = dist;
				float f_r = 1.0 - fabs(mm-0*percolor)/percolor;
				float f_g = 1.0 - fabs(mm-1*percolor)/percolor;
				float f_b = 1.0 - fabs(mm-2*percolor)/percolor;

				r = f_r*256.0; if(r<0) r=0; else if(r>255) r=255;
				g = f_g*256.0; if(g<0) g=0; else if(g>255) g=255;
				b = f_b*256.0; if(b<0) b=0; else if(b>255) b=255;

			}

			pixels[4*i+0] = r;
			pixels[4*i+1] = g;
			pixels[4*i+2] = b;
			pixels[4*i+3] = 255;

		}
	}

	sf::Text te2;
	char tbuf2[16];
	if(mx >= 0 && mx < pict_xs && my >= 0 && my <= pict_ys)
	{
		int imy = (int)my;
		int imx = (int)mx;
		pixels[4*(imy*pict_xs+imx)+0] = 128;
		pixels[4*(imy*pict_xs+imx)+1] = 255;
		pixels[4*(imy*pict_xs+imx)+2] = 255;
		pixels[4*(imy*pict_xs+imx)+3] = 255;

		int val;
		if(process_as_dist)
			val = ((uint16_t*)pict_data)[imy*pict_xs+imx];
		else
			val = pict_data[imy*pict_xs+imx];


		sprintf(tbuf2, "(%d,%d)=%d", imx, imy, val);
		te2.setFont(arial);
		te2.setFillColor(sf::Color(0,0,0,255));
		te2.setString(tbuf2);
		te2.setCharacterSize(12);
		te2.setPosition(pic_x+scale*mx+8, pic_y+scale*my-5);

	}

	t.update(pixels);
	sprite.setTexture(t);
	sprite.setScale(sf::Vector2f(scale, scale));
	win.draw(sprite);
	win.draw(te2);
	te2.setFillColor(sf::Color(255,255,255,255));
	te2.setPosition(pic_x+scale*mx+8-1, pic_y+scale*my-5-1);
	win.draw(te2);

	{
		sf::Text te;
		char tbuf[16];
		sprintf(tbuf, "ID=%d", pict_id);
		te.setFont(arial);
		te.setFillColor(sf::Color(0,0,0,255));
		te.setString(tbuf);
		te.setCharacterSize(9);
		te.setPosition(20,2);
		win.draw(te);
	}


#ifdef TOF_DEV
	pict_id = -1;
#endif

}


void draw_map(sf::RenderWindow& win)
{
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(world.pages[x][y])
			{
				int startx = -MAP_MIDDLE_UNIT*MAP_UNIT_W + x*MAP_PAGE_W*MAP_UNIT_W + origin_x;
				int starty = -MAP_MIDDLE_UNIT*MAP_UNIT_W + y*MAP_PAGE_W*MAP_UNIT_W + origin_y;
				draw_page(win, world.pages[x][y], startx, starty);
			}
		}
	}
}


void draw_robot(sf::RenderWindow& win)
{
	sf::ConvexShape r(7);
	r.setPoint(0, sf::Vector2f(0,0));
	r.setPoint(1, sf::Vector2f(0,robot_ys/mm_per_pixel));
	r.setPoint(2, sf::Vector2f(robot_xs/mm_per_pixel,robot_ys/mm_per_pixel));
	r.setPoint(3, sf::Vector2f(robot_xs/mm_per_pixel,0.70*robot_ys/mm_per_pixel));
	r.setPoint(4, sf::Vector2f((robot_xs+0.2*robot_ys)/mm_per_pixel,0.5*robot_ys/mm_per_pixel));
	r.setPoint(5, sf::Vector2f(robot_xs/mm_per_pixel,0.30*robot_ys/mm_per_pixel));
	r.setPoint(6, sf::Vector2f(robot_xs/mm_per_pixel,0));

	r.setOrigin((0.5*robot_xs+lidar_xoffs)/mm_per_pixel,(0.5*robot_ys+lidar_yoffs)/mm_per_pixel);

	r.setFillColor(sf::Color(200,90,50,160));

	r.setRotation(cur_angle);
	r.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

	win.draw(r);

	if(static_cast<int>(dest_type) > -1 && static_cast<int>(dest_type) < 8)
	{
		const float robot_mark_radius = 200.0;
		const float robot_mark_radius2 = 40.0;
		sf::CircleShape circ(robot_mark_radius/mm_per_pixel);
		circ.setOrigin(robot_mark_radius/(mm_per_pixel), robot_mark_radius/(mm_per_pixel));
		circ.setFillColor(click_mode_colors[static_cast<int>(dest_type)]);
		circ.setOutlineThickness(1.0);
		circ.setOutlineColor(sf::Color(0,0,0,100));
		circ.setPosition((dest_x+origin_x)/mm_per_pixel,(dest_y+origin_y)/mm_per_pixel);
		win.draw(circ);
		sf::CircleShape circ2(robot_mark_radius2/mm_per_pixel);
		circ2.setOrigin(robot_mark_radius2/(mm_per_pixel), robot_mark_radius2/(mm_per_pixel));
		circ2.setFillColor(click_mode_colors[static_cast<int>(dest_type)]);
		circ2.setOutlineThickness(1.0);
		circ2.setOutlineColor(sf::Color(0,0,0,150));
		circ2.setPosition((dest_x+origin_x)/mm_per_pixel,(dest_y+origin_y)/mm_per_pixel);
		win.draw(circ2);

		if(dest_type == MODE_ADDCONSTRAINT || dest_type == MODE_REMCONSTRAINT)
			dest_type = MODE_INVALID;
	}
}

/*typedef struct
{
	int32_t ang; // int32_t range --> -180..+180 deg; let it overflow freely. 1 unit = 83.81903171539 ndeg
	int32_t x;   // in mm
	int32_t y;
} pos_t;

typedef struct
{
	int valid;
	int32_t x;   // in mm
	int32_t y;
} point_t;
*/

#define MAX_LIDAR_POINTS 720

typedef struct
{
	pos_t robot_pos;
	int n_points;
	point_t scan[MAX_LIDAR_POINTS];
} client_lidar_scan_t;

typedef struct
{
	pos_t robot_pos;
	int xsamples;
	int ysamples;
	int unit_size;
	int8_t data[256*256];
} client_tof3d_hmap_t;

client_lidar_scan_t lidar;
#define SONAR_POINTS 6
sonar_point_t sonar[SONAR_POINTS];
static int sonar_wr = 0;


client_tof3d_hmap_t hmap;


#define HMAP_ALPHA 255UL

#define TOF3D_WALL           8 
#define TOF3D_BIG_ITEM       7 
#define TOF3D_LOW_CEILING    6 
#define TOF3D_BIG_DROP       5
#define TOF3D_SMALL_ITEM     4 
#define TOF3D_SMALL_DROP     3
#define TOF3D_THRESHOLD      2   
#define TOF3D_FLOOR          1
#define TOF3D_UNSEEN         0

#define RGBA32(r_,g_,b_,a_)  ((r_) | ((g_)<<8) | ((b_)<<16) | ((a_)<<24))

static const uint32_t hmap_colors[9] = {
/* 0 UNSEEN     */ RGBA32(128UL,128UL,128UL, 0),
/* 1 FLOOR      */ RGBA32(150UL,255UL,150UL, HMAP_ALPHA/2),
/* 2 THRESHOLD  */ RGBA32(  0UL,200UL,200UL, HMAP_ALPHA),
/* 3 SMALL_DROP */ RGBA32( 50UL,  0UL,200UL, HMAP_ALPHA),
/* 4 SMALL_ITEM */ RGBA32(  0UL,255UL,  0UL, HMAP_ALPHA),
/* 5 BIG_DROP   */ RGBA32(220UL,  0UL,220UL, HMAP_ALPHA),
/* 6 LOW_CEILING*/ RGBA32(255UL,  0UL, 50UL, HMAP_ALPHA),
/* 7 BIG_ITEM   */ RGBA32(220UL,100UL,  0UL, HMAP_ALPHA),
/* 8 WALL       */ RGBA32(200UL,200UL,  0UL, HMAP_ALPHA)
};


static int hmap_alpha_mult = 255;

void draw_tof3d_hmap(sf::RenderWindow& win, client_tof3d_hmap_t* hm)
{
	if(hm->xsamples == 0 || hm->ysamples == 0)
		return;

	if(hm->xsamples > 256 || hm->ysamples > 256)
	{
		printf("Invalid hmap size\n");
		return;
	}

	static uint32_t pixels[256*256];


	float scale = (float)hm->unit_size/mm_per_pixel;

	for(int sy=0; sy < hm->ysamples; sy++)
	{
		for(int sx=0; sx < hm->xsamples; sx++)
		{
			uint8_t val = hm->data[sy*hm->xsamples+sx];
			if(val > 8)
			{
				printf("draw_tof3d_hmap() invalid val %d at (%d, %d)\n", val, sx, sy);
				continue;
			}
			pixels[sy*hm->xsamples+sx] = hmap_colors[val];
		}
	}

	float ang = hm->robot_pos.ang;

	sf::Texture t;
	t.create(hm->xsamples, hm->ysamples);
	t.setSmooth(false);
	t.update((uint8_t*)pixels);
	sf::Sprite sprite;
	sprite.setTexture(t);
	sprite.setOrigin(hm->xsamples/2.0, hm->ysamples/2.0);
	sprite.setRotation(ang);
	sprite.setPosition((hm->robot_pos.x+origin_x)/mm_per_pixel, (hm->robot_pos.y+origin_y)/mm_per_pixel);
	sprite.setScale(sf::Vector2f(scale, scale));
	sprite.setColor(sf::Color(255,255,255,hmap_alpha_mult));
	win.draw(sprite);

}

void draw_lidar(sf::RenderWindow& win, client_lidar_scan_t* lid)
{
	for(int i=0; i < lid->n_points; i++)
	{
		sf::RectangleShape rect(sf::Vector2f(3,3));
		rect.setOrigin(1.5,1.5);
		rect.setPosition((lid->scan[i].x+origin_x)/mm_per_pixel, (lid->scan[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(255, 0, 0, 100));
		win.draw(rect);
	}
}

void draw_sonar(sf::RenderWindow& win)
{
	for(int i=0; i < SONAR_POINTS; i++)
	{
		int c = sonar[i].c;
		if(c==0) continue;

		sf::RectangleShape rect(sf::Vector2f(6,6));
		rect.setOrigin(3,3);
		rect.setPosition((sonar[i].x+origin_x)/mm_per_pixel, (sonar[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(50,50,200));
		win.draw(rect);

		#if SONAR_POINTS < 10

		sf::Text t;
		char tbuf[16];
		sprintf(tbuf, "%d", sonar[i].z);
		t.setFont(arial);
		t.setFillColor(sf::Color(0,0,0,255));
		t.setString(tbuf);
		t.setCharacterSize(9);
		t.setPosition((sonar[i].x+origin_x)/mm_per_pixel, (sonar[i].y+origin_y)/mm_per_pixel);
		win.draw(t);

		#endif
	}
}


sf::IpAddress serv_ip;
unsigned short serv_port;

sf::TcpSocket tcpsock;

void mode_msg(uint8_t mode)
{
	uint8_t test[4] = {58 /*MODE*/, 0, 1, mode};

	if(tcpsock.send(test, 4) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}

void speedlimit_msg(uint8_t limit)
{
	uint8_t test[8] = {63 /*SPEEDLIM*/, 0, 5, limit, limit, limit, 40, 40};

	if(tcpsock.send(test, 8) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}


void go_charge_msg(uint8_t params)
{
	uint8_t test[4] = {57, 0, 1, params};

	if(tcpsock.send(test, 4) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}

void maintenance_msg(int restart_mode)
{
	const int size = 1+2+4+4;
	uint8_t test[size];
	test[0] = 62;
	test[1] = ((size-3)&0xff00)>>8;
	test[2] = (size-3)&0xff;
	I32TOBUF(0x12345678, test, 3);
	I32TOBUF(restart_mode, test, 7);

	if(tcpsock.send(test, size) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}

state_vect_t received_state_vect;
state_vect_t state_vect_to_send;

void send_state_vect()
{
	uint8_t test[3+STATE_VECT_LEN];
	test[0] = 64;
	test[1] = ((STATE_VECT_LEN)&0xff00)>>8;
	test[2] = (STATE_VECT_LEN)&0xff;
	memcpy(&test[3], state_vect_to_send.table, STATE_VECT_LEN);

	if(tcpsock.send(test, 3+STATE_VECT_LEN) != sf::Socket::Done)
	{
		printf("Send error\n");
	}
}


#define NUM_DECORS 8

info_state_t cur_info_state = INFO_STATE_UNDEF;

int main(int argc, char** argv)
{
	bool f_pressed[13] = {false};
	bool return_pressed = false;
	int focus = 1;
	int online = 1;


	if(argc != 3)
	{
		printf("Usage: rn1client addr port\n");
		printf("Starting in offline mode.\n");
		online = 0;
	}

	init_rsync_argv();
	sprintf(status_text, "Status bar");

	if(online)
	{
		strncpy(rsync_argv[2], argv[1], 1023);
		rsync_argv[2][1023] = 0;

		serv_ip = argv[1];
		serv_port = atoi(argv[2]);

		tcpsock.setBlocking(false);
		printf("Connecting...\n");
		while(tcpsock.connect(serv_ip, serv_port) != sf::Socket::Done)
		{
			usleep(1000);
			//TODO: timeout
		}
	}

	if (!arial.loadFromFile("arial.ttf"))
	{
	    return 1;
	}

	sf::ContextSettings sets;
	sets.antialiasingLevel = 8;
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "PULUROBOT SLAM", sf::Style::Default, sets);
	win.setFramerateLimit(30);

	sf::Texture decors[NUM_DECORS];

	decors[INFO_STATE_IDLE].loadFromFile    ("decoration/idle.png");
	decors[INFO_STATE_THINK].loadFromFile   ("decoration/think.png");
	decors[INFO_STATE_FWD].loadFromFile     ("decoration/fwd.png");
	decors[INFO_STATE_REV].loadFromFile     ("decoration/rev.png");
	decors[INFO_STATE_LEFT].loadFromFile    ("decoration/left.png");
	decors[INFO_STATE_RIGHT].loadFromFile   ("decoration/right.png");
	decors[INFO_STATE_CHARGING].loadFromFile("decoration/charging.png");
	decors[INFO_STATE_DAIJUING].loadFromFile("decoration/party.png");



	sfml_gui gui(win, arial);

	#define BUT_WIDTH 220

	int but_start_x = screen_x-BUT_WIDTH;

	int but_localize    = gui.add_button(but_start_x, 50 + 0*35, 140, 25, "     Localize (init)", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);
	int but_stop        = gui.add_button(but_start_x, 50 + 1*35, 65, 25, "Stop", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);

	int but_route     = gui.add_button(but_start_x, 60 + 2*35, 100, 25, "Route", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_ROUTE, DEF_BUT_COL_PRESSED, false);
	int but_manu_fwd  = gui.add_button(but_start_x, 60 + 3*35, 100, 25, "Manual fwd", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_MANUAL, DEF_BUT_COL_PRESSED, false);
	int but_manu_back = gui.add_button(but_start_x+110, 60 + 3*35, 30, 25, "rev", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_force_fwd = gui.add_button(but_start_x, 60 + 4*35, 100, 25, "Force fwd", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_FORCE, DEF_BUT_COL_PRESSED, false);
	int but_force_back= gui.add_button(but_start_x+110, 60 + 4*35, 30, 25, "rev", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_pose      = gui.add_button(but_start_x, 60 + 5*35, 100, 25, "Rotate pose", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_POSE, DEF_BUT_COL_PRESSED, false);

	int but_findcharger = gui.add_button(but_start_x, 70 + 6*35, 140, 25, "  Find charger", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_speedminus  = gui.add_button(but_start_x, 70 + 7*35, 25, 25, " -", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_speedplus  = gui.add_button(but_start_x+115, 70 + 7*35, 25, 25, " +", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_addconstraint  = gui.add_button(but_start_x, 70 + 8*35, 60, 25, "ADD", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_remconstraint  = gui.add_button(but_start_x+65, 70 + 8*35, 60, 25, "REM", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_state_vect[STATE_VECT_LEN];

	for(int i=0; i<STATE_VECT_LEN; i++)
	{
		but_state_vect[i] = gui.add_button(but_start_x, 70 + 9*35 + i*25, 190, 18, state_vect_names[i], DEF_BUT_COL, /*font size:*/11, -1, DEF_BUT_COL_PRESSED, SYM_STOP);
		state_vect_to_send.table[i] = received_state_vect.table[i] = 0;
	}
	

	bool right_click_on = false;
	bool left_click_on = false;
	double prev_click_x = 0.0, prev_click_y = 0.0;

	int cnt = 0;
	while(win.isOpen())
	{
		cnt++;
		int gui_box_xs = BUT_WIDTH;
		int gui_box_ys = screen_y-65;
		int gui_box_x = screen_x-BUT_WIDTH-15;
		int gui_box_y = 15;

		gui.buttons[but_route]->pressed =      (click_mode==MODE_ROUTE);
		gui.buttons[but_manu_fwd]->pressed =   (click_mode==MODE_MANUAL_FWD);
		gui.buttons[but_manu_back]->pressed =  (click_mode==MODE_MANUAL_BACK);
		gui.buttons[but_force_fwd]->pressed =  (click_mode==MODE_FORCE_FWD);
		gui.buttons[but_force_back]->pressed = (click_mode==MODE_FORCE_BACK);
		gui.buttons[but_pose]->pressed =       (click_mode==MODE_POSE);
		gui.buttons[but_addconstraint]->pressed = (click_mode==MODE_ADDCONSTRAINT);
		gui.buttons[but_remconstraint]->pressed = (click_mode==MODE_REMCONSTRAINT);

		state_is_unsynchronized = 0;
		for(int i=0; i<STATE_VECT_LEN; i++)
		{
			gui.buttons[but_state_vect[i]]->pressed = state_vect_to_send.table[i];
			if(received_state_vect.table[i] != state_vect_to_send.table[i])
			{
				gui.buttons[but_state_vect[i]]->symbol = SYM_PLAY;
				state_is_unsynchronized = 1;
			}
			else
			{
				gui.buttons[but_state_vect[i]]->symbol = SYM_STOP;
			}
		}


		if(poll_map_rsync() >= 0)
			load_all_pages_on_disk(&world);


		sf::Event event;
		while (win.pollEvent(event))
		{
			if(event.type == sf::Event::Closed)
				win.close();
			if(event.type == sf::Event::Resized)
			{
				sf::Vector2u size = win.getSize();
				screen_x = size.x;
				screen_y = size.y;
				but_start_x = screen_x-BUT_WIDTH;

				gui.buttons[but_route]->x = but_start_x;
				gui.buttons[but_manu_fwd]->x = but_start_x;
				gui.buttons[but_manu_back]->x = but_start_x+110;
				gui.buttons[but_force_fwd]->x = but_start_x;
				gui.buttons[but_force_back]->x = but_start_x+110;
				gui.buttons[but_pose]->x = but_start_x;
				gui.buttons[but_localize]->x = but_start_x;
				gui.buttons[but_stop]->x = but_start_x;
				gui.buttons[but_findcharger]->x = but_start_x;
				gui.buttons[but_speedminus]->x = but_start_x;
				gui.buttons[but_speedplus]->x = but_start_x+115;
				gui.buttons[but_addconstraint]->x = but_start_x;
				gui.buttons[but_remconstraint]->x = but_start_x+65;
		
				for(int i=0; i<STATE_VECT_LEN; i++)
				{
					gui.buttons[but_state_vect[i]]->x = but_start_x;
				}

				sf::FloatRect visibleArea(0, 0, screen_x, screen_y);
				win.setView(sf::View(visibleArea));
			}
			if(event.type == sf::Event::LostFocus)
				focus = 0;

			if(event.type == sf::Event::GainedFocus)
				focus = 1;

		}

		uint8_t buf[3] = {0,0,0};
		size_t received = 0;

		if(online)
		{
			sf::Socket::Status ret;
			if( (ret = tcpsock.receive(buf, 3, received)) == sf::Socket::Done)
			{
				if(received != 3)
				{
					printf("error horror.\n");
				}

				int msgid = buf[0];
				int len = ((int)buf[1]<<8) | buf[2];

				//printf("msgid=%d len=%d\n", msgid, len);

				if(len > 100000)
				{
					printf("Error: msg too long.\n");
					len=100000;
				}
				uint8_t rxbuf[100000];

				int total_rx = 0;
				while(total_rx < len)
				{
					if( (ret = tcpsock.receive(&rxbuf[total_rx], len-total_rx, received)) == sf::Socket::Done)
					{
						total_rx += received;
			//			printf("    rx %d -> total %d\n", received, total_rx);
					}
				}


				if(total_rx != len)
					printf("error horror2  %d != %d\n", total_rx, len);

				switch(msgid)
				{
					case 130:
					{
						cur_angle = ((double)I16FROMBUF(rxbuf, 0))/65536.0 * 360.0;
						cur_x = (int32_t)I32FROMBUF(rxbuf,2);
						cur_y = (int32_t)I32FROMBUF(rxbuf,6);
						if(len>10) cur_cmd_status = rxbuf[10];
					}
					break;

					case 131:
					{
						lidar.robot_pos.ang = ((double)I16FROMBUF(rxbuf, 0))/65536.0 * 360.0;
						int mid_x = lidar.robot_pos.x = (int32_t)I32FROMBUF(rxbuf,2);
						int mid_y = lidar.robot_pos.y = (int32_t)I32FROMBUF(rxbuf,6);

						int n_points = (len-10)/2;

						//printf("lowres lidar: n_points = %d\n", n_points);
						for(int i=0; i<n_points; i++)
						{
							int x = (int8_t)rxbuf[10+2*i];
							int y = (int8_t)rxbuf[10+2*i+1];
							lidar.scan[i].valid = 1;
							lidar.scan[i].x = x*160 + mid_x;
							lidar.scan[i].y = y*160 + mid_y;
						}
						lidar.n_points = n_points;
					}
					break;

					case 141:
					{
						lidar.robot_pos.ang = ((double)I16FROMBUF(rxbuf, 0))/65536.0 * 360.0;
						int mid_x = lidar.robot_pos.x = (int32_t)I32FROMBUF(rxbuf,2);
						int mid_y = lidar.robot_pos.y = (int32_t)I32FROMBUF(rxbuf,6);

						int n_points = (len-10)/4;

						//printf("highres lidar: n_points = %d\n", n_points);
						for(int i=0; i<n_points; i++)
						{
							int x = (int16_t)I16FROMBUF(rxbuf, 10+0+4*i);
							int y = (int16_t)I16FROMBUF(rxbuf, 10+2+4*i);
							lidar.scan[i].valid = 1;
							lidar.scan[i].x = x + mid_x;
							lidar.scan[i].y = y + mid_y;
						}
						lidar.n_points = n_points;
					}
					break;


					case 132:
					{
						for(int i=0; i<10; i++)
						{
							hwdbg[i] = (int32_t)I32FROMBUF(rxbuf,4*i);
						}
					}
					break;

					case 133: // sonar point
					{
						sonar[sonar_wr].x = (int32_t)I32FROMBUF(rxbuf,0);
						sonar[sonar_wr].y = (int32_t)I32FROMBUF(rxbuf,4);
						sonar[sonar_wr].z = (int32_t)I16FROMBUF(rxbuf,8);
						sonar[sonar_wr].c = rxbuf[10];

						//printf("SONAR: x=%d   y=%d   c=%d\n", sonar[sonar_wr].x, sonar[sonar_wr].y, sonar[sonar_wr].c);

						sonar_wr++; if(sonar_wr >= SONAR_POINTS) sonar_wr = 0;
					}
					break;

					case 134: // Battery status
					{
						charging = rxbuf[0]&1;
						charge_finished = rxbuf[0]&2;
						bat_voltage = (float)(((int)rxbuf[1]<<8) | rxbuf[2])/1000.0;
						bat_percentage = rxbuf[3];
						cha_voltage = (float)(((int)rxbuf[4]<<8) | rxbuf[5])/1000.0;
						//printf("bat status %d %d %f %d\n", charging, charge_finished, bat_voltage, bat_percentage);
					}
					break;

					case 135: // Route info
					{
						clear_route(&some_route);
						int n_elems = len/9;

						route_start_x = (int32_t)I32FROMBUF(rxbuf,0);
						route_start_y = (int32_t)I32FROMBUF(rxbuf,4);

						for(int i = 0; i < n_elems; i++)
						{
							route_unit_t* point = (route_unit_t*)malloc(sizeof(route_unit_t));
							point->backmode = rxbuf[i*9+8];
							point->loc.x = (int32_t)I32FROMBUF(rxbuf,i*9+9);
							point->loc.y = (int32_t)I32FROMBUF(rxbuf,i*9+13);
							printf("i=%d  back=%d, x=%d, y=%d\n", i, point->backmode, point->loc.x, point->loc.y);
							DL_APPEND(some_route, point);
						}
					}
					break;

					case 136:
					{
						run_map_rsync();
					}
					break;

					case 137: // dbg_point
					{

						if(rxbuf[11] == 0)
						{
							show_dbgpoint = 1;
							dbgpoint_x = (int32_t)I32FROMBUF(rxbuf,0);
							dbgpoint_y = (int32_t)I32FROMBUF(rxbuf,4);
							dbgpoint_r = rxbuf[8];
							dbgpoint_g = rxbuf[9];
							dbgpoint_b = rxbuf[10];
						}
						else
						{
							pers_dbgpoint_x[num_pers_dbgpoints] = (int32_t)I32FROMBUF(rxbuf,0);
							pers_dbgpoint_y[num_pers_dbgpoints] = (int32_t)I32FROMBUF(rxbuf,4);
							pers_dbgpoint_r[num_pers_dbgpoints] = rxbuf[8];
							pers_dbgpoint_g[num_pers_dbgpoints] = rxbuf[9];
							pers_dbgpoint_b[num_pers_dbgpoints] = rxbuf[10];
							num_pers_dbgpoints++;
							if(num_pers_dbgpoints > 99) num_pers_dbgpoints = 0;
						}


					}
					break;

					case 138: // 3D TOF HMAP
					{
						hmap_alpha_mult = 255;
						hmap.xsamples = I16FROMBUF(rxbuf, 0);
						hmap.ysamples = I16FROMBUF(rxbuf, 2);

						if(hmap.xsamples < 1 || hmap.xsamples > 256 || hmap.ysamples < 1 || hmap.ysamples > 256)
						{
							printf("Invalid 3D TOF HMAP xsamples * ysamples (%d * %d)\n", hmap.xsamples, hmap.ysamples);
							break;
						}
						hmap.robot_pos.ang = ((double)I16FROMBUF(rxbuf, 4))/65536.0 * 360.0;
						hmap.robot_pos.x = (int32_t)I32FROMBUF(rxbuf,6);
						hmap.robot_pos.y = (int32_t)I32FROMBUF(rxbuf,10);
						hmap.unit_size = rxbuf[14];

						memcpy(hmap.data, &rxbuf[15], hmap.xsamples*hmap.ysamples);
						//printf("Got %d x %d hmap at %d, %d, %d.\n", hmap.xsamples, hmap.ysamples, hmap.robot_pos.ang, hmap.robot_pos.x, hmap.robot_pos.y);
					}
					break;

					case 139: // info state
					{
						cur_info_state = static_cast<info_state_t>(rxbuf[0]);
					}
					break;


					case 140: // Robot info
					{
						robot_xs = (double)I16FROMBUF(rxbuf, 0);
						robot_ys = (double)I16FROMBUF(rxbuf, 2);
						lidar_xoffs = (double)I16FROMBUF(rxbuf, 4);
						lidar_yoffs = (double)I16FROMBUF(rxbuf, 6);

						printf("Robot size msg: xs=%.1f ys=%.1f lidar_x=%.1f lidar_y=%.1f\n", robot_xs, robot_ys, lidar_xoffs, lidar_yoffs);
					}
					break;

					case 142: // Picture
					{
						pict_id = I16FROMBUF(rxbuf, 0);
						pict_bpp = rxbuf[2];
						pict_xs = I16FROMBUF(rxbuf, 3);
						pict_ys = I16FROMBUF(rxbuf, 5);
						printf("Picture msg: id=%u bytes_per_pixel=%u xs=%u ys=%u\n", pict_id, pict_bpp, pict_xs, pict_ys);
						int pict_size = pict_bpp*pict_xs*pict_ys;
						if(pict_size > 100000)
						{
							printf("Ignoring oversized image.\n");
							pict_id = -1;
						}
						else
							memcpy(pict_data, &rxbuf[7], pict_size);
					}
					break;

					case 143: // Movement status
					{
						int16_t mov_start_ang = I16FROMBUF(rxbuf, 0);
						int32_t mov_start_x = I32FROMBUF(rxbuf, 2);
						int32_t mov_start_y = I32FROMBUF(rxbuf, 6);

						int32_t mov_requested_x = I32FROMBUF(rxbuf, 10);
						int32_t mov_requested_y = I32FROMBUF(rxbuf, 14);
						int8_t  mov_requested_backmode = rxbuf[18];

						int16_t mov_cur_ang = I16FROMBUF(rxbuf, 19);
						int32_t mov_cur_x = I32FROMBUF(rxbuf, 21);
						int32_t mov_cur_y = I32FROMBUF(rxbuf, 25);
						uint8_t mov_status = rxbuf[29];
						uint32_t mov_obstacle_flags = I32FROMBUF(rxbuf, 30);

						if(mov_status == 0)
							sprintf(status_text, "Manual movement SUCCESS, start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm", mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y);
						else
							sprintf(status_text, "Manual movement STOPPED, start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, statuscode=%u, HW obstacle flags=%08x", mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_status, mov_obstacle_flags);
					}
					break;

					case 144: // Route status
					{
						int16_t mov_start_ang = I16FROMBUF(rxbuf, 0);
						int32_t mov_start_x = I32FROMBUF(rxbuf, 2);
						int32_t mov_start_y = I32FROMBUF(rxbuf, 6);

						int32_t mov_requested_x = I32FROMBUF(rxbuf, 10);
						int32_t mov_requested_y = I32FROMBUF(rxbuf, 14);

						int16_t mov_cur_ang = I16FROMBUF(rxbuf, 18);
						int32_t mov_cur_x = I32FROMBUF(rxbuf, 20);
						int32_t mov_cur_y = I32FROMBUF(rxbuf, 24);

						uint8_t mov_status = rxbuf[28];
						int16_t mov_reroute_cnt = I16FROMBUF(rxbuf, 29);

						if(mov_status == 0)
							sprintf(status_text, "SUCCESSFULLY followed the route, start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, needed to reroute %d times", 
								mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_reroute_cnt);
						else
						{
							static const char* fail_reasons[5] =
							{
								"Success",
								"Obstacles on map close to the beginning, can't get started",
								"Got a good start thanks to backing off, but obstacles on the way later",
								"Got a good start, but obstacles on the way later",
								"Unknown (newly implemented?) reason"
							};

							uint8_t reason = mov_status; if(reason >= 4) reason=4;

							sprintf(status_text, "GAVE UP routefinding, reason: %u[%s], start=(%d,%d)mm -> req=(%d,%d)mm, actual=(%d,%d)mm, did reroute %d times (reason applies to the latest reroute)", mov_status, 
								fail_reasons[reason], mov_start_x, mov_start_y, mov_requested_x, mov_requested_y, mov_cur_x, mov_cur_y, mov_reroute_cnt);
						}
					}
					break;


					case 145: // State vector
					{
						if(len != STATE_VECT_LEN)
						{
							printf("Illegal state vector message length - do the API versions of rn1host and rn1client match?\n");
							break;
						}

						memcpy(received_state_vect.table, rxbuf, STATE_VECT_LEN);
						memcpy(state_vect_to_send.table, rxbuf, STATE_VECT_LEN);
					}
					break;

					default:
					break;
				}
			}
		}

		if(focus)
		{
			sf::Vector2i localPosition = sf::Mouse::getPosition(win);

			if(localPosition.x > gui_box_x-5 && localPosition.x < gui_box_x+gui_box_xs+5 && localPosition.y > gui_box_y-5 && localPosition.y < gui_box_y+gui_box_ys+5)
			{
				int but = gui.check_button_status();
				if     (but == but_route)      click_mode = MODE_ROUTE;
				else if(but == but_manu_fwd)   click_mode = MODE_MANUAL_FWD;
				else if(but == but_force_fwd)  click_mode = MODE_FORCE_FWD;
				else if(but == but_manu_back)  click_mode = MODE_MANUAL_BACK;
				else if(but == but_force_back) click_mode = MODE_FORCE_BACK;
				else if(but == but_pose)       click_mode = MODE_POSE;
				else if(but == but_addconstraint) click_mode = MODE_ADDCONSTRAINT;
				else if(but == but_remconstraint) click_mode = MODE_REMCONSTRAINT;

				if(but == but_speedplus)
				{
					gui.buttons[but_speedplus]->pressed = true;

					if(cnt&1)
					{
						if(cur_speed_limit < 10 || cur_speed_limit > 40)
							cur_speed_limit++;
						else
							cur_speed_limit = cur_speed_limit*11/10;

						if(cur_speed_limit > 70)
							cur_speed_limit = 70;
					}
				}
				else
				{
					if(gui.buttons[but_speedplus]->pressed)
					{
						gui.buttons[but_speedplus]->pressed = false;
						speedlimit_msg(cur_speed_limit);
					}
				}

				if(but == but_speedminus)
				{
					gui.buttons[but_speedminus]->pressed = true;

					if(cnt&1)
					{
						if(cur_speed_limit < 11)
							cur_speed_limit--;
						else
							cur_speed_limit = cur_speed_limit*10/11;

						if(cur_speed_limit < 1)
							cur_speed_limit = 1;
					}
					
				}
				else
				{
					if(gui.buttons[but_speedminus]->pressed)
					{
						gui.buttons[but_speedminus]->pressed = false;
						speedlimit_msg(cur_speed_limit);
					}
				}
				
				if(but == but_localize)
				{
					gui.buttons[but_localize]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_localize]->pressed)
					{
						gui.buttons[but_localize]->pressed = false;
						mode_msg(3);
					}
				}

				if(but == but_stop)
				{
					gui.buttons[but_stop]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_stop]->pressed)
					{
						mode_msg(8);
						gui.buttons[but_stop]->pressed = false;
					}
				}

				if(but == but_findcharger)
				{
					gui.buttons[but_findcharger]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_findcharger]->pressed)
					{
						go_charge_msg(0);
						gui.buttons[but_findcharger]->pressed = false;
					}
				}

				static bool statebut_pressed[STATE_VECT_LEN];
				for(int i=0; i<STATE_VECT_LEN; i++)
				{
					if(but == but_state_vect[i])
					{
						if(!statebut_pressed[i])
						{
							statebut_pressed[i] = true;
							state_vect_to_send.table[i] = received_state_vect.table[i]?0:1;
							send_state_vect();
						}				
					}
					else
						statebut_pressed[i] = false;
				}




			}
			else if(localPosition.x > 10 && localPosition.x < screen_x-10 && localPosition.y > 10 && localPosition.y < screen_y-10)
			{
				click_x = (localPosition.x * mm_per_pixel) - origin_x;
				click_y = (localPosition.y * mm_per_pixel) - origin_y;

				if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
				{
//					bool shift_on = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift);
					if(!left_click_on)
					{
						dest_type = click_mode;
						dest_x = click_x; dest_y = click_y;

						int back = 0;

						switch(click_mode)
						{
							case MODE_ROUTE: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[12] = {56 /*ROUTE*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0};

								if(tcpsock.send(test, 12) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: search for route");
								}
							} break;

							case MODE_MANUAL_BACK:
							back = 1;
							case MODE_MANUAL_FWD: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[12] = {55 /*DEST*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, back};

								if(tcpsock.send(test, 12) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: move directly");
								}

							} break;

							case MODE_FORCE_BACK:
							back = 1;
							case MODE_FORCE_FWD: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[12] = {55 /*DEST*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0b100 & back};

								if(tcpsock.send(test, 12) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: move directly (force)");
								}


							} break;

							case MODE_POSE: {
								clear_route(&some_route);

								int x = dest_x; int y = dest_y;

								uint8_t test[12] = {55 /*DEST*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, 0b1000};

								if(tcpsock.send(test, 12) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: rotate robot pose");
								}

							} break;

							case MODE_ADDCONSTRAINT: {
								int x = click_x; int y = click_y;
								uint8_t test[11] = {60, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(tcpsock.send(test, 11) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: add forbidden area");
								}

							} break;

							case MODE_REMCONSTRAINT: {
								int x = click_x; int y = click_y;
								uint8_t test[11] = {61, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(tcpsock.send(test, 11) != sf::Socket::Done)
								{
									printf("Send error\n");
									sprintf(status_text, "Send error, connection lost?");
								}
								else
								{
									sprintf(status_text, "Sent command: remove forbidden area");
								}

							} break;
							default: break;
						}

					}

					left_click_on = true;
				}
				else
					left_click_on = false;

				if(sf::Mouse::isButtonPressed(sf::Mouse::Right))
				{
					if(right_click_on)
					{
						double dx = click_x - prev_click_x; double dy = click_y - prev_click_y;
						origin_x += dx; origin_y += dy;
					}
					else
					{
						prev_click_x = click_x; prev_click_y = click_y;
					}

					right_click_on = true;
				}
				else
					right_click_on = false;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageUp))
			{
				origin_x -= (screen_x/2.0)*mm_per_pixel;
				origin_y -= (screen_y/2.0)*mm_per_pixel;
				mm_per_pixel *= 1.05;
				origin_x += (screen_x/2.0)*mm_per_pixel;
				origin_y += (screen_y/2.0)*mm_per_pixel;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::PageDown))
			{
				origin_x -= (screen_x/2.0)*mm_per_pixel;
				origin_y -= (screen_y/2.0)*mm_per_pixel;
				mm_per_pixel *= 0.95;
				origin_x += (screen_x/2.0)*mm_per_pixel;
				origin_y += (screen_y/2.0)*mm_per_pixel;
			}

//			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F1)) { click_mode = MODE_ROUTE; }
//			else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F2)) { click_mode = MODE_MANUAL; }
//			else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F3)) { click_mode = MODE_FORCE; }
//			else if(sf::Keyboard::isKeyPressed(sf::Keyboard::F4)) { click_mode = MODE_POSE; }

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F5)) { if(!f_pressed[5]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
					run_map_rsync();
				else
					load_all_pages_on_disk(&world);
				f_pressed[5] = true;
			}} else f_pressed[5] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F6)) { if(!f_pressed[6]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(6); // rn1host git pull + restart
					win.close();
				}
				f_pressed[6] = true;
			}} else f_pressed[6] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F7)) { if(!f_pressed[7]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(135); // reboot raspi
					win.close();
				}

				f_pressed[7] = true;
			}} else f_pressed[7] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F8)) { if(!f_pressed[8]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(10); // update firmware
					win.close();
				}
				f_pressed[8] = true;
			}} else f_pressed[8] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F9)) { if(!f_pressed[9]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(136); // shut down raspi
					win.close();
				}

				f_pressed[9] = true;
			}} else f_pressed[9] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F10)) { if(!f_pressed[10]) 
			{
				if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
				{
					maintenance_msg(7); // delete maps & restart rn1host
					win.close();
				}
				f_pressed[10] = true;
			}} else f_pressed[10] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F11)) { if(!f_pressed[11]) 
			{
				mode_msg(7); // conf charger
				f_pressed[11] = true;
			}} else f_pressed[11] = false;

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::C))
			{
				num_pers_dbgpoints = 0;
				dbg_boost = 0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::B))
			{
				dbg_boost = 1;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
			{
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
			{
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
			{
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
			{
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Return))
			{
				if(!return_pressed)
				{
					return_pressed = 1;
				}
			}
			else
			{
				return_pressed = 0;
			}



		}

		#ifndef TOF_DEV
		win.clear(sf::Color(230,230,230));

		draw_map(win);

		draw_robot(win);

		draw_lidar(win, &lidar);
		draw_sonar(win);
		draw_tof3d_hmap(win, &hmap);
		if(hmap_alpha_mult) hmap_alpha_mult-=8; if(hmap_alpha_mult < 40) hmap_alpha_mult = 40;

		draw_hwdbg(win);
		draw_bat_status(win);

		draw_route_mm(win, &some_route);

		draw_texts(win);
		sf::RectangleShape rect(sf::Vector2f( gui_box_xs, gui_box_ys));
		rect.setPosition(gui_box_x, gui_box_y);
		rect.setFillColor(sf::Color(255,255,255,160));
		win.draw(rect);
		gui.draw_all_buttons();



		{
			sf::Text t;
			char tbuf[256];
			t.setFont(arial);

			static int fx=0;

			sprintf(tbuf, "SPEED %d", cur_speed_limit);
			if(cur_speed_limit > 45)
				t.setFillColor(sf::Color(255,0,0,255));
			else
				t.setFillColor(sf::Color(200,200,0,255));
			t.setString(tbuf);
			t.setCharacterSize(14);
			t.setPosition(but_start_x+35, 70 + 7*35);
			win.draw(t);
		}

		{
			sf::Text t;
			t.setFont(arial);

			t.setString(status_text);
			t.setCharacterSize(12);
			t.setFillColor(sf::Color(255,255,255,200));
			t.setPosition(10, screen_y-15);
			win.draw(t);
			t.setFillColor(sf::Color(0,0,0,255));
			t.setPosition(8, screen_y-17);
			win.draw(t);
		}

		print_cur_cmd_status(win);

		if(static_cast<int>(cur_info_state) >= 0 && static_cast<int>(cur_info_state) < NUM_DECORS)
		{
			sf::Sprite decor_sprite;
			decor_sprite.setTexture(decors[static_cast<int>(cur_info_state)]);
			decor_sprite.setPosition(screen_x-180, (cur_info_state==INFO_STATE_DAIJUING)?(screen_y-240):(screen_y-220));
			win.draw(decor_sprite);
		}


		#endif

		draw_picture(win);

		win.display();

		usleep(100);

		static int sonar_fade = 0;
		if(++sonar_fade > 10)
		{
			sonar_fade = 0;
			sonar[sonar_wr].c = 0;
			sonar_wr++; if(sonar_wr >= SONAR_POINTS) sonar_wr = 0;
		}


	}

	deinit_rsync_argv();
	return 0;
}
