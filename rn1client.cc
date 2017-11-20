#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
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
#include "../rn1-brain/comm.h"
#include "client_memdisk.h"
#include "uthash.h"
#include "utlist.h"
#include "sfml_gui.h"

#define I16FROMBUF(b_, s_)  ( ((uint16_t)b_[(s_)+0]<<8) | ((uint16_t)b_[(s_)+1]<<0) )
#define I32FROMBUF(b_, s_)  ( ((uint32_t)b_[(s_)+0]<<24) | ((uint32_t)b_[(s_)+1]<<16) | ((uint32_t)b_[(s_)+2]<<8) | ((uint32_t)b_[(s_)+3]<<0) )

world_t world;


uint32_t robot_id = 0xacdcabba;
int cur_world_id = -1;

sf::Font arial;

//int screen_x = 1200;
//int screen_y = 900;

int screen_x = 1200;
int screen_y = 700; //700;

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
int bat_percentage;

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
		static char *argv[] = {"/bin/bash", "/home/hrst/rn1-client/do_map_sync.sh", NULL};
		if((execve(argv[0], (char **)argv , NULL)) == -1)
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

typedef struct search_unit_T
{
	xy_t loc;
	float g;
	float f;
	int direction;

	search_unit_T* parent;

	UT_hash_handle hh;
} search_unit_t;


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

void draw_route(sf::RenderWindow& win, route_unit_t **route)
{

	if(!route)
		return;

	route_unit_t *rt;
	DL_FOREACH(*route, rt)
	{
		if(rt->backmode)
			dev_draw_circle(win, rt->loc.x, rt->loc.y, 130,20,40,-123);
		else
			dev_draw_circle(win, rt->loc.x, rt->loc.y, 20,130,40,-123);

	}
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
				int alpha = (3*(int)page->units[x][y].num_seen) + (255/4);
				if(alpha > 255) alpha=255;
				if(page->units[x][y].result & UNIT_DBG)
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
				}
				else if(page->units[x][y].result & UNIT_ITEM)
				{
					pixels[4*(y*MAP_PAGE_W+x)+0] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+1] = 0;
					pixels[4*(y*MAP_PAGE_W+x)+2] = 255;
					pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
				}
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

				if(!(page->units[x][y].result & UNIT_INVISIBLE_WALL))
				{
					if(page->units[x][y].result & UNIT_3D_WALL)
					{
						pixels[4*(y*MAP_PAGE_W+x)+0] >>= 1;
						pixels[4*(y*MAP_PAGE_W+x)+1] >>= 0;
						pixels[4*(y*MAP_PAGE_W+x)+2] >>= 1;
						int a = pixels[4*(y*MAP_PAGE_W+x)+3]<<1; if(a>255) a=255;
						pixels[4*(y*MAP_PAGE_W+x)+3] = a;
					}
					else if(page->units[x][y].result & UNIT_DROP)
					{
						pixels[4*(y*MAP_PAGE_W+x)+0] >>= 0;
						pixels[4*(y*MAP_PAGE_W+x)+1] >>= 1;
						pixels[4*(y*MAP_PAGE_W+x)+2] >>= 0;
						int a = pixels[4*(y*MAP_PAGE_W+x)+3]<<1; if(a>255) a=255;
						pixels[4*(y*MAP_PAGE_W+x)+3] = a;
					}
					else if(page->units[x][y].result & UNIT_ITEM)
					{
						pixels[4*(y*MAP_PAGE_W+x)+0] >>= 0;
						pixels[4*(y*MAP_PAGE_W+x)+1] >>= 0;
						pixels[4*(y*MAP_PAGE_W+x)+2] >>= 2;
						int a = pixels[4*(y*MAP_PAGE_W+x)+3]<<1; if(a>255) a=255;
						pixels[4*(y*MAP_PAGE_W+x)+3] = a;
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
		t.setPosition(10,screen_y-170 + 15*i);
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
	t.setPosition(screen_x-180,screen_y-30);
	win.draw(t);

	if(charging)
	{
		t.setString("charging");
		t.setCharacterSize(16);
		t.setFillColor(sf::Color(200,110,0));
		t.setPosition(screen_x-130,screen_y-50);
		win.draw(t);
	}

	if(charge_finished)
	{
		t.setString("charge finished");
		t.setCharacterSize(16);
		t.setFillColor(sf::Color(10,200,50));
		t.setPosition(screen_x-160,screen_y-50);
		win.draw(t);
	}
}

void draw_texts(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[256];
	t.setFont(arial);

	const int bot_box_xs = 400;
	const int bot_box_ys = 63;
	sf::RectangleShape rect(sf::Vector2f( bot_box_xs, bot_box_ys));
	rect.setPosition(screen_x/2 - bot_box_xs/2, screen_y-bot_box_ys-10);
	rect.setFillColor(sf::Color(255,255,255,160));
	win.draw(rect);


	sprintf(buf, "robot: x=%d  y=%d  mm  (ang=%.1f deg)", (int)cur_x, (int)cur_y, cur_angle);
	t.setString(buf);
	t.setCharacterSize(17);
	t.setFillColor(sf::Color(0,0,0,160));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-51);
	win.draw(t);

	sprintf(buf, "cursor: x=%d  y=%d  mm", (int)click_x, (int)click_y);
	t.setString(buf);
	t.setCharacterSize(14);
	t.setFillColor(sf::Color(0,0,0, 120));
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-30);
	win.draw(t);

	if(rsync_running)
		sprintf(buf, "Syncing maps...");
	else
		sprintf(buf, "%s", click_mode_names[click_mode]);
	t.setString(buf);
	t.setCharacterSize(17);
	t.setFillColor(sf::Color(0,0,0,130));
	t.setPosition(screen_x/2-bot_box_xs/2+11,screen_y-72);
	win.draw(t);
	t.setFillColor(rsync_running?sf::Color(0,190,20,200):click_mode_colors[click_mode]);
	t.setPosition(screen_x/2-bot_box_xs/2+10,screen_y-73);
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

	if(dest_type > -1 && dest_type < 8)
	{
		const float robot_mark_radius = 200.0;
		const float robot_mark_radius2 = 40.0;
		sf::CircleShape circ(robot_mark_radius/mm_per_pixel);
		circ.setOrigin(robot_mark_radius/(mm_per_pixel), robot_mark_radius/(mm_per_pixel));
		circ.setFillColor(click_mode_colors[dest_type]);
		circ.setOutlineThickness(1.0);
		circ.setOutlineColor(sf::Color(0,0,0,100));
		circ.setPosition((dest_x+origin_x)/mm_per_pixel,(dest_y+origin_y)/mm_per_pixel);
		win.draw(circ);
		sf::CircleShape circ2(robot_mark_radius2/mm_per_pixel);
		circ2.setOrigin(robot_mark_radius2/(mm_per_pixel), robot_mark_radius2/(mm_per_pixel));
		circ2.setFillColor(click_mode_colors[dest_type]);
		circ2.setOutlineThickness(1.0);
		circ2.setOutlineColor(sf::Color(0,0,0,150));
		circ2.setPosition((dest_x+origin_x)/mm_per_pixel,(dest_y+origin_y)/mm_per_pixel);
		win.draw(circ2);

		if(dest_type == MODE_ADDCONSTRAINT || dest_type == MODE_REMCONSTRAINT)
			dest_type = -1;
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
	int8_t data[100*100];
} client_tof3d_hmap_t;

client_lidar_scan_t lidar;
#define SONAR_POINTS 6
sonar_point_t sonar[SONAR_POINTS];
static int sonar_wr = 0;


client_tof3d_hmap_t hmap;


#define HMAP_ALPHA 160
const sf::Color hmap_colors[8] = {
/*-2*/ sf::Color(200,   0, 255, HMAP_ALPHA),
/*-1*/ sf::Color(  0,   0, 255, HMAP_ALPHA),
/* 0*/ sf::Color(  0,   0,   0, HMAP_ALPHA/3),
/*+1*/ sf::Color(255, 255, 255, HMAP_ALPHA/2),
/*+2*/ sf::Color(  0, 200, 200, HMAP_ALPHA),
/*+3*/ sf::Color(  0, 200,   0, HMAP_ALPHA),
/*+4*/ sf::Color(100, 200,   0, HMAP_ALPHA),
/*+5*/ sf::Color(200, 200,   0, HMAP_ALPHA)
};


void draw_tof3d_hmap(sf::RenderWindow& win, client_tof3d_hmap_t* hm)
{
	for(int sy=0; sy < hm->ysamples; sy++)
	{
		for(int sx=0; sx < hm->xsamples; sx++)
		{
			int8_t val = hm->data[sy*hm->xsamples+sx];
			if(val == 0) continue;
			if(val < -2 || val > 5)
			{
				printf("draw_tof3d_hmap() invalid val %d at (%d, %d)\n", val, sx, sy);
				continue;
			}

			sf::RectangleShape rect(sf::Vector2f((float)hm->unit_size/mm_per_pixel/1.5,(float)hm->unit_size/mm_per_pixel/1.5));
			rect.setOrigin((float)hm->unit_size/mm_per_pixel/3.0,(float)hm->unit_size/mm_per_pixel/3.0);
			float x = sx*hm->unit_size;
			float y = (sy-hm->ysamples/2)*hm->unit_size;

			float ang = hm->robot_pos.ang/-360.0*2.0*M_PI;
			float rotax = x*cos(ang) + y*sin(ang) + hm->robot_pos.x;
			float rotay = -1*x*sin(ang) + y*cos(ang) + hm->robot_pos.y;

			rect.setPosition((rotax + origin_x)/mm_per_pixel,
					 (rotay + origin_y)/mm_per_pixel);
			rect.setFillColor(hmap_colors[val+2]);
			win.draw(rect);

/*
			float x = sx*hm->unit_size;
			float y = (sy-hm->ysamples/2)*hm->unit_size;

			float ang = hm->robot_pos.ang/-360.0*2.0*M_PI;
			float rotax = x*cos(ang) + y*sin(ang) + hm->robot_pos.x;
			float rotay = -1*x*sin(ang) + y*cos(ang) + hm->robot_pos.y;

			
			sf::Text t;
			char tbuf[16];
			if(val==-100)
				sprintf(tbuf, "  -");
			else
				sprintf(tbuf, "%d", (int32_t)val*2);
			t.setFont(arial);
			t.setFillColor(sf::Color(0,0,0,255));
			t.setString(tbuf);
			t.setCharacterSize(12);
			t.setPosition((rotax + origin_x)/mm_per_pixel,
					 (rotay + origin_y)/mm_per_pixel);
			win.draw(t);
*/
		}
	}
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
		if(c < -2) c = -2;
		else if(c > 5) c = 5;
		rect.setFillColor(hmap_colors[c+2]);
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

void go_charge_msg(uint8_t params)
{
	uint8_t test[4] = {57, 0, 1, params};

	if(tcpsock.send(test, 4) != sf::Socket::Done)
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


	if(online)
	{
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

	int but_localize    = gui.add_button(screen_x-170, 50 + 0*35, 140, 25, "     Localize (init)", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);
	int but_autonomous  = gui.add_button(screen_x-170, 50 + 1*35, 140, 25, "      Autonomous", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);
	int but_takecontrol = gui.add_button(screen_x-170, 50 + 2*35, 140, 25, "      Take control", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);
	int but_stop        = gui.add_button(screen_x-170, 50 + 3*35, 65, 25, "Stop", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);
	int but_free        = gui.add_button(screen_x-170+75, 50 + 3*35, 65, 25, "Free", DEF_BUT_COL, DEF_BUT_FONT_SIZE+2, -1, DEF_BUT_COL_PRESSED, false);

	int but_route     = gui.add_button(screen_x-170, 60 + 4*35, 100, 25, "Route", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_ROUTE, DEF_BUT_COL_PRESSED, false);
	int but_manu_fwd  = gui.add_button(screen_x-170, 60 + 5*35, 100, 25, "Manual fwd", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_MANUAL, DEF_BUT_COL_PRESSED, false);
	int but_manu_back = gui.add_button(screen_x-170+110, 60 + 5*35, 30, 25, "rev", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_force_fwd = gui.add_button(screen_x-170, 60 + 6*35, 100, 25, "Force fwd", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_FORCE, DEF_BUT_COL_PRESSED, false);
	int but_force_back= gui.add_button(screen_x-170+110, 60 + 6*35, 30, 25, "rev", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_pose      = gui.add_button(screen_x-170, 60 + 7*35, 100, 25, "Rotate pose", DEF_BUT_COL, DEF_BUT_FONT_SIZE, SYM_POSE, DEF_BUT_COL_PRESSED, false);

	int but_findcharger = gui.add_button(screen_x-170, 70 + 8*35, 140, 25, "  Find charger", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_worldminus  = gui.add_button(screen_x-170, 70 + 9*35, 25, 25, " -", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_worldplus  = gui.add_button(screen_x-170+115, 70 + 9*35, 25, 25, " +", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);

	int but_addconstraint  = gui.add_button(screen_x-170, 70 + 10*35, 60, 25, "ADD", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);
	int but_remconstraint  = gui.add_button(screen_x-170+65, 70 + 10*35, 60, 25, "REM", DEF_BUT_COL, DEF_BUT_FONT_SIZE, -1, DEF_BUT_COL_PRESSED, false);


	bool right_click_on = false;
	bool left_click_on = false;
	double prev_click_x = 0.0, prev_click_y = 0.0;

	while(win.isOpen())
	{
		int gui_box_xs = 170;
		int gui_box_ys = screen_y-65;
		int gui_box_x = screen_x-185;
		int gui_box_y = 15;

		gui.buttons[but_route]->pressed =      (click_mode==MODE_ROUTE);
		gui.buttons[but_manu_fwd]->pressed =   (click_mode==MODE_MANUAL_FWD);
		gui.buttons[but_manu_back]->pressed =  (click_mode==MODE_MANUAL_BACK);
		gui.buttons[but_force_fwd]->pressed =  (click_mode==MODE_FORCE_FWD);
		gui.buttons[but_force_back]->pressed = (click_mode==MODE_FORCE_BACK);
		gui.buttons[but_pose]->pressed =       (click_mode==MODE_POSE);
		gui.buttons[but_addconstraint]->pressed = (click_mode==MODE_ADDCONSTRAINT);
		gui.buttons[but_remconstraint]->pressed = (click_mode==MODE_REMCONSTRAINT);

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
				int but_start_x = screen_x-170;
				gui.buttons[but_route]->x = but_start_x;
				gui.buttons[but_manu_fwd]->x = but_start_x;
				gui.buttons[but_manu_back]->x = but_start_x+110;
				gui.buttons[but_force_fwd]->x = but_start_x;
				gui.buttons[but_force_back]->x = but_start_x+110;
				gui.buttons[but_pose]->x = but_start_x;
				gui.buttons[but_localize]->x = but_start_x;
				gui.buttons[but_autonomous]->x = but_start_x;
				gui.buttons[but_takecontrol]->x = but_start_x;
				gui.buttons[but_stop]->x = but_start_x;
				gui.buttons[but_free]->x = but_start_x+75;
				gui.buttons[but_findcharger]->x = but_start_x;
				gui.buttons[but_worldminus]->x = but_start_x;
				gui.buttons[but_worldplus]->x = but_start_x+115;
				gui.buttons[but_addconstraint]->x = but_start_x;
				gui.buttons[but_remconstraint]->x = but_start_x+65;

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

				if(len > 10000) len=10000;
				uint8_t rxbuf[16384];

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
						//printf("bat status %d %d %f %d\n", charging, charge_finished, bat_voltage, bat_percentage);
					}
					break;

					case 135: // Route info
					{
						clear_route(&some_route);
						int n_elems = len/9;

						route_start_x = cur_x;
						route_start_y = cur_y;

						for(int i = 0; i < n_elems; i++)
						{
							route_unit_t* point = malloc(sizeof(route_unit_t));
							point->backmode = rxbuf[i*9+0];
							point->loc.x = (int32_t)I32FROMBUF(rxbuf,i*9+1);
							point->loc.y = (int32_t)I32FROMBUF(rxbuf,i*9+5);
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
						hmap.xsamples = rxbuf[0];
						hmap.ysamples = rxbuf[1];

						if(hmap.xsamples < 1 || hmap.xsamples > 99 || hmap.ysamples < 1 || hmap.ysamples > 99)
						{
							printf("Invalid 3D TOF HMAP xsamples * ysamples (%d * %d)\n", hmap.xsamples, hmap.ysamples);
							break;
						}
						hmap.robot_pos.ang = ((double)I16FROMBUF(rxbuf, 2))/65536.0 * 360.0;
						hmap.robot_pos.x = (int32_t)I32FROMBUF(rxbuf,4);
						hmap.robot_pos.y = (int32_t)I32FROMBUF(rxbuf,8);
						hmap.unit_size = rxbuf[12];

//						printf("Got %d x %d hmap at %d, %d, %d\n", hmap.xsamples, hmap.ysamples, hmap.robot_pos.ang, hmap.robot_pos.x, hmap.robot_pos.y);
						memcpy(hmap.data, &rxbuf[13], hmap.xsamples*hmap.ysamples);
					}
					break;

					case 139: // info state
					{
						cur_info_state = (info_state_t)rxbuf[0];
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

				if(but == but_worldplus)
				{
					gui.buttons[but_worldplus]->pressed = true;
					cur_world_id = -1;
				}
				else
				{
					if(gui.buttons[but_worldplus]->pressed)
					{
						gui.buttons[but_worldplus]->pressed = false;
						mode_msg(9);
					}
				}

				if(but == but_worldminus)
				{
					gui.buttons[but_worldminus]->pressed = true;
					cur_world_id = -1;
				}
				else
				{
					if(gui.buttons[but_worldminus]->pressed)
					{
						gui.buttons[but_worldminus]->pressed = false;
						mode_msg(10);
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

				if(but == but_autonomous)
				{
					gui.buttons[but_autonomous]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_autonomous]->pressed)
					{
						gui.buttons[but_autonomous]->pressed = false;
						mode_msg(2);
					}
				}

				if(but == but_takecontrol)
				{
					gui.buttons[but_takecontrol]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_takecontrol]->pressed)
					{
						mode_msg(1);
						gui.buttons[but_takecontrol]->pressed = false;
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

				if(but == but_free)
				{
					gui.buttons[but_free]->pressed = true;
				}
				else
				{
					if(gui.buttons[but_free]->pressed)
					{
						mode_msg(5);
						gui.buttons[but_free]->pressed = false;
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
								}

							} break;

							case MODE_ADDCONSTRAINT: {
								int x = click_x; int y = click_y;
								uint8_t test[11] = {60, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(tcpsock.send(test, 11) != sf::Socket::Done)
								{
									printf("Send error\n");
								}
							} break;

							case MODE_REMCONSTRAINT: {
								int x = click_x; int y = click_y;
								uint8_t test[11] = {61, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
									(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};

								if(tcpsock.send(test, 11) != sf::Socket::Done)
								{
									printf("Send error\n");
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
				mode_msg(3);
				f_pressed[6] = true;
			}} else f_pressed[6] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F7)) { if(!f_pressed[7]) 
			{
				mode_msg(2);
				f_pressed[7] = true;
			}} else f_pressed[7] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F8)) { if(!f_pressed[8]) 
			{
				mode_msg(1);
				f_pressed[8] = true;
			}} else f_pressed[8] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F9)) { if(!f_pressed[9]) 
			{
				go_charge_msg(0);
				f_pressed[9] = true;
			}} else f_pressed[9] = false;
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F10)) { if(!f_pressed[10]) 
			{
				mode_msg(8);
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

		win.clear(sf::Color(230,230,230));

		draw_map(win);

		draw_robot(win);

		draw_lidar(win, &lidar);
		draw_sonar(win);
		draw_tof3d_hmap(win, &hmap);

		draw_hwdbg(win);
		draw_bat_status(win);

		draw_route_mm(win, &some_route);

		draw_texts(win);
		sf::RectangleShape rect(sf::Vector2f( gui_box_xs, gui_box_ys));
		rect.setPosition(gui_box_x, gui_box_y);
		rect.setFillColor(sf::Color(255,255,255,160));
		win.draw(rect);
		gui.draw_all_buttons();

		sf::Text t;
		char tbuf[256];
		t.setFont(arial);

		static int fx=0;
		if(cur_world_id == -1)
		{
			fx++;
			if(fx < 15)
				sprintf(tbuf, ".  .  .");
			else if(fx < 30)
				sprintf(tbuf, " .  .  .");
			else if(fx < 45)
			{
				sprintf(tbuf, "  .  .  .");
			}
			else
			{
				sprintf(tbuf, "  .  .  .");
				fx = 0;
			}
			t.setFillColor(sf::Color(220,0,0,255));
		}
		else
		{
			fx = 0;
			sprintf(tbuf, "W %d", cur_world_id);
			t.setFillColor(sf::Color(0,200,20,255));
		}
		t.setString(tbuf);
		t.setCharacterSize(17);
		t.setPosition(screen_x-170+35, 70 + 9*35);
		win.draw(t);


		if(cur_info_state >= 0 && cur_info_state < NUM_DECORS)
		{
			sf::Sprite decor_sprite;
			decor_sprite.setTexture(decors[cur_info_state]);
			decor_sprite.setPosition(screen_x-180, (cur_info_state==INFO_STATE_DAIJUING)?(screen_y-240):(screen_y-220));
			win.draw(decor_sprite);
		}

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
	return 0;
}
