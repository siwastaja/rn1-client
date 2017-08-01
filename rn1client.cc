#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>
#include <sys/select.h>
#include <sys/ioctl.h>
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

#define I16FROMBUF(b_, s_)  ( ((uint16_t)b_[(s_)+0]<<8) | ((uint16_t)b_[(s_)+1]<<0) )
#define I32FROMBUF(b_, s_)  ( ((uint32_t)b_[(s_)+0]<<24) | ((uint32_t)b_[(s_)+1]<<16) | ((uint32_t)b_[(s_)+2]<<8) | ((uint32_t)b_[(s_)+3]<<0) )

world_t world;


uint32_t robot_id = 0xacdcabba;

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

bool dest_clicked;
double dest_x, dest_y, dest_angle, dest_backmode;

double robot_xs = 480.0;
double robot_ys = 524.0;
double lidar_xoffs = 120.0;
double lidar_yoffs = 0.0;

int charging;
int charge_finished;
float bat_voltage;
int bat_percentage;


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

//#define WALL_LEVEL(i) (255*((int)(i).num_obstacles)/((int)(i).num_seen))
#define WALL_LEVEL(i) ((int)(i).num_obstacles*4)

void draw_page(sf::RenderWindow& win, map_page_t* page, int startx, int starty)
{
	if(!page)
		return;

	static uint8_t pixels[MAP_PAGE_W*MAP_PAGE_W*4];

	for(int x = 0; x < MAP_PAGE_W; x++)
	{
		for(int y = 0; y < MAP_PAGE_W; y++)
		{

			int alpha = (30*(int)page->units[x][y].num_seen)/3 + (255/3);
			if(alpha > 255) alpha=255;
			alpha /= 2;
			if(page->units[x][y].result & UNIT_DBG)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 255;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 255;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+3] = 255;
			}
			else if(page->units[x][y].result & UNIT_3D_WALL)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 100;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+3] = 255; //alpha;
			}
			else if(page->units[x][y].result & UNIT_DROP)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 100;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 100;
				pixels[4*(y*MAP_PAGE_W+x)+3] = 255; //alpha;
			}
			else if(page->units[x][y].result & UNIT_ITEM)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 100;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 100;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+3] = 255; //alpha;
			}
			else if(page->units[x][y].num_obstacles)
			{
				int lvl = WALL_LEVEL(page->units[x][y]);
				if(lvl > 140) lvl = 140;
				int color = 140 - lvl;
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
				pixels[4*(y*MAP_PAGE_W+x)+1] = 240 - sqrt(page->units[x][y].num_visited*200);
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
	t.setCharacterSize(12);
	t.setColor(sf::Color(0,0,0));
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
	t.setColor(sf::Color(r,g,0));
	t.setPosition(screen_x-180,screen_y-40);
	win.draw(t);

	if(charging)
	{
		t.setString("charging");
		t.setCharacterSize(16);
		t.setColor(sf::Color(200,110,0));
		t.setPosition(screen_x-180,screen_y-80);
		win.draw(t);
	}

	if(charge_finished)
	{
		t.setString("charge finished");
		t.setCharacterSize(16);
		t.setColor(sf::Color(60,255,60));
		t.setPosition(screen_x-220,screen_y-60);
		win.draw(t);
	}
}

void draw_texts(sf::RenderWindow& win)
{
	sf::Text t;
	char buf[256];
	t.setFont(arial);

	sprintf(buf, "x=%d  y=%d  mm", (int)click_x, (int)click_y);
	t.setString(buf);
	t.setCharacterSize(18);
	t.setColor(sf::Color(0,0,0));
	t.setPosition(screen_x/2-50,screen_y-30);
	win.draw(t);

	if(show_dbgpoint)
	{

		sf::CircleShape circ(120.0/mm_per_pixel);
		circ.setOrigin(120.0/mm_per_pixel, 120.0/mm_per_pixel);
		circ.setFillColor(sf::Color(dbgpoint_r,dbgpoint_g,dbgpoint_b, 120));
		circ.setOutlineColor(sf::Color(0,0,0,150));

		circ.setPosition((dbgpoint_x+origin_x)/mm_per_pixel,(dbgpoint_y+origin_y)/mm_per_pixel);
		win.draw(circ);
	}

	for(int i = 0; i < num_pers_dbgpoints; i++)
	{

		sf::CircleShape circ(120.0/mm_per_pixel);
		circ.setOrigin(120.0/mm_per_pixel, 120.0/mm_per_pixel);
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
	sf::ConvexShape r(5);
	r.setPoint(0, sf::Vector2f(0,0));
	r.setPoint(1, sf::Vector2f(0,robot_ys/mm_per_pixel));
	r.setPoint(2, sf::Vector2f(robot_xs/mm_per_pixel,robot_ys/mm_per_pixel));
	r.setPoint(3, sf::Vector2f(1.3*robot_xs/mm_per_pixel,0.5*robot_ys/mm_per_pixel));
	r.setPoint(4, sf::Vector2f(robot_xs/mm_per_pixel,0));

	r.setOrigin((0.5*robot_xs+lidar_xoffs)/mm_per_pixel,(0.5*robot_ys+lidar_yoffs)/mm_per_pixel);

	r.setFillColor(sf::Color(200,90,50,160));

	r.setRotation(cur_angle);
	r.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

	win.draw(r);

	if(dest_clicked)
	{
		if(dest_backmode == 1)
			r.setFillColor(sf::Color(255,0,0,128));
		else if(dest_backmode == 2)
			r.setFillColor(sf::Color(255,255,0,128));
		else
			r.setFillColor(sf::Color(0,255,0,128));

		r.setRotation(cur_angle+dest_angle);
		r.setPosition((origin_x+dest_x)/mm_per_pixel,(origin_y+dest_y)/mm_per_pixel);
		win.draw(r);
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
typedef struct
{
	pos_t robot_pos;
	point_t scan[180];
} client_lidar_scan_t;

typedef struct
{
	point_t scan[3];
} client_sonar_scan_t;

typedef struct
{
	pos_t robot_pos;
	int xsamples;
	int ysamples;
	int unit_size;
	int8_t data[100*100];
} client_tof3d_hmap_t;

client_lidar_scan_t lidar;
client_sonar_scan_t sonar;
client_tof3d_hmap_t hmap;

#define HMAP_ALPHA 200
const sf::Color hmap_colors[7] = {
/*-2*/ sf::Color(200,   0, 255, HMAP_ALPHA),
/*-1*/ sf::Color(  0,   0, 255, HMAP_ALPHA),
/* 0*/ sf::Color(255, 255, 255, HMAP_ALPHA/3),
/*+1*/ sf::Color(  0, 200, 200, HMAP_ALPHA),
/*+2*/ sf::Color(  0, 200,   0, HMAP_ALPHA),
/*+3*/ sf::Color(100, 200,   0, HMAP_ALPHA),
/*+4*/ sf::Color(200, 200,   0, HMAP_ALPHA)
};


void draw_tof3d_hmap(sf::RenderWindow& win, client_tof3d_hmap_t* hm)
{
	for(int sy=0; sy < hm->ysamples; sy++)
	{
		for(int sx=0; sx < hm->xsamples; sx++)
		{
			int8_t val = hm->data[sy*hm->xsamples+sx];
			if(val == 0) continue;
			if(val < -2 || val > 4)
			{
				printf("draw_tof3d_hmap() invalid val %d at (%d, %d)\n", val, sx, sy);
				continue;
			}

			sf::RectangleShape rect(sf::Vector2f((float)hm->unit_size/mm_per_pixel/1.5,(float)hm->unit_size/mm_per_pixel/1.5));
			rect.setOrigin((float)hm->unit_size/mm_per_pixel/3.0,(float)hm->unit_size/mm_per_pixel/3.0);
			float x = /*hm->robot_pos.x + */sx*hm->unit_size;
			float y = /*hm->robot_pos.y + */(sy-hm->ysamples/2)*hm->unit_size;

			float ang = hm->robot_pos.ang/-360.0*2.0*M_PI;
			float rotax = x*cos(ang) + y*sin(ang) + hm->robot_pos.x;
			float rotay = -1*x*sin(ang) + y*cos(ang) + hm->robot_pos.y;

			rect.setPosition((rotax + origin_x)/mm_per_pixel,
					 (rotay + origin_y)/mm_per_pixel);
			rect.setFillColor(hmap_colors[val+2]);
			win.draw(rect);
			
		}
	}
}

void draw_lidar(sf::RenderWindow& win, client_lidar_scan_t* lid)
{
	for(int i=0; i < 180; i++)
	{
		if(!lid->scan[i].valid) continue;

		sf::RectangleShape rect(sf::Vector2f(3,3));
		rect.setOrigin(1.5,1.5);
		rect.setPosition((lid->scan[i].x+origin_x)/mm_per_pixel, (lid->scan[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(255, 0, 0, 100));
		win.draw(rect);
	}
}

void draw_sonar(sf::RenderWindow& win, client_sonar_scan_t* son)
{
	for(int i=0; i < 3; i++)
	{
		if(!son->scan[i].valid) continue;

		sf::RectangleShape rect(sf::Vector2f(6,6));
		rect.setOrigin(3,3);
		rect.setPosition((son->scan[i].x+origin_x)/mm_per_pixel, (son->scan[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(0, 0, 70, 90));
		win.draw(rect);
	}
}


sf::IpAddress serv_ip;
unsigned short serv_port;

sf::TcpSocket tcpsock;

int main(int argc, char** argv)
{
	bool f1_pressed = false;
	bool f5_pressed = false;
	bool f12_pressed = false;
	bool return_pressed = false;
	bool num3_pressed = false;
	bool num4_pressed = false;
	bool num5_pressed = false;
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
	sf::RenderWindow win(sf::VideoMode(screen_x,screen_y), "RN#1 Client", sf::Style::Default, sets);
	win.setFramerateLimit(30);

	bool right_click_on = false;
	double prev_click_x = 0.0, prev_click_y = 0.0;

	while(win.isOpen())
	{
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

			//	printf("msgid=%d len=%d\n", msgid, len);

				if(len > 2000) len=2000;
				uint8_t rxbuf[2048];

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

						for(int i=0; i<180; i++)
						{
							int x = (int8_t)rxbuf[10+2*i];
							int y = (int8_t)rxbuf[10+2*i+1];
							if(x==0 && y==0)
							{
								lidar.scan[i].valid = 0;
								continue;
							}
							lidar.scan[i].valid = 1;
							lidar.scan[i].x = x*40 + mid_x;
							lidar.scan[i].y = y*40 + mid_y;
						}
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

					case 133: // sonar points
					{
						for(int i=0; i<3; i++)
						{
							if(rxbuf[0] & (1<<i))
							{
								sonar.scan[i].valid = 1;
								sonar.scan[i].x = (int32_t)I32FROMBUF(rxbuf,1+8*i);
								sonar.scan[i].y = (int32_t)I32FROMBUF(rxbuf,5+8*i);
							}
							else
							{
								sonar.scan[i].valid = 0;
							}
						}
					}
					break;

					case 134: // Battery status
					{
						charging = rxbuf[0]&1;
						charge_finished = rxbuf[0]&2;
						bat_voltage = (float)(((int)rxbuf[1]<<8) | rxbuf[2])/1000.0;
						bat_percentage = rxbuf[3];
					}
					break;

					case 135: // Route info
					{
						clear_route(&some_route);
						int n_elems = len/9;

						printf("got %d elements\n", n_elems);

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

						memcpy(hmap.data, &rxbuf[13], hmap.xsamples*hmap.ysamples);
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
			click_x = (localPosition.x * mm_per_pixel) - origin_x;
			click_y = (localPosition.y * mm_per_pixel) - origin_y;

			if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
			{
				dest_clicked = true;
				dest_x = click_x; dest_y = click_y;
				dest_backmode = 2;
			}


			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num5))
			{
				if(!num5_pressed)
				{
					if(p_cur_step != NULL)
					{
						dest_clicked = true;

						int mm_x, mm_y;
						mm_from_unit_coords(p_cur_step->loc.x, p_cur_step->loc.y, &mm_x, &mm_y);

						dest_x = mm_x;
						dest_y = mm_y;
						dest_backmode = p_cur_step->backmode;
						if(p_cur_step->next != NULL)
							p_cur_step = p_cur_step->next;
					}
					num5_pressed = true;
				}
			}
			else
				num5_pressed = false;

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num4))
			{
				if(!num4_pressed)
				{
					if(p_cur_step != NULL)
					{
						dest_clicked = true;

						int mm_x, mm_y;
						mm_from_unit_coords(p_cur_step->loc.x, p_cur_step->loc.y, &mm_x, &mm_y);

						dest_x = mm_x;
						dest_y = mm_y;
						dest_backmode = p_cur_step->backmode;
						if(p_cur_step->prev != NULL)
							p_cur_step = p_cur_step->prev;
					}
					num4_pressed = true;
				}
			}
			else
				num4_pressed = false;


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

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F5))
			{
				if(!f5_pressed)
				{
					load_all_pages_on_disk(&world);
					f5_pressed = true;
				}
			}
			else
				f5_pressed = false;
			

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::F1))
			{
				if(!f1_pressed)
				{
					uint8_t test[11] = {55, 0, 8,   0,0,1,10,  255,255,254,245};
					if(tcpsock.send(test, 11) != sf::Socket::Done)
					{
						printf("Send error\n");
					}
					f1_pressed = true;
				}
			}
			else
				f1_pressed = false;


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
					if(dest_clicked)
					{
						clear_route(&some_route);

						int x = dest_x; int y = dest_y;

						uint8_t test[12] = {56 /*ROUTE*/, 0, 9,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
							(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff, dest_backmode};

						if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift))
							test[0] = 55; // DEST

						if(tcpsock.send(test, 12) != sf::Socket::Done)
						{
							printf("Send error\n");
						}

						dest_clicked = false;
					}
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
		draw_sonar(win, &sonar);
		draw_tof3d_hmap(win, &hmap);

		draw_hwdbg(win);
		draw_bat_status(win);

		draw_route_mm(win, &some_route);

		draw_texts(win);

		win.display();

		usleep(100);

	}
	return 0;
}
