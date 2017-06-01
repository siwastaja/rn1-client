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

#define I16FROMBUF(b_, s_)  ( ((uint16_t)b_[(s_)+0]<<8) | ((uint16_t)b_[(s_)+1]<<0) )
#define I32FROMBUF(b_, s_)  ( ((uint32_t)b_[(s_)+0]<<24) | ((uint32_t)b_[(s_)+1]<<16) | ((uint32_t)b_[(s_)+2]<<8) | ((uint32_t)b_[(s_)+3]<<0) )

world_t world;


uint32_t robot_id = 0xacdcabba;

sf::Font arial;

//int screen_x = 1200;
//int screen_y = 900;

int screen_x = 1200;
int screen_y = 1000; //700;

double origin_x = 0;
double origin_y = 0;

double cur_angle = 0.0;
double cur_x = 0.0;
double cur_y = 0.0;

bool dest_clicked;
double dest_x, dest_y, dest_angle;

double robot_xs = 480.0;
double robot_ys = 524.0;
double lidar_xoffs = 120.0;
double lidar_yoffs = 0.0;

int charging;
int charge_finished;
float bat_voltage;
int bat_percentage;


double dev_start_x;
double dev_start_y;
double dev_end_x;
double dev_end_y;

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


void draw_dev(sf::RenderWindow& win)
{
	sf::CircleShape circ(40.0/mm_per_pixel);
	circ.setOrigin(20.0/mm_per_pixel, 20.0/mm_per_pixel);
	circ.setFillColor(sf::Color(200,50,50));
	circ.setPosition((dev_start_x+origin_x)/mm_per_pixel,(dev_start_y+origin_y)/mm_per_pixel);
	win.draw(circ);

	circ.setFillColor(sf::Color(50,200,50));
	circ.setPosition((dev_end_x+origin_x)/mm_per_pixel,(dev_end_y+origin_y)/mm_per_pixel);
	win.draw(circ);
}

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

	search_unit_T* parent;

	UT_hash_handle hh;
} search_unit_t;

#define sq(x) ((x)*(x))
#define MAX_F 99999999999999999.9

void dev_draw_circle(sf::RenderWindow& win, int unit_x, int unit_y, int r, int g, int b)
{
	int x_mm, y_mm;
	mm_from_unit_coords(unit_x, unit_y, &x_mm, &y_mm);

	sf::CircleShape circ(14.0/mm_per_pixel);
	circ.setOrigin(7.0/mm_per_pixel, 7.0/mm_per_pixel);
	circ.setFillColor(sf::Color(r,g,b));
	circ.setPosition((x_mm+origin_x)/mm_per_pixel,(y_mm+origin_y)/mm_per_pixel);
	win.draw(circ);
	win.display();
}

void dev_search(sf::RenderWindow& win)
{
	win.setFramerateLimit(0);

	int s_x, s_y, e_x, e_y;
	unit_coords(dev_start_x, dev_start_y, &s_x, &s_y);
	unit_coords(dev_end_x, dev_end_y, &e_x, &e_y);

	printf("Start %d,%d,  end %d,%d\n", s_x, s_y, e_x, e_y);

	search_unit_t* closed_set = NULL;

	search_unit_t* open_set = NULL;

	search_unit_t* p_start = (search_unit_t*) malloc(sizeof(search_unit_t));
	memset(p_start, 0, sizeof(search_unit_t));

	p_start->loc.x = s_x;
	p_start->loc.y = s_y;
	p_start->parent = NULL;
	// g = 0
	p_start->f = sqrt((float)(sq(e_x-s_x) + sq(e_y-s_y)));

	HASH_ADD(hh, open_set, loc,sizeof(xy_t), p_start);
	dev_draw_circle(win, p_start->loc.x, p_start->loc.y, 200,0,0);

	while(HASH_CNT(hh, open_set) > 0)
	{
		// Find the lowest f score from open_set.
		search_unit_t* p_cur;
		float lowest_f = 2.0*MAX_F;
		for(search_unit_t* p_iter = open_set; p_iter != NULL; p_iter=p_iter->hh.next)
		{
			if(p_iter->f < lowest_f)
			{
				lowest_f = p_iter->f;
				p_cur = p_iter;
			}
		}

		if(p_cur->loc.x == e_x && p_cur->loc.y == e_y)
		{
			// solution found.

			// Reconstruct the path

			search_unit_t* p_recon = p_cur;
			while( (p_recon = p_recon->parent) )
			{
				dev_draw_circle(win, p_recon->loc.x, p_recon->loc.y, 255,255,255);
			}

			// Free all memory.
			search_unit_t *p_del, *p_tmp;
			HASH_ITER(hh, closed_set, p_del, p_tmp)
			{
				HASH_DEL(closed_set, p_del);
				free(p_del);
			}
			HASH_ITER(hh, open_set, p_del, p_tmp)
			{
				HASH_DEL(open_set, p_del);
				free(p_del);
			}

			sleep(10);
			return;
		}

		// move from open to closed:
		HASH_DELETE(hh, open_set, p_cur);
		HASH_ADD(hh, closed_set, loc,sizeof(xy_t), p_cur);
		dev_draw_circle(win, p_cur->loc.x, p_cur->loc.y, 0,200,0);


		// For each neighbor
		for(int xx=-1; xx<=1; xx++)
		{
			for(int yy=-1; yy<=1; yy++)
			{
				search_unit_t* found;
				float new_g;
				if(xx == 0 && yy == 0) continue;

				search_unit_t* p_neigh;
				xy_t neigh_loc = {p_cur->loc.x + xx, p_cur->loc.y + yy};

				// Check if it's out-of-allowed area here:

				int num_obstacles = 0;
				for(int chk_x=-6; chk_x<=6; chk_x++)
				{
					for(int chk_y=-6; chk_y<=6; chk_y++)
					{
						int pageidx_x, pageidx_y, pageoffs_x, pageoffs_y;
						page_coords_from_unit_coords(neigh_loc.x+chk_x, neigh_loc.y+chk_y, &pageidx_x, &pageidx_y, &pageoffs_x, &pageoffs_y);

						if(!world.pages[pageidx_x][pageidx_y]) // out of bounds (not allocated) - give up instantly
						{
							goto OUT_OF_BOUNDS;
						}

						if(world.pages[pageidx_x][pageidx_y]->units[pageoffs_x][pageoffs_y].result & UNIT_WALL)
						{
							num_obstacles++;
						}
					}
				}

				if(num_obstacles > 2)
					continue;

				HASH_FIND(hh, closed_set, &neigh_loc,sizeof(xy_t), found);
				if(found)
					continue; // ignore neighbor that's in closet_set.

				// gscore for the neigbor: distance from the start to neighbor
				// is current unit's g score plus distance to the neighbor.
				new_g = p_cur->g;
				if(xx == 0 || yy == 0)
					new_g += 1.0;
				else
					new_g += sqrt(2.0);

				HASH_FIND(hh, open_set, &neigh_loc,sizeof(xy_t), p_neigh);
				if(!p_neigh)
				{
					p_neigh = (search_unit_t*) malloc(sizeof(search_unit_t));
					memset(p_neigh, 0, sizeof(search_unit_t));
					p_neigh->loc.x = neigh_loc.x; p_neigh->loc.y = neigh_loc.y;
					HASH_ADD(hh, open_set, loc,sizeof(xy_t), p_neigh);
					dev_draw_circle(win, p_cur->loc.x, p_cur->loc.y, 200,0,0);

				}
				else /*found it*/ if(new_g >= p_neigh->g)
				{
					continue; // Neighbor was in open set, but it already has better score
				}

				p_neigh->parent = p_cur;
				p_neigh->g = new_g;
				p_neigh->f = new_g + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));

				OUT_OF_BOUNDS: ;

			}
		}		
	}

	search_unit_t *p_del, *p_tmp;
	HASH_ITER(hh, closed_set, p_del, p_tmp)
	{
		HASH_DELETE(hh, closed_set, p_del);
		free(p_del);
	}
	HASH_ITER(hh, open_set, p_del, p_tmp)
	{
		HASH_DELETE(hh, open_set, p_del);
		free(p_del);
	}
	
	return;
	// Failure.

}


void draw_page(sf::RenderWindow& win, map_page_t* page, int startx, int starty)
{
	if(!page)
		return;

	static uint8_t pixels[MAP_PAGE_W*MAP_PAGE_W*4];


/*
				sf::RectangleShape rect(sf::Vector2f(MAP_UNIT_W/mm_per_pixel,MAP_UNIT_W/mm_per_pixel));
				rect.setPosition((startx+x*MAP_UNIT_W)/mm_per_pixel, (starty+y*MAP_UNIT_W)/mm_per_pixel);
				if(page->units[x][y].result & UNIT_WALL)
					rect.setFillColor(sf::Color(60,20,0));
				else if(page->units[x][y].result & UNIT_ITEM)
					rect.setFillColor(sf::Color(0,100,200));
				else
					rect.setFillColor(sf::Color(255,240,190)); // mapped

				win.draw(rect);
*/

	for(int x = 0; x < MAP_PAGE_W; x++)
	{
		for(int y = 0; y < MAP_PAGE_W; y++)
		{

			int alpha = (30*(int)page->units[x][y].num_seen)/3 + (255/3);
			if(alpha > 255) alpha=255;
			if(page->units[x][y].result & UNIT_WALL)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 60;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 20;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+3] = alpha;
			}
			else if(page->units[x][y].result & UNIT_ITEM)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 0;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 100;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 200;
				pixels[4*(y*MAP_PAGE_W+x)+3] = alpha;
			}
			else if(page->units[x][y].result & UNIT_MAPPED)
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 255;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 240;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 190;
				pixels[4*(y*MAP_PAGE_W+x)+3] = alpha;
			}
			else
			{
				pixels[4*(y*MAP_PAGE_W+x)+0] = 200;
				pixels[4*(y*MAP_PAGE_W+x)+1] = 220;
				pixels[4*(y*MAP_PAGE_W+x)+2] = 240;
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

	r.setFillColor(sf::Color(180,100,70));

	r.setRotation(cur_angle);
	r.setPosition((cur_x+origin_x)/mm_per_pixel,(cur_y+origin_y)/mm_per_pixel);

	win.draw(r);

	if(dest_clicked)
	{
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

client_lidar_scan_t lidar;
client_sonar_scan_t sonar;

void draw_lidar(sf::RenderWindow& win, client_lidar_scan_t* lid)
{
	for(int i=0; i < 180; i++)
	{
		if(!lid->scan[i].valid) continue;

		sf::RectangleShape rect(sf::Vector2f(3,3));
		rect.setOrigin(1.5,1.5);
		rect.setPosition((lid->scan[i].x+origin_x)/mm_per_pixel, (lid->scan[i].y+origin_y)/mm_per_pixel);
		rect.setFillColor(sf::Color(0, 70, 0, 80));
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
	bool return_pressed = false;
	bool num3_pressed = false;
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

					default:
					break;
				}
			}
		}

		if(focus)
		{
			sf::Vector2i localPosition = sf::Mouse::getPosition(win);
			double click_x = (localPosition.x * mm_per_pixel) - origin_x;
			double click_y = (localPosition.y * mm_per_pixel) - origin_y;

			if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
			{
				dest_clicked = true;
				dest_x = click_x; dest_y = click_y;
			}

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

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num1))
			{
				dev_start_x = click_x; dev_start_y = click_y;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num2))
			{
				dev_end_x = click_x; dev_end_y = click_y;
			}

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Num3))
			{
				if(!num3_pressed)
				{
					dev_search(win);
					num3_pressed = true;
				}
			}
			else
				num3_pressed = false;



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
						int x = dest_x; int y = dest_y;

						uint8_t test[11] = {55, 0, 8,   (x>>24)&0xff,(x>>16)&0xff,(x>>8)&0xff,(x>>0)&0xff,
							(y>>24)&0xff, (y>>16)&0xff, (y>>8)&0xff, (y>>0)&0xff};
						if(tcpsock.send(test, 11) != sf::Socket::Done)
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

		win.clear(sf::Color(200,220,240));

		draw_map(win);

		draw_robot(win);

		draw_lidar(win, &lidar);
		draw_sonar(win, &sonar);

		draw_hwdbg(win);
		draw_bat_status(win);

		draw_dev(win);

		win.display();

		usleep(100);

	}
	return 0;
}
