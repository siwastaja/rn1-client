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
	int direction;

	search_unit_T* parent;

	UT_hash_handle hh;
} search_unit_t;


typedef struct route_unit_T
{
	xy_t loc;
	route_unit_T* prev;
	route_unit_T* next;
} route_unit_t;


#define sq(x) ((x)*(x))
#define MAX_F 99999999999999999.9

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
	win.display();


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

/*
int test_line_of_sight_blocked(int cur_x, int cur_y)
{
	int pageidx_x, pageidx_y, pageoffs_x, pageoffs_y;
	page_coords_from_unit_coords(cur_x, cur_y, &pageidx_x, &pageidx_y, &pageoffs_x, &pageoffs_y);

	if(!world.pages[pageidx_x][pageidx_y]) // out of bounds (not allocated) - give up instantly
	{
		printf("line of sight -- not allocated\n");
		return 1;
	}

	if(world.pages[pageidx_x][pageidx_y]->units[pageoffs_x][pageoffs_y].result & UNIT_WALL)
	{
//		printf("line of sight -- a wall\n");
		return 1;
	}

	return 0;
}

int line_of_sight(search_unit_t* p1, search_unit_t* p2)
{
	int sx = p1->loc.x;
	int sy = p1->loc.y;

	int dx = p2->loc.x - sx;
	int dy = p2->loc.y - sy;


	int cur_x, cur_y;

	if(abs(dx) >= abs(dy)) // Step in X direction
	{
		float dy_per_step = (float)dy/(float)dx;
		if(dx >= 0)
		{
			for(int ix = 0; ix < dx; ix++)
			{
				cur_y = sy + dy_per_step*(float)ix;
				cur_x = sx + ix;

				if(test_line_of_sight_blocked(cur_x, cur_y))
					return 0;
			}
		}
		else // dx < 0
		{
			for(int ix = 0; ix < -1*dx; ix++)
			{
				cur_y = sy - dy_per_step*(float)ix;
				cur_x = sx - ix;

				if(test_line_of_sight_blocked(cur_x, cur_y))
					return 0;
			}
		}

	}
	else // Step in Y direction
	{
		float dx_per_step = (float)dx/(float)dy;
		if(dy >= 0)
		{
			for(int iy = 0; iy < dy; iy++)
			{
				cur_x = sx + dx_per_step*(float)iy;
				cur_y = sy + iy;

				if(test_line_of_sight_blocked(cur_x, cur_y))
					return 0;
			}
		}
		else // dy < 0
		{
			for(int iy = 0; iy < -1*dy; iy++)
			{
				cur_x = sx - dx_per_step*(float)iy;
				cur_y = sy - iy;

				if(test_line_of_sight_blocked(cur_x, cur_y))
					return 0;
			}
		}

	}

	

	return 1;
}
*/

#define ROBOT_SHAPE_WINDOW 32
uint8_t robot_shapes[32][ROBOT_SHAPE_WINDOW][ROBOT_SHAPE_WINDOW];

int los_dbg;
void draw_map(sf::RenderWindow& win);

int check_hit(sf::RenderWindow& win, int x, int y, int direction)
{
	if(los_dbg)
	{
		win.clear(sf::Color(200,220,240));
		draw_map(win);
	}

	int num_obstacles = 0;
	for(int chk_x=0; chk_x<ROBOT_SHAPE_WINDOW; chk_x++)
	{
		for(int chk_y=0; chk_y<ROBOT_SHAPE_WINDOW; chk_y++)
		{
			int pageidx_x, pageidx_y, pageoffs_x, pageoffs_y;
			page_coords_from_unit_coords(x-ROBOT_SHAPE_WINDOW/2+chk_x, y-ROBOT_SHAPE_WINDOW/2+chk_y, &pageidx_x, &pageidx_y, &pageoffs_x, &pageoffs_y);

			if(!world.pages[pageidx_x][pageidx_y]) // out of bounds (not allocated) - give up instantly
			{
				return 1;
			}

			if(robot_shapes[direction][chk_x][chk_y])
			{
				if((world.pages[pageidx_x][pageidx_y]->units[pageoffs_x][pageoffs_y].result & UNIT_WALL))
					num_obstacles++;

				if(los_dbg)
				{
					dev_draw_circle(win, x-ROBOT_SHAPE_WINDOW/2+chk_x, y-ROBOT_SHAPE_WINDOW/2+chk_y, 255,70,255,-100);
				}
			}

		}
	}

	if(los_dbg)
	{

		win.display();
	}

	if(num_obstacles > 2)
		return 1;

	return 0;
}

int line_of_sight(sf::RenderWindow& win, xy_t p1, xy_t p2)
{
	int dx = p2.x - p1.x;
	int dy = p2.y - p1.y;

	const float step = 500.0/MAP_UNIT_W;

	float len = sqrt(sq(dx) + sq(dy));

	float pos = 0.0;
	int terminate = 0;

	float ang = atan2(dy, dx);
	if(ang < 0.0) ang += 2.0*M_PI;
	int dir = (ang/(2.0*M_PI) * 32.0)+0.5;


	printf("ang = %.4f  dir = %d \n", ang, dir);

	while(1)
	{
		int x = (cos(ang)*pos + (float)p1.x)+0.5;
		int y = (sin(ang)*pos + (float)p1.y)+0.5;

//		printf("check_hit(%d, %d, %d) = ", x, y, dir);

		if(check_hit(win, x, y, dir))
		{
//			printf("1 !\n");
			return 0;
		}
//		printf("0\n");

		if(terminate) break;
		pos += step;
		if(pos > len)
		{
			pos = len;
			terminate = 1;
		}
	}

	return 1;
}


//#define SetPixel(shape, x, y) {if((x)>=0 && (y)>=0 && (x)<ROBOT_SHAPE_WINDOW && (y)<ROBOT_SHAPE_WINDOW) robot_shapes[shape][(x)][(y)] = 1;}
#define SetPixel(shape, x, y) { robot_shapes[shape][(x)][(y)] = 1;}

// min X and max X for every horizontal line within the triangle
long ContourX[ROBOT_SHAPE_WINDOW][2];

#define ABS(x) ((x >= 0) ? x : -x)

// Scans a side of a triangle setting min X and max X in ContourX[][]
// (using the Bresenham's line drawing algorithm).
void ScanLine(long x1, long y1, long x2, long y2)
{
  long sx, sy, dx1, dy1, dx2, dy2, x, y, m, n, k, cnt;

  sx = x2 - x1;
  sy = y2 - y1;

  if (sx > 0) dx1 = 1;
  else if (sx < 0) dx1 = -1;
  else dx1 = 0;

  if (sy > 0) dy1 = 1;
  else if (sy < 0) dy1 = -1;
  else dy1 = 0;

  m = ABS(sx);
  n = ABS(sy);
  dx2 = dx1;
  dy2 = 0;

  if (m < n)
  {
    m = ABS(sy);
    n = ABS(sx);
    dx2 = 0;
    dy2 = dy1;
  }

  x = x1; y = y1;
  cnt = m + 1;
  k = n / 2;

  while (cnt--)
  {
    if ((y >= 0) && (y < ROBOT_SHAPE_WINDOW))
    {
      if (x < ContourX[y][0]) ContourX[y][0] = x;
      if (x > ContourX[y][1]) ContourX[y][1] = x;
    }

    k += n;
    if (k < m)
    {
      x += dx2;
      y += dy2;
    }
    else
    {
      k -= m;
      x += dx1;
      y += dy1;
    }
  }
}

void draw_triangle(int a_idx, int x0, int y0, int x1, int y1, int x2, int y2)
{
  int y;

  for (y = 0; y < ROBOT_SHAPE_WINDOW; y++)
  {
    ContourX[y][0] = 999999; // min X
    ContourX[y][1] = -999999; // max X
  }

  ScanLine(x0, y0, x1, y1);
  ScanLine(x1, y1, x2, y2);
  ScanLine(x2, y2, x0, y0);

  for (y = 0; y < ROBOT_SHAPE_WINDOW; y++)
  {
    if (ContourX[y][1] >= ContourX[y][0])
    {
      long x = ContourX[y][0];
      long len = 1 + ContourX[y][1] - ContourX[y][0];


      // Can draw a horizontal line instead of individual pixels here
      while (len--)
      {
        SetPixel(a_idx, x, y);
	x++;
      }
    }
  }
}

#define TODEG(x) ((360.0*x)/(2.0*M_PI))

void draw_robot_shape(int a_idx, float ang)
{
	float o_x = (ROBOT_SHAPE_WINDOW/2.0)*(float)MAP_UNIT_W;
	float o_y = (ROBOT_SHAPE_WINDOW/2.0)*(float)MAP_UNIT_W;

	// Robot size plus margin
//	float robot_xs = (524.0 + 20.0);
//	float robot_ys = (480.0 + 20.0);

	float robot_xs = (524.0 + 0.0);
	float robot_ys = (380.0 + 0.0);

	float middle_xoffs = -120.0; // from o_x, o_y to the robot middle point.
	float middle_yoffs = -0.0;

/*

	Y                         / positive angle
	^                       /
	|                     /
	|                   /
	+----> X            -------------> zero angle

	Corner numbers:

	1-----------------------2
	|                       |
	|                       |
	|                       |
	|                 O     |  --->
	|                       |
	|                       |
	|                       |
	4-----------------------3

	Vector lengths O->1, O->2, O->3, O->4 are called a,b,c,d, respectively.
	Angles of those vectors relative to O are called ang1, ang2, ang3, ang4.

*/	
	// Basic Pythagoras thingie
	float a = sqrt(sq(0.5*robot_xs - middle_xoffs) + sq(0.5*robot_ys + middle_yoffs));
	float b = sqrt(sq(0.5*robot_xs + middle_xoffs) + sq(0.5*robot_ys + middle_yoffs));
	float c = sqrt(sq(0.5*robot_xs + middle_xoffs) + sq(0.5*robot_ys - middle_yoffs));
	float d = sqrt(sq(0.5*robot_xs - middle_xoffs) + sq(0.5*robot_ys - middle_yoffs));

//	printf("a=%.1f b=%.1f c=%.1f d=%.1f\n", a,b,c,d);

	// The angles:
	float ang1 = M_PI - asin((0.5*robot_ys)/a);
	float ang2 = asin((0.5*robot_ys)/b);
	float ang3 = -1*asin((0.5*robot_ys)/c);
	float ang4 = -1*(M_PI - asin((0.5*robot_ys)/d));

//	printf("ang1=%.2f ang2=%.2f ang3=%.2f ang4=%.2f\n", ang1,ang2,ang3,ang4);
//	printf("(ang1=%.2f ang2=%.2f ang3=%.2f ang4=%.2f)\n", TODEG(ang1),TODEG(ang2),TODEG(ang3),TODEG(ang4));

//	printf("adj ang = %.2f\n", ang);

	// Turn the whole robot:
	ang1 += ang;
	ang2 += ang;
	ang3 += ang;
	ang4 += ang;

//	printf("ang1=%.2f ang2=%.2f ang3=%.2f ang4=%.2f\n", ang1,ang2,ang3,ang4);

	float x1 = cos(ang1)*a + o_x;
	float y1 = sin(ang1)*a + o_y;
	float x2 = cos(ang2)*b + o_x;
	float y2 = sin(ang2)*b + o_y;
	float x3 = cos(ang3)*c + o_x;
	float y3 = sin(ang3)*c + o_y;
	float x4 = cos(ang4)*d + o_x;
	float y4 = sin(ang4)*d + o_y;

//	printf("x1 = %4.0f  y1 = %4.0f\n", x1, y1);
//	printf("x2 = %4.0f  y2 = %4.0f\n", x2, y2);
//	printf("x3 = %4.0f  y3 = %4.0f\n", x3, y3);
//	printf("x4 = %4.0f  y4 = %4.0f\n", x4, y4);

	// "Draw" the robot in two triangles:

	// First, triangle 1-2-3
	draw_triangle(a_idx, x1/(float)MAP_UNIT_W, y1/(float)MAP_UNIT_W, x2/(float)MAP_UNIT_W, y2/(float)MAP_UNIT_W, x3/(float)MAP_UNIT_W, y3/(float)MAP_UNIT_W);
	// Second, triangle 1-3-4
	draw_triangle(a_idx, x1/(float)MAP_UNIT_W, y1/(float)MAP_UNIT_W, x3/(float)MAP_UNIT_W, y3/(float)MAP_UNIT_W, x4/(float)MAP_UNIT_W, y4/(float)MAP_UNIT_W);
}


void dev_gen_robot_shapes()
{
	// Generate lookup tables showing the shape of robot in mapping unit matrices in different orientations.
	// These are used in mapping to test whether the (x,y) coords result in some part of a robot hitting a wall.

	for(int a=0; a<32; a++)
	{
		draw_robot_shape(a, ((float)a*2.0*M_PI)/32.0);

/*		printf("a = %d\n", a);

		for(int y = 0; y < ROBOT_SHAPE_WINDOW; y++)
		{
			for(int x = 0; x < ROBOT_SHAPE_WINDOW; x++)
			{
				printf(robot_shapes[a][x][y]?"# ":"  ");
			}
			printf("\n");
		}
*/
	}

}

search_unit_t* closed_set = NULL;
search_unit_t* open_set = NULL;


void dev_draw_sets(sf::RenderWindow& win)
{
	for(search_unit_t* p_iter = open_set; p_iter != NULL; p_iter=p_iter->hh.next)
	{
		dev_draw_circle(win, p_iter->loc.x, p_iter->loc.y, 250,50,50, p_iter->direction);
	}

	for(search_unit_t* p_iter = closed_set; p_iter != NULL; p_iter=p_iter->hh.next)
	{
		dev_draw_circle(win, p_iter->loc.x, p_iter->loc.y, 50,250,50, p_iter->direction);
	}
}

route_unit_t *some_route = NULL;

void dev_search(sf::RenderWindow& win, route_unit_t *route)
{
	win.setFramerateLimit(0);

	route = NULL;

	int s_x, s_y, e_x, e_y;
	unit_coords(dev_start_x, dev_start_y, &s_x, &s_y);
	unit_coords(dev_end_x, dev_end_y, &e_x, &e_y);

	printf("Start %d,%d,  end %d,%d\n", s_x, s_y, e_x, e_y);


	search_unit_t* p_start = (search_unit_t*) malloc(sizeof(search_unit_t));
	memset(p_start, 0, sizeof(search_unit_t));

	p_start->loc.x = s_x;
	p_start->loc.y = s_y;
	p_start->parent = NULL;
	// g = 0
	p_start->f = sqrt((float)(sq(e_x-s_x) + sq(e_y-s_y)));

	HASH_ADD(hh, open_set, loc,sizeof(xy_t), p_start);

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

				win.clear(sf::Color(200,220,240));
				draw_map(win);
				dev_draw_sets(win);
				win.display();


			search_unit_t* p_recon = p_cur;
			while( (p_recon = p_recon->parent) )
			{
				dev_draw_circle(win, p_recon->loc.x, p_recon->loc.y, 255,255,255,-123);
				win.display(); usleep(100000);
				route_unit_t* point = malloc(sizeof(route_unit_t));
				point->loc.x = p_recon->loc.x; point->loc.y = p_recon->loc.y;				
				DL_PREPEND(route, point);
			}

			route_unit_t *rt = route;
			while(1)
			{
				if(rt->next && rt->next->next)
				{
//					los_dbg = 1;
					if(line_of_sight(win,rt->loc, rt->next->next->loc))
					{
						dev_draw_circle(win, rt->next->loc.x, rt->next->loc.y, 255,128,128,-123);
						win.display(); usleep(200000);

						printf("Deleting.\n");
						route_unit_t *tmp = rt->next;
						DL_DELETE(route, tmp);
						free(tmp);
					}
					else
						rt = rt->next;
				}
				else
					break;
			}

			los_dbg = 0;

			DL_FOREACH(route, rt)
			{
				dev_draw_circle(win, rt->loc.x, rt->loc.y, 255,255,0,-123);
				win.display(); usleep(200000);
			}

			win.display();

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

				sf::Event event;
				while (win.pollEvent(event)) ;
				while(!sf::Keyboard::isKeyPressed(sf::Keyboard::Return)) ;


			return;
		}

		// move from open to closed:
		HASH_DELETE(hh, open_set, p_cur);
		HASH_ADD(hh, closed_set, loc,sizeof(xy_t), p_cur);

		// For each neighbor
		for(int xx=-1; xx<=1; xx++)
		{
			for(int yy=-1; yy<=1; yy++)
			{
				search_unit_t* found;
				float new_g;
				float new_g_from_parent;
				if(xx == 0 && yy == 0) continue;

				search_unit_t* p_neigh;
				xy_t neigh_loc = {p_cur->loc.x + xx, p_cur->loc.y + yy};

				// Check if it's out-of-allowed area here:


				HASH_FIND(hh, closed_set, &neigh_loc,sizeof(xy_t), found);
				if(found)
					continue; // ignore neighbor that's in closed_set.

				// gscore for the neigbor: distance from the start to neighbor
				// is current unit's g score plus distance to the neighbor.
				new_g = p_cur->g + sqrt((float)(sq(p_cur->loc.x-neigh_loc.x) + sq(p_cur->loc.y-neigh_loc.y)));

				// for theta*:
				if(p_cur->parent)
					new_g_from_parent = p_cur->parent->g + sqrt((float)(sq(p_cur->parent->loc.x-neigh_loc.x) + sq(p_cur->parent->loc.y-neigh_loc.y)));


				HASH_FIND(hh, open_set, &neigh_loc,sizeof(xy_t), p_neigh);


				int num_obstacles = 0;

				int direction_from_cur_parent = -1;
				int direction_from_neigh_parent = -1;

				if(p_cur->parent)  // Use this if possible
				{
					int dir_dx = neigh_loc.x - p_cur->parent->loc.x;
					int dir_dy = neigh_loc.y - p_cur->parent->loc.y;
					float ang = atan2(dir_dy, dir_dx);
					if(ang < 0.0) ang += 2.0*M_PI;
					int dir_parent = ang/(2.0*M_PI) * 32.0;
					direction_from_cur_parent = dir_parent;
				}

				if(p_neigh && p_neigh->parent)  // secondary
				{
					int dir_dx = p_neigh->parent->loc.x - neigh_loc.x;
					int dir_dy = p_neigh->parent->loc.y - neigh_loc.y;
					float ang = atan2(dir_dy, dir_dx);
					if(ang < 0.0) ang += 2.0*M_PI;
					int dir_parent = ang/(2.0*M_PI) * 32.0;
					direction_from_neigh_parent = dir_parent;
				}

				
				int direction = 0;
				if(!direction_from_cur_parent && !direction_from_neigh_parent)  // if the previous two don't work out.
				{
					if(xx==1 && yy==1)        direction = 1*4; 
					else if(xx==0 && yy==1)   direction = 2*4; 
					else if(xx==-1 && yy==1)  direction = 3*4; 
					else if(xx==-1 && yy==0)  direction = 4*4; 
					else if(xx==-1 && yy==-1) direction = 5*4; 
					else if(xx==0 && yy==-1)  direction = 6*4; 
					else if(xx==1 && yy==-1)  direction = 7*4; 
				}
				else
				{
					direction = direction_from_cur_parent;
				}


//				dev_draw_circle(win, neigh_loc.x, neigh_loc.y, 255,255,255, direction);

				if(check_hit(win, neigh_loc.x, neigh_loc.y, direction))
				{

					if(direction_from_neigh_parent != -1)
					{
						// try another thing before giving up:
						direction = direction_from_neigh_parent;

						if(check_hit(win, neigh_loc.x, neigh_loc.y, direction))
							continue;
					}
					else
						continue;

				}


				if(!p_neigh)
				{
					p_neigh = (search_unit_t*) malloc(sizeof(search_unit_t));
					memset(p_neigh, 0, sizeof(search_unit_t));
					p_neigh->loc.x = neigh_loc.x; p_neigh->loc.y = neigh_loc.y;
					HASH_ADD(hh, open_set, loc,sizeof(xy_t), p_neigh);

					p_neigh->direction = direction;
					p_neigh->parent = p_cur;
					p_neigh->g = new_g;
					p_neigh->f = new_g + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));

				}
				else
				{
					if(p_cur->parent && line_of_sight(win,p_cur->parent->loc, p_neigh->loc)) // Theta* style near-optimum (probably shortest) path
					{
						if(new_g_from_parent < p_neigh->g)
						{
							p_neigh->direction = direction;
							p_neigh->parent = p_cur->parent;
							p_neigh->g = new_g_from_parent;
							p_neigh->f = new_g_from_parent + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));
						}
					}
					else if(new_g < p_neigh->g)  // A* style path shorter than before.
					{
						p_neigh->direction = direction;
						p_neigh->parent = p_cur;
						p_neigh->g = new_g;
						p_neigh->f = new_g + sqrt((float)(sq(e_x-neigh_loc.x) + sq(e_y-neigh_loc.y)));
					}
				}


				OUT_OF_BOUNDS: ;

			}


		}		
	}


	win.clear(sf::Color(200,220,240));
	draw_map(win);
	dev_draw_sets(win);

	sf::Event event;
	while (win.pollEvent(event)) ;
	while(!sf::Keyboard::isKeyPressed(sf::Keyboard::Return)) ;


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


	dev_gen_robot_shapes();

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
					dev_search(win, some_route);
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
