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

sf::Font arial;

int screen_x = 1200;
int screen_y = 900;

double origin_x = 0;
double origin_y = 0;


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

double mm_per_pixel = 10.0;

void draw_page(sf::RenderWindow& win, map_page_t* page, int startx, int starty)
{
	if(!page)
		return;

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


	for(int x = 0; x < MAP_PAGE_W; x++)
	{
		for(int y = 0; y < MAP_PAGE_W; y++)
		{
			if(page->units[x][y].result)
			{
				sf::RectangleShape rect(sf::Vector2f(MAP_UNIT_W/mm_per_pixel,MAP_UNIT_W/mm_per_pixel));
				//rect.setOrigin(0.0,0.0);
				rect.setPosition((startx+x*MAP_UNIT_W)/mm_per_pixel, (starty+y*MAP_UNIT_W)/mm_per_pixel);
				if(page->units[x][y].result & UNIT_WALL)
					rect.setFillColor(sf::Color(60,20,0));
				else
					rect.setFillColor(sf::Color(0,100,200));

				win.draw(rect);
			}
		}
	}
}

map_page_t*  pages[MAP_W][MAP_W];
qmap_page_t* qpages[MAP_W][MAP_W];

void draw_map(sf::RenderWindow& win)
{
	for(int x = 0; x < MAP_W; x++)
	{
		for(int y = 0; y < MAP_W; y++)
		{
			if(pages[x][y])
			{
				int startx = -MAP_MIDDLE_UNIT*MAP_UNIT_W + x*MAP_PAGE_W*MAP_UNIT_W + origin_x;
				int starty = -MAP_MIDDLE_UNIT*MAP_UNIT_W + y*MAP_PAGE_W*MAP_UNIT_W + origin_y;
				draw_page(win, pages[x][y], startx, starty);
			}
		}
	}
}

sf::IpAddress serv_ip;
unsigned short serv_port;

int main(int argc, char** argv)
{

	pages[128][128] = (map_page_t*)malloc(sizeof(map_page_t));

	pages[128][128]->units[100][100].result = 2;
	pages[128][128]->units[101][100].result = 2;
	pages[128][128]->units[102][100].result = 2;
	pages[128][128]->units[103][100].result = 2;
	pages[128][128]->units[104][100].result = 2;
	pages[128][128]->units[105][100].result = 2;
	pages[128][128]->units[105][101].result = 2;
	pages[128][128]->units[105][102].result = 2;
	pages[128][128]->units[105][103].result = 2;

	pages[128][128]->units[120][120].result = 1;


	pages[129][128] = (map_page_t*)malloc(sizeof(map_page_t));

	pages[129][128]->units[100][100].result = 1;
	pages[129][128]->units[101][100].result = 1;
	pages[129][128]->units[102][100].result = 1;
	pages[129][128]->units[103][100].result = 1;
	pages[129][128]->units[104][100].result = 1;
	pages[129][128]->units[105][100].result = 1;
	pages[129][128]->units[105][101].result = 1;
	pages[129][128]->units[105][102].result = 1;
	pages[129][128]->units[105][103].result = 1;

	pages[129][128]->units[120][120].result = 2;

	if(argc != 3)
	{
		printf("Usage: rn1client addr port\n");
		return 1;
	}

	serv_ip = argv[1];
	serv_port = atoi(argv[2]);

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
			
		}

		if(1)
		{
			sf::Vector2i localPosition = sf::Mouse::getPosition(win);
			double click_x = (localPosition.x * mm_per_pixel) - origin_x;
			double click_y = (localPosition.y * mm_per_pixel) - origin_y;

			if(sf::Mouse::isButtonPressed(sf::Mouse::Left))
			{
				printf("Click x=%.1f y=%.1f mm\n", click_x, click_y);
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

			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
			{
				origin_y -= 20.0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
			{
				origin_y += 20.0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
			{
				origin_x -= 20.0;
			}
			if(sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
			{
				origin_x += 20.0;
			}
		}

		win.clear(sf::Color(200,220,240));

		draw_map(win);

		win.display();

		usleep(100);

	}
	return 0;
}
