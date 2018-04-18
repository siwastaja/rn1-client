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

#ifndef UI_HPP
#define UI_HPP

#include <SFML/Graphics.hpp>

#define MAX_BUTTONS	 64

struct button
{
	int x;
	int y;
	int xs;
	int ys;
	bool visible;
	//	int state;
	bool pressed;
	char* text;
	int symbol;
	int font_size;
	sf::Color color;
	sf::Color color_pressed;
	sf::Color text_color;
};

#define SYMB_MAX_POLYS   5
#define POLY_MAX_POINTS 20*2

struct poly
{
	int n_points;
	sf::Color fill_color;
	bool line;
	int line_width;
	sf::Color line_color;
	int points[POLY_MAX_POINTS];
};

struct symbol
{
	int n_polys;
	poly polys[SYMB_MAX_POLYS];
	int text_offset;
};


const symbol SYMBOLS[] = 
{     // nPoint fill? fill color            line?  W  line color            points (x,y,x,y,...)          text offset
	1, {3, sf::Color(180,  0,  0), true, 1, sf::Color(  0, 80,  0), {0,0, 0,10, 10,5}},           15,	    // 0 = PLAY
	1, {4, sf::Color(  0,180, 40), true, 1, sf::Color( 80,  0,  0), {0,0, 0,10, 10,10, 10,0}},    15,	    // 1 = STOP
	1, {3, sf::Color(  0,200,200), true, 1, sf::Color(  0, 90, 90), {0,2, 0,8,   8,5}},           15,	    // 2 = PLAYSLOW
	1, {3, sf::Color(110,255,110), true, 1, sf::Color( 30, 60, 30), {0,0, 0,10, 10,5}},           15,	    // 3 = ROUTE
	1, {3, sf::Color(235,235,110), true, 1, sf::Color( 60, 60, 30), {0,0, 0,10, 10,5}},           15,	    // 4 = MANUAL
	1, {3, sf::Color(255,110,110), true, 1, sf::Color( 60, 30, 30), {0,0, 0,10, 10,5}},           15,	    // 5 = FORCE
	1, {3, sf::Color(110,200,200), true, 1, sf::Color( 30, 60, 60), {0,0, 0,10, 10,5}},           15	    // 6 = POSE
};

#define SYM_PLAY 0
#define SYM_STOP 1
#define SYM_PLAYSLOW 2
#define SYM_ROUTE  3
#define SYM_MANUAL 4
#define SYM_FORCE  5
#define SYM_POSE   6

const sf::Color DEF_BUT_COL = sf::Color(200,200,200);
const sf::Color DEF_BUT_COL_PRESSED = sf::Color(128,128,128);
const int DEF_BUT_FONT_SIZE = 14;

// Multiple Windows or RenderWindows each need their own sfml_gui objects.

struct sfml_gui
{
	sfml_gui(sf::RenderWindow& win, sf::Font& fo);

	sf::RenderWindow& window;  

	sf::Font font;

//	int window_xsize; 
//	int window_ysize; 

	int num_buttons;

	button* buttons[MAX_BUTTONS];
	//	checkbox checkboxes[MAX_CHECKBOXES];


	// Returns the tag value of the new button.
	int add_button(int x, int y, int xs, int ys, const char* text, sf::Color color = DEF_BUT_COL, int font_size = DEF_BUT_FONT_SIZE,  
		   int symbol = -1, sf::Color color_pressed = DEF_BUT_COL_PRESSED, bool pressed = false);

	void destroy_buttons();

	void show_button(int num);
	void hide_button(int num);

	// Draws the button on screen
	void draw_button(int num);

	// Draws all the buttons on screen. Calls draw_button.
	void draw_all_buttons();

	// Returns the number of button pressed, or -1 if no button is pressed
	int check_button_status();

//	void query_mouse(int* x, int* y, bool* left, bool* mid = 0, bool* right = 0);

	void change_text(int num, const char* text);

};

#endif

