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



	Helpers to draw&click buttons etc.

*/

#include <iostream>
#include <cstdio>
#include <cstring>
#include <SFML/Graphics.hpp>

#include "sfml_gui.h"

using namespace std;

sfml_gui::sfml_gui(sf::RenderWindow& win, sf::Font& fo) : window(win), font(fo)
{
	num_buttons = 0;
}

void sfml_gui::change_text(int num, const char* text)
{
	delete buttons[num]->text;
	buttons[num]->text = new char[strlen(text)+1];
	strcpy(buttons[num]->text, text);
}

int sfml_gui::add_button(int x, int y, int xs, int ys, const char* text, sf::Color color, int font_size,
			 int symbol, sf::Color color_pressed, bool pressed)
{
	buttons[num_buttons] = new button;
	buttons[num_buttons]->x = x;
	buttons[num_buttons]->y = y;
	buttons[num_buttons]->xs = xs;
	buttons[num_buttons]->ys = ys;
	buttons[num_buttons]->text = new char[strlen(text)+1];
	strcpy(buttons[num_buttons]->text, text);
	buttons[num_buttons]->visible = true;
	buttons[num_buttons]->font_size = font_size;
	buttons[num_buttons]->color = color;
	buttons[num_buttons]->symbol = symbol;
	buttons[num_buttons]->pressed = pressed;
	buttons[num_buttons]->color_pressed = color_pressed;
	buttons[num_buttons]->text_color = sf::Color(0,0,0);

	num_buttons++;

	return num_buttons-1;
}

void sfml_gui::hide_button(int num)
{
	buttons[num]->visible = false;
}

void sfml_gui::show_button(int num)
{
	buttons[num]->visible = true;
}

void sfml_gui::draw_button(int num)
{
	if(buttons[num]->visible == false)
	return;

	const sf::Color line_color = sf::Color(0,0,0);
	const int shadow_offset = 2;
	const sf::Color shadow_color = sf::Color(100,100,100);

	// Draw shadow first

	sf::RectangleShape rect(sf::Vector2f(buttons[num]->xs, buttons[num]->ys));
	rect.setPosition(buttons[num]->x+shadow_offset, buttons[num]->y+shadow_offset);
	rect.setFillColor(shadow_color);
	rect.setOutlineThickness(2.0);
	rect.setOutlineColor(shadow_color);

	window.draw(rect);

	int button_offset = (buttons[num]->pressed)?shadow_offset:0;
	rect.setPosition(buttons[num]->x+button_offset, buttons[num]->y+button_offset);
	rect.setFillColor((buttons[num]->pressed)?(buttons[num]->color_pressed):(buttons[num]->color));
	rect.setOutlineColor(line_color);

	window.draw(rect);

	int text_offset = 0;
	const int sym_xoffs = 5;
	const int sym_yoffs = 6;
	if(buttons[num]->symbol > -1)
	{
		for(int poly = 0; poly < SYMBOLS[buttons[num]->symbol].n_polys; poly++)
		{
			sf::ConvexShape shape;
			if(SYMBOLS[buttons[num]->symbol].polys[poly].line) shape.setOutlineThickness(SYMBOLS[buttons[num]->symbol].polys[poly].line_width);
			shape.setFillColor(SYMBOLS[buttons[num]->symbol].polys[poly].fill_color);
			shape.setOutlineColor(SYMBOLS[buttons[num]->symbol].polys[poly].line_color);
			shape.setPointCount(SYMBOLS[buttons[num]->symbol].polys[poly].n_points);

			for(int i = 0; i < SYMBOLS[buttons[num]->symbol].polys[poly].n_points; i++)
			{
				shape.setPoint(i, sf::Vector2f(SYMBOLS[buttons[num]->symbol].polys[poly].points[i*2],
					SYMBOLS[buttons[num]->symbol].polys[poly].points[i*2+1]));
			}

			shape.setPosition(buttons[num]->x+sym_xoffs+button_offset, buttons[num]->y+sym_yoffs+button_offset);
			window.draw(shape);
			text_offset = SYMBOLS[buttons[num]->symbol].text_offset;
		}
	}

	sf::Text t;
	t.setFont(font);
	t.setCharacterSize(buttons[num]->font_size);
	t.setString(buttons[num]->text);
	t.setFillColor(buttons[num]->text_color);
	t.setPosition(buttons[num]->x+5+text_offset+button_offset, buttons[num]->y+3+button_offset);
	window.draw(t);
}

void sfml_gui::draw_all_buttons()
{
	for(int i = 0; i < num_buttons; i++)
	{
		draw_button(i);
	}
}

int sfml_gui::check_button_status()
{
	unsigned int x, y;
	bool mouse;

	x = sf::Mouse::getPosition(window).x;
	y = sf::Mouse::getPosition(window).y;
	mouse = sf::Mouse::isButtonPressed(sf::Mouse::Left);

	if(mouse) // && x > 0 && y > 0 && x < window_xsize && y < window_ysize)
	{
		for(int i = 0; i < num_buttons; i++)
		{
			if(x > buttons[i]->x && x < buttons[i]->x+buttons[i]->xs &&
			   y > buttons[i]->y && y < buttons[i]->y+buttons[i]->ys &&
			buttons[i]->visible)
			return i;
		}
	}

	return -1;

}

/*
void sfml_gui::query_mouse(int* x, int* y, bool* left, bool* mid, bool* right)
{
	*x = input.GetMouseX();
	*y = input.GetMouseY();
	if(left != 0)
	*left = input.IsMouseButtonDown(sf::Mouse::Left);
	if(mid != 0)
	*mid = input.IsMouseButtonDown(sf::Mouse::Middle);
	if(right != 0)
	*right = input.IsMouseButtonDown(sf::Mouse::Right);
}
*/

void sfml_gui::destroy_buttons()
{
	for(int i = 0; i < num_buttons; i++)
	{
		delete buttons[i]->text;
		delete buttons[i];
		buttons[i]->text = 0;
		buttons[i] = 0;
	}

	num_buttons = 0;
}

