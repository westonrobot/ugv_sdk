/* 
 * ncolors.cpp
 * 
 * Created on: Jun 20, 2019 06:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "monitor/ncolors.hpp"

#include <iostream>

namespace
{
int IsBold(int fg)
{
    /* return the intensity bit */

    int i;

    i = 1 << 3;
    return (i & fg);
}

int ColorNum(int fg, int bg)
{
    int B, bbb, ffff;

    B = 1 << 7;
    bbb = (7 & bg) << 4;
    ffff = 7 & fg;

    return (B | bbb | ffff);
}

short CursorColor(int fg)
{
    switch (7 & fg)
    {       /* RGB */
    case 0: /* 000 */
        return (COLOR_BLACK);
    case 1: /* 001 */
        return (COLOR_BLUE);
    case 2: /* 010 */
        return (COLOR_GREEN);
    case 3: /* 011 */
        return (COLOR_CYAN);
    case 4: /* 100 */
        return (COLOR_RED);
    case 5: /* 101 */
        return (COLOR_MAGENTA);
    case 6: /* 110 */
        return (COLOR_YELLOW);
    case 7: /* 111 */
        return (COLOR_WHITE);
    default:
        return COLOR_BLACK;
    }
}
} // namespace

namespace westonrobot
{
void NColors::InitColors()
{
    if (has_colors() != FALSE)
        start_color();
    else
        std::cerr << "Your terminal does not support color" << std::endl;

    int fg, bg;
    int colorpair;

    for (bg = 0; bg <= 7; bg++)
    {
        for (fg = 0; fg <= 7; fg++)
        {
            colorpair = ColorNum(fg, bg);
            init_pair(colorpair, CursorColor(fg), CursorColor(bg));
        }
    }
}

void NColors::SetColor(int fg, int bg)
{
    // set the color pair (ColorNum) and bold/bright (A_BOLD)
    attron(COLOR_PAIR(ColorNum(fg, bg)));
    if (IsBold(fg))
        attron(A_BOLD);
}

void NColors::UnsetColor(int fg, int bg)
{
    // unset the color pair (ColorNum) and bold/bright (A_BOLD)
    attroff(COLOR_PAIR(ColorNum(fg, bg)));
    if (IsBold(fg))
        attroff(A_BOLD);
}

void NColors::WSetColor(WINDOW *win, int fg, int bg)
{
    // set the color pair (ColorNum) and bold/bright (A_BOLD)
    wattron(win, COLOR_PAIR(ColorNum(fg, bg)));
    if (IsBold(fg))
        wattron(win, A_BOLD);
}

void NColors::WUnsetColor(WINDOW *win, int fg, int bg)
{
    // unset the color pair (ColorNum) and bold/bright (A_BOLD)
    wattroff(win, COLOR_PAIR(ColorNum(fg, bg)));
    if (IsBold(fg))
        wattroff(win, A_BOLD);
}
} // namespace westonrobot