/* 
 * ncolors.hpp
 * 
 * Created on: Jun 20, 2019 06:22
 * Description: 
 * 
 * Original source: https://www.linuxjournal.com/content/about-ncurses-colors-0
 * This copy is based on the original implementation, modified and 
 *  maintained by Ruixiang.
 * 
 * Copyright (c) 2018 Jim Hall
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef NCOLORS_HPP
#define NCOLORS_HPP

#include <ncurses.h>

namespace westonrobot
{
struct NColors
{
    enum BackgroundColor
    {
        BLACK = 0,
        BLUE,
        GREEN,
        CYAN,
        RED,
        MAGENTA,
        YELLOW,
        WHITE
    };

    enum ForegroundColor
    {
        /*
        BLACK = 0,
        BLUE,
        GREEN,
        CYAN,
        RED,
        MAGENTA,
        YELLOW,
        WHITE,*/
        BRIGHT_BLACK = 8,
        BRIGHT_BLUE,
        BRIGHT_GREEN,
        BRIGHT_CYAN,
        BRIGHT_RED,
        BRIGHT_MAGENTA,
        BRIGHT_YELLOW,
        BRIGHT_WHITE
    };

    static void InitColors();

    static void SetColor(int fg, int bg = BLACK);
    static void UnsetColor(int fg, int bg = BLACK);
    
    static void WSetColor(WINDOW *win, int fg, int bg = BLACK);
    static void WUnsetColor(WINDOW *win, int fg, int bg = BLACK);
};
} // namespace westonrobot

#endif /* NCOLORS_HPP */
