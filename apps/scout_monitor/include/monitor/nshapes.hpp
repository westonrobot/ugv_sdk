/* 
 * nshapes.hpp
 * 
 * Created on: Jun 20, 2019 06:21
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef NSHAPES_HPP
#define NSHAPES_HPP

#include <ncurses.h>

namespace westonrobot
{
struct NShapes
{
    static void DrawRectangle(int tl_y, int tl_x, int br_y, int br_x);
    static void WDrawRectangle(WINDOW *win, int tl_y, int tl_x, int br_y, int br_x);
};
} // namespace westonrobot

#endif /* NSHAPES_HPP */
