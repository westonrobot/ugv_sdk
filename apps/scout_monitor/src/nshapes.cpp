/* 
 * nshapers.cpp
 * 
 * Created on: Jun 20, 2019 06:21
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "monitor/nshapes.hpp"

namespace westonrobot
{
void NShapes::DrawRectangle(int tl_y, int tl_x, int br_y, int br_x)
{
    for (int i = tl_y; i <= br_y; ++i)
    {
        mvprintw(i, tl_x, "|");
        mvprintw(i, br_x, "|");
    }
    for (int i = tl_x; i <= br_x; ++i)
    {
        mvprintw(tl_y, i, "-");
        mvprintw(br_y, i, "-");
    }
}

void NShapes::WDrawRectangle(WINDOW *win, int tl_y, int tl_x, int br_y, int br_x)
{
    for (int i = tl_y; i <= br_y; ++i)
    {
        mvwprintw(win, i, tl_x, "|");
        mvwprintw(win, i, br_x, "|");
    }
    for (int i = tl_x; i <= br_x; ++i)
    {
        mvwprintw(win, tl_y, i, "-");
        mvwprintw(win, br_y, i, "-");
    }
}
} // namespace westonrobot