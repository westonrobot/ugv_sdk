#include "monitor/ncolors.hpp"

using namespace westonrobot;

int main(void)
{
    int fg, bg;

    /* initialize curses */

    initscr();
    keypad(stdscr, TRUE);
    cbreak();
    noecho();

    /* initialize colors */

    if (has_colors() == FALSE)
    {
        endwin();
        puts("Your terminal does not support color");
        return 1;
    }

    NColors::InitColors();

    /* draw test pattern */

    if ((LINES < 24) || (COLS < 80))
    {
        endwin();
        puts("Your terminal needs to be at least 80x24");
        return 2;
    }

    mvaddstr(0, 35, "COLOR DEMO");
    mvaddstr(2, 0, "low intensity text colors (0-7)");
    mvaddstr(12, 0, "high intensity text colors (8-15)");

    for (bg = 0; bg <= 7; bg++)
    {
        for (fg = 0; fg <= 7; fg++)
        {
            NColors::SetColor(fg, bg);
            mvaddstr(fg + 3, bg * 10, "...test...");
            NColors::UnsetColor(fg, bg);
        }

        for (fg = 8; fg <= 15; fg++)
        {
            NColors::SetColor(fg, bg);
            mvaddstr(fg + 5, bg * 10, "...test...");
            NColors::UnsetColor(fg, bg);
        }
    }

    mvaddstr(LINES - 1, 0, "press any key to quit");

    refresh();

    getch();
    endwin();

    return 0;
}