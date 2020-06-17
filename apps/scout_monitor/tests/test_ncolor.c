/* color-demo.c */
// source: https://www.linuxjournal.com/content/about-ncurses-colors-0

#include <curses.h>
#include <stdio.h>
#include <stdlib.h>

int is_bold(int fg);
void init_colorpairs(void);
short curs_color(int fg);
int colornum(int fg, int bg);
void setcolor(int fg, int bg);
void unsetcolor(int fg, int bg);

void init_colorpairs(void)
{
    int fg, bg;
    int colorpair;

    for (bg = 0; bg <= 7; bg++)
    {
        for (fg = 0; fg <= 7; fg++)
        {
            colorpair = colornum(fg, bg);
            init_pair(colorpair, curs_color(fg), curs_color(bg));
        }
    }
}

short curs_color(int fg)
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
    }
}

int colornum(int fg, int bg)
{
    int B, bbb, ffff;

    B = 1 << 7;
    bbb = (7 & bg) << 4;
    ffff = 7 & fg;

    return (B | bbb | ffff);
}

void setcolor(int fg, int bg)
{
    /* set the color pair (colornum) and bold/bright (A_BOLD) */

    attron(COLOR_PAIR(colornum(fg, bg)));
    if (is_bold(fg))
    {
        attron(A_BOLD);
    }
}

void unsetcolor(int fg, int bg)
{
    /* unset the color pair (colornum) and
       bold/bright (A_BOLD) */

    attroff(COLOR_PAIR(colornum(fg, bg)));
    if (is_bold(fg))
    {
        attroff(A_BOLD);
    }
}

int is_bold(int fg)
{
    /* return the intensity bit */

    int i;

    i = 1 << 3;
    return (i & fg);
}

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
        exit(1);
    }

    start_color();
    init_colorpairs();

    /* draw test pattern */

    if ((LINES < 24) || (COLS < 80))
    {
        endwin();
        puts("Your terminal needs to be at least 80x24");
        exit(2);
    }

    mvaddstr(0, 35, "COLOR DEMO");
    mvaddstr(2, 0, "low intensity text colors (0-7)");
    mvaddstr(12, 0, "high intensity text colors (8-15)");

    for (bg = 0; bg <= 7; bg++)
    {
        for (fg = 0; fg <= 7; fg++)
        {
            setcolor(fg, bg);
            mvaddstr(fg + 3, bg * 10, "...test...");
            unsetcolor(fg, bg);
        }

        for (fg = 8; fg <= 15; fg++)
        {
            setcolor(fg, bg);
            mvaddstr(fg + 5, bg * 10, "...test...");
            unsetcolor(fg, bg);
        }
    }

    mvaddstr(LINES - 1, 0, "press any key to quit");

    refresh();

    getch();
    endwin();

    exit(0);
}
