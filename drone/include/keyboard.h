#pragma once

#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <stdio.h>

extern struct termios initial_settings, new_settings;
 
extern int peek_character;

extern bool check_keyboardSetting;

void init_keyboard();
void close_keyboard();


// Check input char
int _kbhit();

// return received char, if peek_character is not -1
int _getch();

// Display char in terminal
int _putch(int c);
