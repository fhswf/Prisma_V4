/*
  ___          _____     __  __
   _/    _  _ (_  | |\/|  _)  _)
  /__|_||||(_)__) | |  | __) /__

 Fachhochschule S�dwestfalen
 Mechatronik/Mikrocomputer
 Prof. Dr.-Ing. Tobias Ellermeyer

 Datei: display.h
 Pfad:  zumolib/inc/zumo

 Erstellt:    20.08.2019
 Autor:       Tobias Ellermeyer
 Update:	  08.11.2022 - STM32CubeIDE / stdint
*/

#ifndef ZUMO_DISPLAY_H_
#define ZUMO_DISPLAY_H_

void display_init();
void display_update();
void display_clear();
void display_clear_f();
void display_println(char* msg);
void display_gotoxy(int16_t x, int16_t y);
void display_scroll();
void display_line_f(int16_t x1, int16_t y1, int16_t x2, int16_t y2);



#endif /* ZUMO_DISPLAY_H_ */
