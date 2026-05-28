#ifndef MENU_HPP
#define MENU_HPP

void menu_init(void);
void menu_process(void);
void menu_show(void);
void menu_show_run_state(void);

int menu_is_enabled(void);
int menu_is_editing(void);

int menu_car_is_started(void);
void menu_car_stop(void);

#endif