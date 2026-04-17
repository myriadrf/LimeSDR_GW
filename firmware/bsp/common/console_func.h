//
// Created by ts on 5/14/25.
//

#ifndef CONSOLE_H
#define CONSOLE_H

char *readstr(void);

char *get_token(char **str);

void prompt(void);

void help(void);

void reboot_cmd(void);

void led_cmd(void);

void gpioled_cmd(void);

void mmap_cmd(void);

void i2c_test(void);

void lms7002_status(void);

void console_service(void);

#endif // CONSOLE_H