#ifndef __MPRINTF_H
#define __MPRINTF_H

double atod(char *str);
int dprint(double x, int campo, int frac, void (*putc)(int));
int mprintf(void (*putc)(int), const char *formato, ... );
int sprintf(char *buf, const char *formato, ... );
int mscanf(int (*getc)(void), const char *formato, ...);
int va_printf(void (*putc)(int), const char *formato, va_list va);
//int sscanf(char *buf, const char *formato, ... );

#endif
