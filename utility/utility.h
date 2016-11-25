





#ifndef UTILITY_H
#define UTILITY_H


#include <stdint.h>

uint8_t Strigify(char ** token, const char * main_string, char * return_buffer);
uint8_t SplitString(char * str, char token, char ** string_list);


#endif // UTILITY_H
