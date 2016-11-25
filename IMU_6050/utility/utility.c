





#include <string.h>
#include "utility.h"




uint8_t Strigify(	char ** token,
					const char * main_string,
					char * return_buffer)
{

	int str_id = 0;
	int token_id = 0;
	for(int i = 0; i< strlen(main_string); i++)
	{
		char ch = main_string[i];
		if ( ch == '<')
		{
			i++;
			char ch1 = main_string[i];
			if (ch1 == '>')
			{
				strcat(return_buffer,token[token_id]);
				str_id = str_id + strlen(token[token_id]);
				token_id++;
			}
		}
		else
		{
			return_buffer[str_id++]= ch;
		}
	}
	return 0;
}





uint8_t SplitString(char * str, char token, char ** string_list)
{
	char * head = str;
	char * iterator = str;
	uint8_t count = 0;
	uint8_t field_count = 1;
	for ( ; *iterator != 0; )
	{
		if (*iterator == token)
		{
			*iterator = 0;
			string_list[count++] = head;
			head = (iterator + 1);
			field_count++;
		}
		iterator++;
	}
	string_list[count++] = head;
	return field_count++;
}



