/*
 * cmd.c
 *
 * Created: 16-Dec-15 23:59:04
 *  Author: Shakran PC
 */ 





#include <stddef.h>
#include "cmd.h"
#include <string.h>
#include <stdio.h>
#include "heap.h"
#include "time.h"






int ParseCmdString(char *cmd_string, ParamList *list)
{
	if (cmd_string  == NULL)
	{
		return -1;
	}
	
	char * first_string = strtok(cmd_string, " ");

	if(first_string == NULL) return 0;
	if(strlen(first_string)>20)	return -1;
	memset(list->cmd, 0, LENGTHOFCMD);
	strncpy(list->cmd, first_string,LENGTHOFCMD);


	char * second_string = strtok(NULL," ");
	if(second_string == NULL) return 0;
	if(strlen(second_string)>20)	return -1;
	memset(list->Param1, 0, LENGTHOFCMD);
	strncpy(list->Param1, second_string,LENGTHOFCMD);



	char * thired_string = strtok(NULL," ");
	if(thired_string == NULL) return 1;
	if(strlen(thired_string)>20)	return -1;
	memset(list->Param2, 0, LENGTHOFCMD);
	strncpy(list->Param2, thired_string,LENGTHOFCMD);


	char * fourth_string = strtok(NULL," ");
	if(fourth_string == NULL) return 2;
	if(strlen(fourth_string)>20)	return -1;
	memset(list->Param3, 0, LENGTHOFCMD);
	strncpy(list->Param3, fourth_string,LENGTHOFCMD);


	char * fifth_string = strtok(NULL," ");
	if(fifth_string == NULL) return 3;
	if(strlen(fifth_string)>20)	return -1;
	memset(list->Param4, 0, LENGTHOFCMD);
	strncpy(list->Param4, fifth_string,LENGTHOFCMD);


	char * sixth_string = strtok(NULL," ");
	if(sixth_string == NULL) return 4;
	if(strlen(sixth_string)>20)	return -1;
	memset(list->Param5, 0, LENGTHOFCMD);
	strncpy(list->Param5, sixth_string,LENGTHOFCMD);


	char * seven_string = strtok(NULL," ");
	if(seven_string == NULL) return 5;
	if(strlen(seven_string)>20)	return -1;
	memset(list->Param6, 0, LENGTHOFCMD);
	strncpy(list->Param6, seven_string,LENGTHOFCMD);


	char * eithth_string = strtok(NULL," ");
	if(eithth_string == NULL) return 6;
	if(strlen(eithth_string)>20)	return -1;
	memset(list->Param7, 0, LENGTHOFCMD);
	strncpy(list->Param7, eithth_string,LENGTHOFCMD);

	
	return 7;
}




 void Execute(CLIHandle _cli)
 {
 	if(_cli == NULL) return;
 	CLI * cli_obj = (CLI *)_cli;
 	switch(cli_obj->current_processing_command->max_param_count)
 	{
 		case 0:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer);
 			break;
 		case 1:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1);
 			break;
 		case 2:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1, cli_obj->param_list->Param2);
 			break;
 		case 3:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1, cli_obj->param_list->Param2, cli_obj->param_list->Param3);
 			break;
 		case 4:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1, cli_obj->param_list->Param2, cli_obj->param_list->Param3, cli_obj->param_list->Param4);
 			break;
 		case 5:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1, cli_obj->param_list->Param2, cli_obj->param_list->Param3, cli_obj->param_list->Param4, cli_obj->param_list->Param5);
 			break;
 		case 6:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1, cli_obj->param_list->Param2, cli_obj->param_list->Param3, cli_obj->param_list->Param4, cli_obj->param_list->Param5, cli_obj->param_list->Param6);
 			break;
 		case 7:
 			cli_obj->current_processing_command->ptr.func(cli_obj->reply_buffer, cli_obj->param_list->Param1, cli_obj->param_list->Param2, cli_obj->param_list->Param3, cli_obj->param_list->Param4, cli_obj->param_list->Param5, cli_obj->param_list->Param6, cli_obj->param_list->Param7);
 			break;
 			
 	}
 	
 }
 
 
 
 uint8_t CheckPermission(uint8_t perm_value, uint8_t user)
 {
	 switch(user)
	 {
		case 0:
			return 1;
			break;
		case 1:
			if (perm_value & MASTER_ACCESS)
				return 1;
			else
				return 0;
			break;
			
		case 2:
			if (perm_value & DEVELOPER_ACCESS)
				return 1;
			else
				return 0;
			break;
			
		case 3:
			if (perm_value & USER0_ACCESS)
				return 1;
			else
				return 0;			
			break;
			
		case 4:
			if (perm_value & USER1_ACCESS)
				return 1;
			else
				return 0;
			break;
			
		case 5:
			if (perm_value & USER2_ACCESS)
				return 1;
			else
				return 0;
			break;
				
		default:
			return 0;				
			break;
	 }
 }
 

 
 void Invoke(CLIHandle _cli, char * cmd_string)
{
 	
 	CLI * cli_obj = (CLI *)_cli;
 	ParamList param_list;
 	memset(cli_obj->reply_buffer, 0, 160);
 	memset(&param_list, 0, sizeof(param_list)); 	
 	int count = ParseCmdString(cmd_string, &param_list); 
 	CmdType * cmd =NULL;
	if (count == -1)
	{
		strcpy(cli_obj->reply_buffer, "Parameter length should be less than 20 character.");
		return;
	}
	
	
 	for (int i = 0; 1; i++)
 	{
		cmd = (CmdType *)&cli_obj->cmd_list[i];
		if (cmd == NULL)
		{
			break;
		}
		CmdType _cmd;
		
		memset(&_cmd,0,sizeof(CmdType));
		
		memcpy(&_cmd, cmd, sizeof(CmdType));
		
		if (strcmp(param_list.cmd,_cmd.cmd) == 0)
		{
			
			if (_cmd.max_param_count < count)
			{
				strcpy(cli_obj->reply_buffer, "Invalid parameter list.\r\n");
				return;
			}
			cli_obj->current_processing_command = &_cmd;
			
			if (CheckPermission(_cmd.permission,cli_obj->user_id) == 0)
			{
				strcpy(cli_obj->reply_buffer, "Access Denied.\r\n");
				return;	
			}
			cli_obj->param_list = &param_list;
			Execute(_cli);
			return;
		}
 		
 	}
 	strcpy(cli_obj->reply_buffer, "Command Not found.\r\n");
 }
 
 
 
 
uint16_t sizeofcli()
{

	{
		return sizeof(CLI);
	}
}

 CLIHandle InitCmdModule()
 {
	return &cmd_manage;
 }
 
