

/*
 * cmd.h
 *
 * Created: 16-Dec-15 23:58:38
 *  Author: Shakran PC
 */ 


#ifndef CMD_H_
#define CMD_H_
#include <stdint.h>
#include "fifo.h"
#include "time.h"

#define LENGTHOFCMD 20



typedef void * CLIHandle;

#define MASTER_ACCESS 0x01
#define DEVELOPER_ACCESS 0x02
#define USER0_ACCESS 0x04
#define USER1_ACCESS 0x08
#define USER2_ACCESS 0x10



typedef struct
{
	char cmd[LENGTHOFCMD];
	char Param1[LENGTHOFCMD];
	char Param2[LENGTHOFCMD];
	char Param3[LENGTHOFCMD];
	char Param4[LENGTHOFCMD];
	char Param5[LENGTHOFCMD];
	char Param6[LENGTHOFCMD];
	char Param7[LENGTHOFCMD];
} ParamList;



typedef void(*func_ptr0)(char * reply);
typedef void(*func_ptr1)(char * reply, char * param1);
typedef void(*func_ptr2)(char * reply, char * param1,char *param2);
typedef void(*func_ptr3)(char * reply, char * param1,char *param2,char *param3);
typedef void(*func_ptr4)(char * reply, char * param1,char * param2,char *param3,char *param4);
typedef void(*func_ptr5)(char * reply, char * param1,char * param2,char * param3,char *param4,char *param5);
typedef void(*func_ptr6)(char * reply, char * param1,char * param2,char * param3,char *param4,char *param5,char *param6);
typedef void(*func_ptr7)(char * reply, char * param1,char * param2,char * param3,char *param4,char *param5,char *param6,char *param7);


typedef union
{
	union
	{
		func_ptr0 ptr0;
		func_ptr1 ptr1;
		func_ptr2 ptr2;
		func_ptr3 ptr3;
		func_ptr4 ptr4;
		func_ptr5 ptr5;
		func_ptr6 ptr6;
		func_ptr7 ptr7;
	};
	void (*func)();
}FuncPtrType;



typedef struct
{
	uint8_t permission: 5;
	uint8_t max_param_count: 3;
	const char * cmd;
	FuncPtrType ptr;
}CmdType;

typedef struct
{
	const CmdType * const *  cmd_list;
	char cmd_buffer[60];
	char reply_buffer[162];
	CmdType * current_processing_command;
	ParamList * param_list;
	uint8_t user_id;
	uint8_t state;
	uint8_t invoke_state;
	uint8_t last_status;	
	uint8_t total_avaiable_sms;
	Time invoke_time;
	uint8_t sms_index_current;
	int _index;
}CLI;



extern CLI cmd_manage;

typedef void (*CallBackFunc)();

CLIHandle InitCmdModule();
void Invoke(CLIHandle _cli, char * cmd_string);
#endif /* CMD_H_ */
