#ifndef _AT_CMD_H_
#define _AT_CMD_H_

#include <stdio.h>

/**
 * AT-COMMAND support functions and definition
 */
#define TRUE		1
#define FALSE		0
#define SIZEOF(T)       (sizeof(T)/sizeof(T[0]))

#define MAX_COMMAND_PARAMS_COUNT 	20
struct AT_CMMD_HANDLES
{
	const char* cmmd;
	int (*cmmd_handle)(void);
};

struct AT_CMMD
{
	char* cmmd;
	int8_t param_count;
	char* params[MAX_COMMAND_PARAMS_COUNT];
};

extern const char* STR_ERROR;
extern const char* STR_OK;
extern struct AT_CMMD g_user_rtcmmd;

int at_reset(void);
int setAtCmdHandler(void* at_cmd_handlers, uint8_t size);
int AtParserCmd(char * cmd_buffer);
char* AtCmdClean(char* buf, uint16_t len);
int AtCmdMainProc(void);

#endif // _AT_CMD_H_
