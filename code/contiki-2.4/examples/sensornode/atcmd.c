
#include "atcmd.h"
#include <string.h> /* For memcpy */

const char* STR_ERROR = "\r\nERROR\r\n";
const char* STR_OK = "\r\nOK\r\n";

static uint8_t g_size = 0;

struct AT_CMMD g_user_rtcmmd;
struct AT_CMMD_HANDLES* g_cmmd_handles;	

static void
(*reset)(void) = 0x0000;

int at_reset(void)
{
  reset();
  return TRUE;
}

int setAtCmdHandler(void* at_cmd_handlers, uint8_t size)
{  
    if(!at_cmd_handlers || 0 == size) return FALSE;

    g_cmmd_handles = (struct AT_CMMD_HANDLES* )at_cmd_handlers;
    g_size = size;

    return TRUE;
}

int AtParserCmd(char * cmd_buffer)
{ 
  if(!cmd_buffer)   return FALSE;
  
  char* p = cmd_buffer;
  char* buf_ptr = cmd_buffer;

  // clear user at command buf.
  memset(&g_user_rtcmmd, 0, sizeof(g_user_rtcmmd));		 

  while (1)
    {
      if (*p == '=')
        {
          *p++ = '\0';
          g_user_rtcmmd.cmmd = buf_ptr;
          buf_ptr = p;
        }
      else if (*p == ',')
        {
          *p++ = '\0';
          g_user_rtcmmd.params[g_user_rtcmmd.param_count++] = buf_ptr;
          buf_ptr = p;
        }
      else if (*p == '\0' || *p == '\r' || *p == 0xff)
        {
		  *p = '\0';
          if (buf_ptr != p)
            {
              if (NULL == g_user_rtcmmd.cmmd)
                {
                  g_user_rtcmmd.cmmd = buf_ptr;
                }
              else
                {
                  g_user_rtcmmd.params[g_user_rtcmmd.param_count++] = buf_ptr;
                }
            }

          break;
        }
      else 
        {
          p++;
        }
    }

  return TRUE;
}

char* AtCmdClean(char* buf, uint16_t len)
{
	char *p = buf;	
	while(len--){
		if(*p != EOF && *p != '\0' && *p != '\r' && *p != '\n' && *p != ' ') return p;
		p++;
	}
	
	return NULL;
}

/**
 * AT-Command main handle
 */
int AtCmdMainProc(void)
{
  uint8_t i;  

  for(i = 0; i < g_size; ++i)
  {
      uint8_t ret = strcmp((g_cmmd_handles+i)->cmmd, g_user_rtcmmd.cmmd);   
      if (0 == strcmp((g_cmmd_handles+i)->cmmd, g_user_rtcmmd.cmmd))
      {
          if ((g_cmmd_handles+i)->cmmd_handle)
          {
                (g_cmmd_handles+i)->cmmd_handle();
                return TRUE;
          }            
        }
    }

    return FALSE;
}