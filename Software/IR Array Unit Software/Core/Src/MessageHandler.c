#include "MessageHandler.h"
#include <string.h>
#include <stdlib.h>

#define VERSION "1.0.0"

char gCmdBuffer[15][20];

static char * strtok_single (char * str, char const * delims);
static void tokenize(char *tString);
int memsearch(const char *hay, int haysize, const char *needle);


void HandleMessage(volatile tdsCommunicationBuffer * pBuffer)
{
    
    if(pBuffer->writePtr > 0)
    {
        HAL_Delay(100);
        int32_t pos = memsearch((const char *)pBuffer->buffer, pBuffer->writePtr, "*");
        if(pos != -1)//pBuffer->buffer[0] == '*')
        {
			int32_t rx_timeout = 100;

            while(memsearch((void *)pBuffer->buffer, pBuffer->writePtr, "#") == -1)
			{
				HAL_Delay(1);
				--rx_timeout;
				if(rx_timeout < 0)
				{
					memset((void *)pBuffer->buffer,0x00,sizeof(pBuffer->buffer));
					pBuffer->writePtr=0;
					return;
				}
			}
					
			uint8_t commandBuffer[200];
//            pBuffer->fncPtr_send(pBuffer->buffer, pBuffer->writePtr);
            memset((void *)commandBuffer, 0x00, 200);
            memcpy((void *)commandBuffer, (void *)&pBuffer->buffer[pos + 1], strlen((void *)&pBuffer->buffer[pos + 1])-1);
						
            tokenize((char *)commandBuffer);
            
			if(-1 != memsearch((char *)gCmdBuffer[1], 20, "OPEN"))
			{
				char temp[50];
				sprintf(temp,"*%s:OPENED#\r\n", gCmdBuffer[0]);

				uint32_t tTime = 0;
				uint32_t tPercentage = 100;

				if(strlen(gCmdBuffer[2]) != 0)
				{
					tTime = strtol(gCmdBuffer[2], NULL, 10);
				}

				if(strlen(gCmdBuffer[3]) != 0)
				{
					tPercentage = strtol(gCmdBuffer[3], NULL, 10);
					if(tPercentage > 100)
						tPercentage = 100;
				}

				printf("Laser opened with %ld time and %ld percentage\r\n", tTime, tPercentage);



				USB_Send((uint8_t *)temp, strlen(temp));
			}
			else if(-1 != memsearch((char *)gCmdBuffer[1], 20, "CLOSE"))
			{
				char temp[50];
				sprintf(temp,"*%s:CLOSED#\r\n", gCmdBuffer[0]);


				USB_Send((uint8_t *)temp, strlen(temp));
			}

			memset(gCmdBuffer[0], 0, 20);
			memset(gCmdBuffer[1], 0, 20);
			memset(gCmdBuffer[2], 0, 20);
			memset(gCmdBuffer[3], 0, 20);
			memset(gCmdBuffer[4], 0, 20);
			memset(gCmdBuffer[5], 0, 20);
			memset(gCmdBuffer[6], 0, 20);
			memset(gCmdBuffer[7], 0, 20);
			memset(gCmdBuffer[8], 0, 20);
			memset(gCmdBuffer[9], 0, 20);
			memset(gCmdBuffer[10], 0, 20);
			memset(gCmdBuffer[11], 0, 20);
			memset(gCmdBuffer[12], 0, 20);
			memset(gCmdBuffer[13], 0, 20);
			memset(gCmdBuffer[14], 0, 20);
						
            memset((void *)pBuffer->buffer,0x00,pBuffer->writePtr);
            pBuffer->writePtr=0;
        }
        else
        {
            memset((void *)pBuffer->buffer,0x00,pBuffer->writePtr);
            pBuffer->writePtr=0;
        }
    }
}

static void tokenize(char *tString)
{
    uint8_t tCnt= 0;
    char *word = strtok_single (tString, ":");
    while(word)
    {
//        printf("%d: %s\r\n",tCnt,*word ? word : "");
        switch(tCnt)
        {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
            case 10: 
            case 11:
            case 12:
            case 13: 
            case 14: 
                sprintf((char *)gCmdBuffer[tCnt],"%s",word);
                break;
            default: break;

        }
        
        word = strtok_single(NULL, ":");
        tCnt++;
    }
    
}

static char * strtok_single (char * str, char const * delims)
{
  static char  * src = NULL;
  char  *  p,  * ret = 0;

  if (str != NULL)
    src = str;

  if (src == NULL)
    return NULL;

  if ((p = strpbrk (src, delims)) != NULL) {
    *p  = 0;
    ret = src;
    src = ++p;

  } else if (*src) {
    ret = src;
    src = NULL;
  }

  return ret;
}

int memsearch(const char *hay, int haysize, const char *needle)
{
    size_t needlesize = strlen(needle);
    int haypos, needlepos;
    haysize -= needlesize;
    for (haypos = 0; haypos <= haysize; haypos++) {
        for (needlepos = 0; needlepos < needlesize; needlepos++) {
            if (hay[haypos + needlepos] != needle[needlepos]) {
                // Next character in haystack.
                break;
            }
        }
        if (needlepos == needlesize) {
            return haypos;
        }
    }    
    return -1;
}

