
/* Include BEGIN */
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
/* Include END */


/* External BEGIN */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* External END */


/* Define BEGIN */
#define UART_COM huart1 /* Communication */
#define UART_DBG huart2 /* Debug */

#define DEBUG_UART 1
/* Define END */


void UART_Printf(const char* fmt, ...) {
    char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    HAL_UART_Transmit(&UART_COM, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    va_end(args);
}

void UART_Printf_Dbg(const char* fmt, ...) {

	#if(DEBUG_UART)
	char buff[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buff, sizeof(buff), fmt, args);
	HAL_UART_Transmit(&UART_DBG, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
	va_end(args);
	#endif
}
