/*
 * This file is part of Betaflight-ESC (BFESC).
 *
 * Betaflight-ESC is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight-ESC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight-ESC.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>

#include "config/cli.h"
#include "drivers/drv_uart.h"
#include "drivers/drv_led.h"

#include "common/util.h"

#define CLI_IN_BUFFER_SIZE 128
static uint8_t bufferIndex = 0;

static char cliBuffer[CLI_IN_BUFFER_SIZE];

typedef struct {
    const char *name;
    void(*func)(char *cmdline);
} clicmd_t;

#define CLI_COMMAND_DEF(name, method) \
{ \
    name, \
    method \
}

static void cliPrint(const char *str)
{
    serialPrint(str);
}

static void cliPrintLinefeed(void)
{
    cliPrint("\r\n");
}

static void cliPrintLine(const char *str)
{
    cliPrint(str);
    cliPrintLinefeed();
}

static void cliPrompt(void)
{
    cliPrint("\r\n# ");
}

static void cliShowParseError(void)
{
    serialPrint("Parse error\r\n");
}

// Check if a string's length is zero
static bool isEmpty(const char *string)
{
    return (string == NULL || *string == '\0') ? true : false;
}

static void cliLED(char *cmdline)
{
    if (isEmpty(cmdline)) {
        cliShowParseError();
    } else {
        const char *ptr = cmdline;
        const int i = atoi(ptr);
        switch (i) {
            case 0:
                LED0_TOGGLE();
                break;
            case 1:
                LED1_TOGGLE();
                break;
            case 2:
                LED2_TOGGLE();
                break;
            default:         
                cliShowParseError();
        }
    }
}

static void cliADC(char *cmdline)
{
    if (isEmpty(cmdline)) {
        printf("V: %d.%03d, A: %d.%03d, T: %d.%03d\r\n", 
            (int)(motor_adc_get_voltage()),(int)((motor_adc_get_voltage()*1000))%1000, \
            (int)(motor_adc_get_current()),(int)((motor_adc_get_current()*1000))%1000, \
            (int)(motor_adc_get_temperature()),(int)((motor_adc_get_temperature()*1000))%1000);
    } else {
        cliShowParseError();
    }
}

static void cliWrite(char ch)
{
    serialWrite(ch);
}

static char *checkCommand(char *cmdLine, const char *command)
{
    if (!strncasecmp(cmdLine, command, strlen(command))   // command names match
        && (isspace((unsigned)cmdLine[strlen(command)]) || cmdLine[strlen(command)] == 0)) {
        return cmdLine + strlen(command) + 1;
    } else {
        return 0;
    }
}

// should be sorted a..z for bsearch()
const clicmd_t cmdTable[] = {
    CLI_COMMAND_DEF("led", cliLED),
    CLI_COMMAND_DEF("adc", cliADC),
};




void cliProcess(void)
{
    while (serialAvailable()) {
        uint8_t c = serialRead();
        if (c == 12) {                  // NewPage / CTRL-L
            // clear screen
            serialPrint("\033[2J\033[1;1H");
        } else if (bufferIndex && (c == '\n' || c == '\r')) {
            // enter pressed
            serialPrint("\r\n");

            // Strip comment starting with # from line
            char *p = cliBuffer;
            p = strchr(p, '#');
            if (NULL != p) {
                bufferIndex = (uint32_t)(p - cliBuffer);
            }

            // Strip trailing whitespace
            while (bufferIndex > 0 && cliBuffer[bufferIndex - 1] == ' ') {
                bufferIndex--;
            }

            // Process non-empty lines
            if (bufferIndex > 0) {
                cliBuffer[bufferIndex] = 0; // null terminate

                const clicmd_t *cmd;
                char *options;
                for (cmd = cmdTable; cmd < cmdTable + ARRAYLEN(cmdTable); cmd++) {
                    if ((options = checkCommand(cliBuffer, cmd->name))) {
                        break;
                    }
                }
                if (cmd < cmdTable + ARRAYLEN(cmdTable))
                    cmd->func(options);
                else
                    cliPrint("Unknown command, try 'help'");
                bufferIndex = 0;
            }

            memset(cliBuffer, 0, sizeof(cliBuffer));

            cliPrompt();
        } else if (c == 127) {
            // backspace
            if (bufferIndex) {
                cliBuffer[--bufferIndex] = 0;
                cliPrint("\010 \010");
            }
        } else if (bufferIndex < sizeof(cliBuffer) && c >= 32 && c <= 126) {
            if (!bufferIndex && c == ' ')
                continue; // Ignore leading spaces
            cliBuffer[bufferIndex++] = c;
            cliWrite(c);
        }
    }
    
}