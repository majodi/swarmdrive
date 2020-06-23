/**************************************************************************/
/*!
    @file     ns_termcontrol.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Library for crude terminal control

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2020, NickStick BV
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>
#include <iostream>

#ifndef _NS_TERMCONTROL_H_
#define _NS_TERMCONTROL_H_

// terminal codes
#define _NS_TERM_CLS "\x1b[2J"
#define _NS_TERM_SET_BA(f, t) "\x1b[" + string(#f) + ";" + string(#t) + "r"
#define _NS_TERM_PRINTF_SET_BA "\x1b[%s;%sr"
#define _NS_TERM_SAV_POS "\x1b[s"
#define _NS_TERM_RES_POS "\x1b[u"
        // Attention!! this GOTO_REP returns position!
#define _NS_TERM_GOTO_REP(r, c) "\x1b[" + string(#r) + ";" + string(#c) + "H\x1B[6n"
#define _NS_TERM_PRINTF_GOTO "\x1b[%s;%sH"
#define _NS_TERM_SET_LINE "\x1b(0"
#define _NS_TERM_SET_CHAR "\x1b(B"

// line drawing
#define _NS_TERM_DRAW_BOT_CORNER_END 'j'
#define _NS_TERM_DRAW_TOP_CORNER_END 'k'
#define _NS_TERM_DRAW_TOP_CORNER_START 'l'
#define _NS_TERM_DRAW_BOT_CORNER_START 'm'
#define _NS_TERM_DRAW_CROSS 'n'
#define _NS_TERM_DRAW_HOR 'q'
#define _NS_TERM_DRAW_T_LEFT 't'
#define _NS_TERM_DRAW_T_RIGHT 'u'
#define _NS_TERM_DRAW_T_BOT 'v'
#define _NS_TERM_DRAW_T_TOP 'w'
#define _NS_TERM_DRAW_VER 'x'

namespace ns_termcontrol {
    extern int termRows;
    extern int termCols;
    void initTerminal(bool clear);
    void disableCursorMovement();
    void setBrowseArea(int fromLine, int toLine);
    void setCursorPosition(int row, int col);
    void drawHorizontal(char startSymbol, char endSymbol, char midSymbol, std::string title, int totalLength);
    void drawRectangle(int topLeftRow, int topLeftCol, int botRightRow, int botRightCol, std::string title);
}

#endif // #ifndef _NS_CONSOLE_H_
