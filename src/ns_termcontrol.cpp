#include <Arduino.h>
#include <string>
#include <iostream>
#include "ns_termcontrol.h"

using std::string;

namespace ns_termcontrol {

    int termRows = 20;
    int termCols = 60;
    bool cursorMovement = true;

    /**************************************************************************/
    /*!
        @brief  send string to terminal and wait for expected return char,
                return string received until the expected char (timeout can be set for the waiting),
                send string can be empty. 

        @params[in]
                    none
        @returns
                    string received
    */
    /**************************************************************************/
    string terminalReadUntil(string sendStr, char waitFor, int timeOut) {
        string retval = "";
        int c = 0;
        printf("%s",sendStr.c_str());
        int start = millis();
        while(!((millis() - start) > timeOut || c == waitFor)) {
            c = getc(stdin);
            if(c != EOF && c != waitFor) {
                retval.push_back(c);
            }
        }
        return retval;
    }

    /**************************************************************************/
    /*!
        @brief  set Rows (max) and Columns (max) of current terminal display

        @params[in]
                    none
        @returns
                    true (success) or false (failed)
    */
    /**************************************************************************/
    bool setRC(int &rows, int &cols) {
        if (!cursorMovement) return false;
        printf(_NS_TERM_SAV_POS);
        fflush(stdout);
        terminalReadUntil(_NS_TERM_GOTO_REP(999, 999), 91, 1000);
        string rc = terminalReadUntil("", 82, 500);
        printf(_NS_TERM_RES_POS);
        fflush(stdout);
        if(rc.length() < 4) return false;
        sscanf(rc.c_str(), "%d;%d", &rows, &cols);
        return true;
    }

    /**************************************************************************/
    /*!
        @brief  init terminal: set rows, cols and clear if needed

        @params[in]
                    clear true if clear screen required
        @returns
                    none
    */
    /**************************************************************************/
    void initTerminal(bool clear) {
        if(clear) printf(_NS_TERM_CLS);
        if(!setRC(termRows, termCols)) setRC(termRows, termCols); // one retry
        setBrowseArea(1, termRows);
    }

    /**************************************************************************/
    /*!
        @brief  disable cursor movement, avoid cursor repositioning when in
                non GUI mode

        @params[in]
                    none
        @returns
                    none
    */
    /**************************************************************************/
    void disableCursorMovement() {
        cursorMovement = false;
    }

    /**************************************************************************/
    /*!
        @brief  set browse area (area that will scroll on newline)

        @params[in]
                    fromLine to toLine
        @returns
                    none
    */
    /**************************************************************************/
    void setBrowseArea(int fromLine, int toLine) {
        if (!cursorMovement) return;
        char fromLineBuffer[10];
        char toLineBuffer[10];
        itoa(fromLine,fromLineBuffer,10);
        itoa(toLine,toLineBuffer,10);
        printf(_NS_TERM_PRINTF_SET_BA, fromLineBuffer, toLineBuffer);
        fflush(stdout);
    }

    /**************************************************************************/
    /*!
        @brief  set cursor position

        @params[in]
                    row, col
        @returns
                    none
    */
    /**************************************************************************/
    void setCursorPosition(int row, int col) {
        if (!cursorMovement) return;
        char rowBuffer[10];
        char colBuffer[10];
        itoa(row,rowBuffer,10);
        itoa(col,colBuffer,10);
        printf(_NS_TERM_PRINTF_GOTO, rowBuffer, colBuffer);
        fflush(stdout);
    }

    /**************************************************************************/
    /*!
        @brief  draw horizontal line (or character sequence), it needs to
                have cursor movement allowed

        @params[in]
                    startSymbol char
                    endSymbol char
                    midSymbol char
                    title
                    totalLength
        @returns
                    none
    */
    /**************************************************************************/
    void drawHorizontal(char startSymbol, char endSymbol, char midSymbol, string title, int totalLength) {
        if (!cursorMovement) return;
        int middleCount = totalLength - (startSymbol != 0 ? 1 : 0) - (endSymbol != 0 ? 1 : 0) - title.length() - (title != "" ? 2 : 0);
        if(middleCount < 0) return;
        printf(_NS_TERM_SET_LINE);
        printf("%c", startSymbol);
        if(title != "") {
            printf(_NS_TERM_SET_CHAR);
            printf("[%s", title.c_str());
            printf("]");
            printf(_NS_TERM_SET_LINE);
        }
        for (int i=0; i<middleCount; ++i) {
            printf("%c", midSymbol);
        }
        printf("%c", endSymbol);
        printf(_NS_TERM_SET_CHAR);
        fflush(stdout);
    }

    /**************************************************************************/
    /*!
        @brief  draw rectangle, it needs to
                have cursor movement allowed

        @params[in]
                    topLeftRow,
                    topLeftCol,
                    botRightRow,
                    botRightCol,
                    title
        @returns
                    none
    */
    /**************************************************************************/
    void drawRectangle(int topLeftRow, int topLeftCol, int botRightRow, int botRightCol, string title) {
        if (!cursorMovement) return;
        if(botRightRow - topLeftRow < 1) return;
        setCursorPosition(topLeftRow, topLeftCol);
        drawHorizontal(_NS_TERM_DRAW_TOP_CORNER_START, _NS_TERM_DRAW_TOP_CORNER_END, _NS_TERM_DRAW_HOR, title, termCols);
        printf("\n");
        printf(_NS_TERM_SET_LINE);
        for (int i=0; i<(botRightRow - topLeftRow)-1; i++) {
            printf("%c", _NS_TERM_DRAW_VER);
            printf("%s", string(termCols - 2, ' ').c_str());
            printf("%c", _NS_TERM_DRAW_VER);
            printf("\n");
        }
        printf(_NS_TERM_SET_CHAR);
        drawHorizontal(_NS_TERM_DRAW_BOT_CORNER_START, _NS_TERM_DRAW_BOT_CORNER_END, _NS_TERM_DRAW_HOR, "", termCols);
        fflush(stdout);
    }

} // namespace ns_termcontrol
