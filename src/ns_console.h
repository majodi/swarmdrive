/**************************************************************************/
/*!
    @file     ns_console.h
    @author   NickStick BV

    @section  HISTORY

    v0.0 - Start of work

    Library to add crude command console to embedded systems

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
#ifndef _NS_CONSOLE_H_
#define _NS_CONSOLE_H_

#define _NS_CON_OPTION_NO_UI 1
#define _NS_CON_OPTION_NO_IO 2

// 100 - message types
#define _NS_REG_COMMAND 100
#define _NS_REG_PARAMETER 101
#define _NS_SET_PARAMETER 102
#define _NS_LOG_MESSAGE 103
#define _NS_COMMAND 104

namespace ns_console {
	struct consoleMessageStruct			// struct for console messages
	{
		int messageType;
		int identifier;
		char shortcut[3];
		char hrName[11];
		int value;
	};

    void initConsole(int consoleOption = 0);
    void sendToConsole(int messageType, int identifier, int value = 0, std::string shortcut = "", std::string hrName = "");
	void consoleLogMessage(std::string message, int id = 0, int value = 0);
    consoleMessageStruct availableConsoleMessage();
}

#endif // #ifndef _NS_CONSOLE_H_
