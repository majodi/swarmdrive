#include "ns_console.h"

using std::string;
using std::vector;
using std::deque;
using std::cin;
using namespace ns_termcontrol;

namespace ns_console {

	static const int commandLines = 3;
	static const int commandItemWidth = 16;
	static const int commandItemType = 1;
	static const int parameterLines = 3;
	static const int parameterItemWidth = 25;
	static const int parameterItemType = 2;
	static const int maxHistory = 5;

	struct consoleItemStruct					// struct for console items commands or parameters
	{
		int identifier;
		string shortcut;
		string hrName;
		int value;
	};

	QueueHandle_t toConsoleMessageQ = NULL;		// handle of MessageQ TO console
	QueueHandle_t fromConsoleMessageQ = NULL;	// handle of MessageQ FROM console
	bool guiMode = false;
	bool noIO = false;
	bool noUI = false;

	/**************************************************************************/
	/*!
		@brief  log string to log area of console (or to terminal
				if not in GUI mode)
				

		@params[in]
					item to log
		@returns
					none
	*/
	/**************************************************************************/
	static void logToConsole(string logItem) {
		if (!guiMode) {
			printf("-->%s\n", logItem.c_str());
			return;
		}
        printf(_NS_TERM_SAV_POS);
		fflush(stdout);
		setCursorPosition(termRows - 3, 2);
		printf("%s\n", logItem.c_str());
		setCursorPosition(termRows - 3, 1);
		drawHorizontal(_NS_TERM_DRAW_VER, _NS_TERM_DRAW_VER, ' ', "", termCols);
        printf(_NS_TERM_RES_POS);
		fflush(stdout);
	}

	/**************************************************************************/
	/*!
		@brief  initialize the screen, if screen at least 14 rows and 24 cols
				use GUI else simple CLI

		@params[in]
					none
		@returns
					none
	*/
	/**************************************************************************/
	static void initScreen() {
		initTerminal(true);
		if(termRows >= 14 && termCols >= 24) guiMode = true;
		if(guiMode) {
			drawRectangle(1, 1, termRows, termCols, "Commands");
			setCursorPosition(1 + commandLines + 1, 0);
			drawHorizontal(_NS_TERM_DRAW_T_LEFT, _NS_TERM_DRAW_T_RIGHT, _NS_TERM_DRAW_HOR, "Parameters", termCols);
			setCursorPosition(1 + commandLines + 1 + parameterLines + 1, 0);
			drawHorizontal(_NS_TERM_DRAW_T_LEFT, _NS_TERM_DRAW_T_RIGHT, _NS_TERM_DRAW_HOR, "Log", termCols);
			setCursorPosition(termRows - 2, 0);
			drawHorizontal(_NS_TERM_DRAW_T_LEFT, _NS_TERM_DRAW_T_RIGHT, _NS_TERM_DRAW_HOR, "Command", termCols);
			setBrowseArea(1 + commandLines + 1 + parameterLines + 1 + 1, termRows - 3);
			logToConsole("Console started in GUI mode.");
			logToConsole("Up/Dwn arrow for command history.");
			setCursorPosition(termRows - 1, 2);
		} else {
			printf("Console started in -non- GUI mode.\n");
			printf("Use command ci for console info.\n");
			printf("Use command ti for task info.\n");
			printf("Up/Dwn arrow for command history.\n");
		}
	}

	/**************************************************************************/
	/*!
		@brief  add new item to command/parameter array
				
		@params[in]
					the vector array, console message
		@returns
					none
	*/
	/**************************************************************************/
	static void addItemToArray(vector<consoleItemStruct>& itemArray, consoleMessageStruct qMessage) {
		consoleItemStruct newItem;
		newItem.identifier = qMessage.identifier;
		newItem.shortcut = qMessage.shortcut;
		newItem.hrName = qMessage.hrName;
		newItem.value = qMessage.value;
		itemArray.push_back(newItem);
	}

	/**************************************************************************/
	/*!
		@brief  int to std::string
				
		@params[in]
					integer
		@returns
					std::string
	*/
	/**************************************************************************/
	static std::string to_string( int x ) {
		int length = snprintf( NULL, 0, "%d", x );
		if(!(length >= 0)) return "undefined";
		char* buf = new char[length + 1];
		snprintf( buf, length + 1, "%d", x );
		std::string str( buf );
		delete[] buf;
		return str;
	}

	/**************************************************************************/
	/*!
		@brief  trim function
				

		@params[in]
					string
		@returns
					trimmed string
	*/
	/**************************************************************************/
	std::string& ltrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
	{
		str.erase(0, str.find_first_not_of(chars));
		return str;
	}
	
	std::string& rtrim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
	{
		str.erase(str.find_last_not_of(chars) + 1);
		return str;
	}
	
	std::string& trim(std::string& str, const std::string& chars = "\t\n\v\f\r ")
	{
		return ltrim(rtrim(str, chars), chars);
	}

	/**************************************************************************/
	/*!
		@brief  std::string to int
				
		@params[in]
					string
		@returns
					int
	*/
	/**************************************************************************/
	static int to_int( string x ) {
		int value;
		int check_one = sscanf(x.c_str(), "%d", &value);
		if(check_one != 1) return 0;
		return value;
	}

	/**************************************************************************/
	/*!
		@brief  check if item already exist in item array
				
		@params[in]
					the vector array, identifier
		@returns
					true or false
	*/
	/**************************************************************************/
	static bool consoleItemExists(vector<consoleItemStruct>& array, int identifier) {
		if(find_if(array.begin(), array.end(), [&](const consoleItemStruct & item) {return item.identifier == identifier;}) != array.end()) {
			return true;
		} else {
			return false;
		}
	}

	/**************************************************************************/
	/*!
		@brief  set item value
				
		@params[in]
					the vector array, identifier, value
		@returns
					true (success) or false (item not found)
	*/
	/**************************************************************************/
	static bool consoleItemSet(vector<consoleItemStruct>& array, int identifier, int value) {
		auto it = find_if(array.begin(), array.end(), [&](const consoleItemStruct & item) {return item.identifier == identifier;});
		if(it != array.end()) {
			array[it - array.begin()].value = value;
			return true;
		}
		return false;
	}

	/**************************************************************************/
	/*!
		@brief  set item value
				
		@params[in]
					the vector array, shortcut, value
		@returns
					true (success) or false (item not found)
	*/
	/**************************************************************************/
	static bool consoleItemSet(vector<consoleItemStruct>& array, string shortcut, int value) {
		auto it = find_if(array.begin(), array.end(), [&](const consoleItemStruct & item) {return item.shortcut == shortcut;});
		if(it != array.end()) {
			array[it - array.begin()].value = value;
			return true;
		}
		return false;
	}

	/**************************************************************************/
	/*!
		@brief  get item identifier by shortcut
				
		@params[in]
					the vector array, shortcut
		@returns
					int identifier
	*/
	/**************************************************************************/
	static consoleItemStruct consoleItemGetByShortcut(vector<consoleItemStruct>& array, string shortcut) {
		auto it = find_if(array.begin(), array.end(), [&](const consoleItemStruct & item) {return item.shortcut == shortcut;});
		if(it != array.end()) {
			return array[it - array.begin()];
		}
		consoleItemStruct notFound;
		notFound.hrName = "not found";
		return notFound;
	}

	/**************************************************************************/
	/*!
		@brief  handle incoming message
				
		@params[in]
					the qMessage to handle, the console arrays
		@returns
					none
	*/
	/**************************************************************************/
	static void handleQMessage(consoleMessageStruct qMessage, vector<consoleItemStruct>& commandArray, vector<consoleItemStruct>& parameterArray, bool &refresh) {
		switch (qMessage.messageType)
		{
		case _NS_REG_COMMAND: {
				if(!consoleItemExists(commandArray, qMessage.identifier)) {
					addItemToArray(commandArray, qMessage);
					refresh = guiMode ? true : false;
				} else {
					logToConsole("register command failed: " + to_string(qMessage.identifier));
				}
				break;
			}
		case _NS_REG_PARAMETER: {
				if(!consoleItemExists(parameterArray, qMessage.identifier)) {
					addItemToArray(parameterArray, qMessage);
					refresh = guiMode ? true : false;
				} else {
					logToConsole("register parameter failed: " + to_string(qMessage.identifier));
				}
				break;
			}
		case _NS_SET_PARAMETER: {
				if(!consoleItemSet(parameterArray, qMessage.identifier, qMessage.value)) {
					logToConsole("set failed: " + to_string(qMessage.identifier));
				} else {
					refresh = guiMode ? true : false;
				}
				break;
			}
		case _NS_LOG_MESSAGE: {
				string shortMessage = qMessage.hrName;
				logToConsole(shortMessage);
				break;
			}
		default:
			break;
		}
	}

	/**************************************************************************/
	/*!
		@brief  send message from console
				
		@params[in]
					messageType (see defined message types)
					identifier
					value
					shortcut
					human readable name
		@returns
					none
	*/
	/**************************************************************************/
	static void sendFromConsole(int messageType, int identifier, int value, string shortcut, string hrName) {
		consoleMessageStruct message;
		message.messageType = messageType;
		message.identifier = identifier;
		strncpy(message.shortcut, shortcut.c_str(), 2);
		message.shortcut[2] = '\0';
		strncpy(message.hrName, hrName.c_str(), 10);
		message.hrName[10] = '\0';
		message.value = value;
		xQueueSend(fromConsoleMessageQ, ( void * ) &message, ( TickType_t ) 0 );
	}

	/**************************************************************************/
	/*!
		@brief	print task info
				
		@params[in]
					none
		@returns
					none
	*/
	/**************************************************************************/
	static void printTaskInfo() {
		printf( "Task Name\tStatus\tPrio\tHWM\tTask\tAffinity\n");
		char stats_buffer[1024];
		vTaskList(stats_buffer);
		printf("%s\n\n", stats_buffer);
		printf("Console on core: %d\n", xPortGetCoreID());
		printf("Free heap size: %d\n\n", esp_get_free_heap_size());
	}

	/**************************************************************************/
	/*!
		@brief  handle keyboard entered command
				
		@params[in]
					the commandline to handle (will be cleared), the console arrays
		@returns
					none
	*/
	/**************************************************************************/
	static void handleCommandLine(string &commandLine, vector<consoleItemStruct>& commandArray, vector<consoleItemStruct>& parameterArray, bool &refresh) {
		int equalSign = commandLine.find('=');
		if(equalSign != string::npos) {
			string commandShortcut = commandLine.substr(0, equalSign);
			commandShortcut = trim(commandShortcut);
			int setValue = to_int(commandLine.substr(equalSign + 1, 99));
			if(!consoleItemSet(parameterArray, commandShortcut, setValue)) {
				logToConsole("failed " + commandLine.substr(equalSign + 1, 99));
			} else {
				consoleItemStruct item = consoleItemGetByShortcut(parameterArray, commandShortcut);
				sendFromConsole(_NS_SET_PARAMETER, item.identifier, setValue, commandShortcut, item.hrName);
				refresh = true;
			}
		} else {
			string commandShortcut = trim(commandLine);
			if (guiMode == false) {
				if (commandShortcut == "ti") {
					printTaskInfo();
				}
				if (commandShortcut == "ci") {
					refresh = true;
				}
				if (commandShortcut == "re") {
					esp_restart();
				}
				if (commandShortcut == "gui") {
					enableCursorMovement();
					noUI = false;
					guiMode = true;
					initScreen();
					refresh = true;
				}
			} else {
				if (commandShortcut == "nogui") {
					noUI = true;
					guiMode = false;
					initTerminal(true);
					disableCursorMovement();
				}
			}
			auto it = find_if(commandArray.begin(), commandArray.end(), [&](const consoleItemStruct & item) {return item.shortcut == commandShortcut;});
			if(it != commandArray.end()) {
				logToConsole("Send command " + commandArray[it - commandArray.begin()].hrName);
				sendFromConsole(_NS_COMMAND, commandArray[it - commandArray.begin()].identifier, 0, "", "");
			}
		}
		commandLine = "";
	}

	/**************************************************************************/
	/*!
		@brief  print command/parameter item to console window
				
		@params[in]
					type (commandItemType/parameterItemType)
					item width
					row
					col
					item
		@returns
					none
	*/
	/**************************************************************************/
	static void printConsoleItem(int type, int itemWidth, int &row, int &col, consoleItemStruct item) {
		char buffer[itemWidth];
		int maxLine = type == commandItemType ? 1 + commandLines + 1 : 1 + commandLines + 1 + parameterLines + 1;
		if(row < maxLine) {
			if(type == commandItemType) {
				snprintf(buffer, itemWidth, "%s (%s)", item.hrName.c_str(), item.shortcut.c_str());
			} else {
				snprintf(buffer, itemWidth, "%s (%s): %d", item.hrName.c_str(), item.shortcut.c_str(), item.value);
			}
			setCursorPosition(row, col);
			printf("%-*s", itemWidth, buffer);
			fflush(stdout);
			col += itemWidth;
			if((col + itemWidth + 1) > termCols) {
				row++;
				col = 2;
			} else {col += itemWidth;}
		}
	}

	/**************************************************************************/
	/*!
		@brief  refresh console display or print available commands/parameters
				to terminal if not in GUI mode
				
		@params[in]
					the refresh (update) screen with current
					command/parameter contents
		@returns
					none
	*/
	/**************************************************************************/
	static void refreshDisplay(bool & refresh, vector<consoleItemStruct> commandArray, vector<consoleItemStruct> parameterArray) {
		int row, col;
		printf(_NS_TERM_SAV_POS);
		row = 2; col = 2;
		for_each(commandArray.begin(), commandArray.end(), [&](const consoleItemStruct& command) {
			if (guiMode) {
				printConsoleItem(commandItemType, commandItemWidth, row, col, command);
			} else {
				printf("%s (%s)\n", command.hrName.c_str(), command.shortcut.c_str());
			}
		});
		row = 6; col = 2;
		for_each(parameterArray.begin(), parameterArray.end(), [&](const consoleItemStruct& parameter) {
			if (guiMode) {
				printConsoleItem(parameterItemType, parameterItemWidth, row, col, parameter);
			} else {
				printf("%s (%s): %d\n", parameter.hrName.c_str(), parameter.shortcut.c_str(), parameter.value);
			}
		});
		if (guiMode) printf(_NS_TERM_RES_POS);
		fflush(stdout);
		refresh = false;
	}

	/**************************************************************************/
	/*!
		@brief  read possible new char from keyboard and put in commandLine
				
		@params[in]
					the commandLine so far
		@returns
					true on commandLine accepted (enter) else false
	*/
	/**************************************************************************/
	static bool readCommandLine(string &commandLine, deque<string>& commandHistory, int& historyIdx) {
		int nextChar = getchar();
		if(nextChar != EOF) {
			if (nextChar == '\n' && commandLine != "") {
				if(commandHistory.size() >= maxHistory) {
					commandHistory.pop_back();
				}
				commandHistory.push_front(commandLine);
				historyIdx = 0;
				logToConsole(commandLine);
				cin.clear();
				cin.ignore(INT_MAX);
				setCursorPosition(termRows - 1, 1);
				drawHorizontal(_NS_TERM_DRAW_VER, _NS_TERM_DRAW_VER, ' ', "", termCols);
				return true;
			} else if(nextChar == '\b') {
				commandLine = commandLine.substr(0, commandLine.length()-1);
				putchar(nextChar);
				putchar(' ');
				putchar(nextChar);
				fflush(stdout);
			} else if(nextChar == 27) {
				bool specialKey = false;
				nextChar = getchar();
				if(nextChar == 91) {
					nextChar = getchar();
					if (nextChar == 65) {
						if(historyIdx == 0 && ((commandHistory.size() == 0) || (commandHistory[0] != commandLine))) {
							if(commandHistory.size() >= maxHistory) {
								commandHistory.pop_back();
							}
							commandHistory.push_front(commandLine);
						}
						historyIdx = historyIdx >= maxHistory ? historyIdx : historyIdx + 1;
						specialKey = true;
					}
					if (nextChar == 66) {
						historyIdx = historyIdx <= 0 ? 0 : historyIdx - 1;
						specialKey = true;
					}
				}
				if (specialKey) {
					if (!(historyIdx > (commandHistory.size() - 1))) {
						commandLine = commandHistory[historyIdx];
						setCursorPosition(termRows - 1, 1);
						drawHorizontal(_NS_TERM_DRAW_VER, _NS_TERM_DRAW_VER, ' ', "", termCols);
						setCursorPosition(termRows - 1, 2);
						printf("%s", commandLine.c_str());
					}
				} else {
					if (nextChar >= 32 && nextChar <= 126) {
						commandLine += nextChar;
						putchar(nextChar);
					} 
				}
			} else if(nextChar >= 32 && nextChar <= 126) {
				commandLine += nextChar;
				putchar(nextChar);
			}
		}
		return false;
	}

	/**************************************************************************/
	/*!
		@brief  console main entry point; scan keyboard, scan consoleMessageQ,
				handle messages, display. Should be used as Task

		@params[in]
					none
		@returns
					none
	*/
	/**************************************************************************/
	static void	consoleTask(void *arg) {
		consoleMessageStruct qMessage;
		vector<consoleItemStruct> commandArray;
		vector<consoleItemStruct> parameterArray;
		deque<string> commandHistory;
		int historyIdx = 0;
		bool refresh = false;
	    string commandLine;

		while(1){
			if (xQueueReceive(toConsoleMessageQ, &(qMessage), (TickType_t) 0) == pdPASS) {			// see if there is an outside message
				handleQMessage(qMessage, commandArray, parameterArray, refresh);
			}
			if (!noIO) {
				if (readCommandLine(commandLine, commandHistory, historyIdx)) {
					handleCommandLine(commandLine, commandArray, parameterArray, refresh);
					setCursorPosition(termRows - 1, 2);
				}
				if (refresh) refreshDisplay(refresh, commandArray, parameterArray);
			}
			vTaskDelay(1);
		}
	}

	/**************************************************************************/
	/*!
		@brief  initialize console

		@params[in]
					console options:
					_NS_CON_OPTION_NO_UI 1
					_NS_CON_OPTION_NO_IO 2
		@returns
					none
	*/
	/**************************************************************************/
	void initConsole(int consoleOption) {
		noIO = consoleOption == _NS_CON_OPTION_NO_IO ? true : false;
		noUI = consoleOption == _NS_CON_OPTION_NO_UI ? true : false;
		if (consoleOption == 0) initScreen();								// GUI mode? do init screen
		else {
			disableCursorMovement();										// else disable cursor movement
			if (noIO) {														// if noIO set
				printf("Console started in forced non I/O mode.\n");		// let user know
			} else if (noUI) {												// if noUI set
				printf("Console started in forced non GUI mode.\n");		// let user know
				printf("Use command ci for console info.\n");				// and give info on usage
				printf("Use command ti for task info.\n");
				printf("Up/Dwn arrow for command history.\n");
			}
		}
		toConsoleMessageQ = xQueueCreate(20, sizeof( consoleMessageStruct ) );		// console receiveQ
		fromConsoleMessageQ = xQueueCreate(20, sizeof( consoleMessageStruct ) );	// console sendQ
		xTaskCreatePinnedToCore(consoleTask, "consoleTask", 4096, NULL, tskIDLE_PRIORITY, NULL, 0);	// start console task
	}

	/**************************************************************************/
	/*!
		@brief  send message to console

		@params[in]
					messageType (see defined message types)
					identifier
					value
					shortcut
					human readable name
		@returns
					none
	*/
	/**************************************************************************/
	void sendToConsole(int messageType, int identifier, int value, string shortcut, string hrName) {
		consoleMessageStruct message;
		if (toConsoleMessageQ == NULL) return;
		message.messageType = messageType;
		message.identifier = identifier;
		strncpy(message.shortcut, shortcut.c_str(), 2);
		message.shortcut[2] = '\0';
		strncpy(message.hrName, hrName.c_str(), 10);
		message.hrName[10] = '\0';
		message.value = value;
		xQueueSend(toConsoleMessageQ, ( void * ) &message, ( TickType_t ) 0 );
	}

	/**************************************************************************/
	/*!
		@brief  let console log short message (with id and value if needed)

		@params[in]
					message (max 10 chars)
					identifier
					value
		@returns
					none
	*/
	/**************************************************************************/
	void consoleLogMessage(string message, int id, int value) {
		consoleMessageStruct logMessage;
		if (toConsoleMessageQ == NULL) return;
		logMessage.messageType = _NS_LOG_MESSAGE;
		logMessage.identifier = id;
		logMessage.shortcut[0] = '\0';
		strncpy(logMessage.hrName, message.c_str(), 10);
		logMessage.hrName[10] = '\0';
		logMessage.value = value;
		xQueueSend(toConsoleMessageQ, ( void * ) &logMessage, ( TickType_t ) 0 );
	}

	/**************************************************************************/
	/*!
		@brief  get new console message if available (if not an empty message is returned)

		@params[in]

		@returns
				qMessage
	*/
	/**************************************************************************/
	consoleMessageStruct availableConsoleMessage() {
		consoleMessageStruct qMessage;
		if (toConsoleMessageQ == NULL) return {0, 0, {0}, {0}, 0};
		if(xQueueReceive(fromConsoleMessageQ, &(qMessage), (TickType_t) 0) == pdPASS)			// see if there is a console message
			return qMessage;
		else
			return {0, 0, {0}, {0}, 0};
	}

} // namespace ns_console
