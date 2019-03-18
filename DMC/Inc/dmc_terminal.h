#ifndef __DMC_TERMINAL_H
#define __DMC_TERMINAL_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

// https://misc.flogisoft.com/bash/tip_colors_and_formatting
#define TERMINAL_RESET  			"\e[0m"
#define TERMINAL_DEFAULT  			"\e[39m"
#define TERMINAL_RED      			"\e[31m"
#define TERMINAL_GREEN    			"\e[32m"
#define TERMINAL_YELLOW    			"\e[33m"
#define TERMINAL_BLUE    			"\e[34m"
#define TERMINAL_MAGENTA   			"\e[35m"
#define TERMINAL_CYAN    			"\e[36m"
#define TERMINAL_LIGHT_GRAY			"\e[37m"
#define TERMINAL_DARK_GRAY			"\e[90m"
#define TERMINAL_LIGHT_RED			"\e[91m"
#define TERMINAL_LIGHT_GREEN		"\e[92m"
#define TERMINAL_LIGHT_YELLOW		"\e[93m"
#define TERMINAL_LIGHT_BLUE			"\e[94m"
#define TERMINAL_LIGHT_MAGENTA		"\e[95m"
#define TERMINAL_LIGHT_CYAN			"\e[96m"
#define TERMINAL_WHITE				"\e[97m"
#define TERMINAL_BOLD     			"\e[1m"
#define TERMINAL_DIM      			"\e[2m"
#define TERMINAL_UNDERLINE			"\e[4m"
#define TERMINAL_BLINK				"\e[5m"
#define TERMINAL_REVERSE			"\e[7m"
#define TERMINAL_NO_BOLD     		"\e[21m"
#define TERMINAL_NO_DIM      		"\e[22m"
#define TERMINAL_NO_UNDERLINE		"\e[24m"
#define TERMINAL_NO_BLINK			"\e[25m"
#define TERMINAL_NO_REVERSE			"\e[27m"
#define TERMINAL_BG_DEFAULT			"\e[49m"
#define TERMINAL_BG_BLACK			"\e[40m"
#define TERMINAL_BG_RED				"\e[41m"
#define TERMINAL_BG_GREEN			"\e[42m"
#define TERMINAL_BG_YELLOW			"\e[43m"
#define TERMINAL_BG_BLUE			"\e[44m"
#define TERMINAL_BG_MAGENTA			"\e[45m"
#define TERMINAL_BG_CYAN			"\e[46m"
#define TERMINAL_BG_LIGHT_GRAY		"\e[47m"
#define TERMINAL_BG_DARK_GRAY		"\e[100m"
#define TERMINAL_BG_LIGHT_RED		"\e[101m"
#define TERMINAL_BG_LIGHT_GREEN		"\e[102m"
#define TERMINAL_BG_LIGHT_YELLOW	"\e[103m"
#define TERMINAL_BG_LIGHT_BLUE		"\e[104m"
#define TERMINAL_BG_LIGHT_MAGENTA	"\e[105m"
#define TERMINAL_BG_LIGHT_CYAN		"\e[106m"
#define TERMINAL_BG_WHITE			"\e[107m"

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* __DMC_TERMINAL_H */

