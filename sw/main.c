#include "cfg_general.h"
#include "hw_terminal.h"
#include "errors.h"
#include "cli.h"
#include "hw_general.h"

int main()
{
	CHK_PRINT(term_init());
	CHK_PRINT(hw_init());
	CHK_PRINT(cfg_init());

	while (1) {
		CHK_PRINT(cli_get_cmd());
		CHK_PRINT(cli_handle_cmd());
	}

	return 0;
}

#ifndef PC_COMPILATION
void UNDEF_Routine(){}
void SWI_Routine(){}
void FIQ_Routine(){}
#endif

