/****************************************************************************
 * apps/industry/ETCetera/main.c
 * Electronic Throttle Controller program - main entry point
 *
 * Copyright (C) 2020  Matthew Trescott <matthewtrescott@gmail.com>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>

#include "nshlib/nshlib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Defined by the NSH example application */

int nsh_main(int argc, char **argv);

/****************************************************************************
 * Private Data
 ****************************************************************************/


/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 *
 * Description:
 *   Entry point for the ETCetera daemon. Starts a task to run NuttShell then
 *   continues on.
 *
 ****************************************************************************/

int main(int argc, char **argv)
{
  printf("ETCetera: starting NuttShell. Type \"help\" for commands.\n");
#ifdef CONFIG_SYSTEM_NSH
  task_create("nsh",
              CONFIG_SYSTEM_NSH_PRIORITY,
              CONFIG_SYSTEM_NSH_STACKSIZE,
              nsh_main,
              NULL
             );
#endif
  while (true);
}
