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
#include <stdio.h>

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
int drs_main(int argc, char **argv);
int can_broadcast_main(int argc, char **argv);
int safing_main(int argc, char **argv);
int etb_main(int argc, char **argv);

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
  //int ret;
  //int count = 0;
  //struct safing_status_s safing_status;
  
  task_create("drs",
              100,
              2048,
              drs_main,
              NULL);
  task_create("safing",
              CONFIG_SYSTEM_NSH_PRIORITY,
              2048,
              safing_main,
              NULL);
  task_create("can_broadcast",
              CONFIG_SYSTEM_NSH_PRIORITY,
              2048,
              can_broadcast_main,
              NULL
              );
  task_create("etb",
              CONFIG_SYSTEM_NSH_PRIORITY,
              2048,
              etb_main,
              NULL);
  /*
  do
    {
      safing_try_step(SAFING_STATE_ONBOARD_PROVEOUT);
      safing_get_status(&safing_status);
      ++count;
      
      switch (safing_status.state)
      {
        case SAFING_STATE_SOFT_FAULT:
        case SAFING_STATE_HARD_FAULT:
          printf("Safing fault; cannot arm.\n");
          goto start_nsh;
        
        case SAFING_STATE_PAUSED:
          if (safing_status.reason == SAFING_PAUSED_ARM_FAILED)
          {
            if (count == 3)
              {
                safing_trigger_soft_fault();
              }
            else
              {
                printf("Failed to arm; trying again. Fault flags = %x\n",
                      (unsigned int)safing_status.fault_flags);
              }
          }
        case SAFING_STATE_ONBOARD_PROVEOUT:
          break;
        default:
          safing_trigger_soft_fault();
      }
    }
  while (ret < 0);
  
  count = 0;
  do
    {
      ret = safing_try_step(SAFING_STATE_BSPD_PROVEOUT);
      ++count;
      
      if (ret < 0)
      {
        printf("Failed to 
  */
  return OK;
}
