/****************************************************************************
 * apps/industry/ETCetera/etb.c
 * Electronic Throttle Controller program - ETB Control
 *
 * Copyright (C) 2022  Matthew Trescott <matthewtrescott@gmail.com>
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
#include <nuttx/can/can.h>
#include <sys/boardctl.h>
#include <mqueue.h>
#include <nuttx/can/can.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <nuttx/clock.h>
#include <errno.h>
#include <arch/board/board.h>
#include <semaphore.h>

#include "can_broadcast.h"
#include "safing.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPRING_TABLE_SIZE 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spring_table_s
{
  int16_t tps[SPRING_TABLE_SIZE];
  uint16_t duty[SPRING_TABLE_SIZE];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/


/****************************************************************************
 * Private Data
 ****************************************************************************/

static int16_t *g_tps1;
static int16_t *g_tps2;
static uint8_t g_frozen_channels;
static sem_t g_tps_avg_sem;


/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void etb_sigstop_sigaction(int signo, FAR siginfo_t *siginfo, FAR void *context)
{
  g_frozen_channels = siginfo->si_value.sival_int;
  
  if (g_frozen_channels & (TPS1_FROZEN | TPS2_FROZEN))
  {
    while (sem_trywait(&g_tps_avg_sem) == OK)
    {};
  }
}

static void etb_sigcont_sigaction(int signo, FAR siginfo_t *siginfo, FAR void *context)
{
  g_frozen_channels = siginfo->si_value.sival_int;
  
  if (! (g_frozen_channels & (TPS1_FROZEN | TPS2_FROZEN)))
  {
    int sval;
    sval = 0;
    sem_getvalue(&g_tps_avg_sem, &sval);
    if (sval < 1)
      sem_post(&g_tps_avg_sem);
  }
}

static int16_t get_tps_any(void)
{
  if (!(g_frozen_channels & (TPS1_FROZEN | TPS2_FROZEN)))
  {
    return (*g_tps1 + *g_tps2) / 2;
  }
  else if (g_frozen_channels & TPS1_FROZEN)
  {
    return *g_tps2;
  }
  else
  {
    return *g_tps1;
  }
}

static uint16_t get_tps_average(void)
{
  sem_wait(&g_tps_avg_sem);
  
  return (*g_tps1 + *g_tps2) / 2;
}
  

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
  
  boardctl(BOARDIOC_ETB_DUTY, 100);

  //int16_t *apps1;
  //int16_t *apps2;
  int32_t tps;
  //int32_t apps;
  int16_t lhp; /* limp home position */
  int i;
  
  struct spring_table_s spring_table;
  
  struct sigaction sigstop_action = {
    .sa_sigaction = etb_sigstop_sigaction,
    .sa_flags = SA_SIGINFO
  };
  struct sigaction sigcont_action = {
    .sa_sigaction = etb_sigcont_sigaction,
    .sa_flags = SA_SIGINFO
  };
  sigemptyset(&sigstop_action.sa_mask);
  sigemptyset(&sigcont_action.sa_mask);
  
  sem_init(&g_tps_avg_sem, 0, 1);
  
  sigaction(SIGSTOP, &sigstop_action, NULL);
  sigaction(SIGCONT, &sigcont_action, NULL);
  
  struct chan_subscription_s subscr;
  
  subscr.tid = gettid();
  
  subscr.ptr = &g_tps1;
  boardctl(BOARDIOC_TPS1_SUBSCRIBE, (uintptr_t)&subscr);
  subscr.ptr = &g_tps2;
  boardctl(BOARDIOC_TPS2_SUBSCRIBE, (uintptr_t)&subscr);
  /*subscr.ptr = &apps1;
  boardctl(BOARDIOC_APPS1_SUBSCRIBE, (uintptr_t)&subscr);
  subscr.ptr = &apps2;
  boardctl(BOARDIOC_APPS2_SUBSCRIBE, (uintptr_t)&subscr);
  */
  boardctl(BOARDIOC_RELAY_ENABLE, 0);
  
  /* Wait for shutdown circuit to arm */
  usleep(200000);
  
  tps = get_tps_average();
  //apps = (*apps1 + *apps2) /2;
  
  lhp = tps;
  
  do
  {
  
  usleep(50000);
  
  for (i = 100; i < 400; i = i + 2)
  {
    tps = get_tps_average();
    
    if (tps > lhp + 2500)
    {
      break;
    }
    else
    {
      boardctl(BOARDIOC_ETB_DUTY, i);
      usleep(50000);
    }
  }
  
  if (i == 400)
  {
    boardctl(BOARDIOC_ETB_DUTY, 0);
    sleep(2);
    continue;
  }
  
  usleep(500000);
  
  if (tps > 8000)
  {
    boardctl(BOARDIOC_ETB_DUTY, 0);
    sleep(2);
    continue;
  }
  
  uint16_t test_duty;
  test_duty = i;
  
  /* Return to zero so we can reliably reproduce the test position quickly */
  boardctl(BOARDIOC_ETB_DUTY, 0);
  usleep(500000); // Ensure ETB is at rest
  
  boardctl(BOARDIOC_ETB_DUTY, test_duty);
  sleep(2); // Ensure ETB is at rest
  
  int16_t test_pos;
  tps = get_tps_average();
  test_pos = tps;
  
  for (i = test_duty; i > 0; --i)
  {
    boardctl(BOARDIOC_ETB_DUTY, i);
    usleep(50000);
    tps = get_tps_average();
    if (tps < test_pos - 50)
    {
      break;
    }
  }
  
  if (i == 0)
  {
    sleep(2);
    continue;
  }
  
  uint16_t test_duty_lower = i;
  
  boardctl(BOARDIOC_ETB_DUTY, 0);
  usleep(500000); // Ensure ETB is at rest
  
  boardctl(BOARDIOC_ETB_DUTY, test_duty);
  sleep(2);
  
  tps = get_tps_average();
  if (tps > test_pos + 20 || tps < test_pos - 20)
  {
    boardctl(BOARDIOC_ETB_DUTY, 0);
    sleep(2);
    continue;
    //return -EIO;
  }
  
  test_pos = get_tps_average();
  
  for (i = test_duty; i < 600; ++i)
  {
    boardctl(BOARDIOC_ETB_DUTY, i);
    usleep(50000);
    tps = get_tps_average();
    if (tps > test_pos + 50)
    {
      break;
    }
  }
  
  if (i == 600)
  {
    boardctl(BOARDIOC_ETB_DUTY, 0);
    sleep(2);
    continue;
  }
  
  uint16_t test_duty_upper = i;
  
  int16_t static_friction = (test_duty_upper - test_duty_lower) / 2;
  
  /* Slowly bring the valve to UMS (upper mechanical stop) */
  int16_t last_tps;
  
  for (i = test_duty; i < 600; ++i)
  {
    last_tps = tps;
    boardctl(BOARDIOC_ETB_DUTY, i);
    usleep(50000);
    tps = get_tps_average();
    if (tps > 10000 && tps < last_tps + 20)
    {
      break;
    }
  }
  
  if (i == 600)
  {
    boardctl(BOARDIOC_ETB_DUTY, 0);
    sleep(2);
    continue;
  }
  
  /* Slowly lower the valve down from UMS; the input torque at which it breaks
   * loose and starts to move is equal to spring torque minus static friction.
   * Record the first spring torque in the table. Every time the ETB moves
   * thereafter, record a new point in the table if the torque is "significantly"
   * different than the previous one (we'll use 1% duty cycle).
   */
  last_tps = tps;
  uint16_t last_duty;
  last_duty = i;
  int spring_table_idx = SPRING_TABLE_SIZE;
  for (/* i already set */ ; i > 0 && spring_table_idx >= 0; --i)
  {
    boardctl(BOARDIOC_ETB_DUTY, i);
    usleep(50000);
    tps = get_tps_average();
    
    if (tps < last_tps - 20)
    {
      /* The ETB moved */
      if (spring_table_idx == SPRING_TABLE_SIZE || last_duty - i >= 10)
      {
        spring_table.tps[spring_table_idx] = last_tps;
        spring_table.duty[spring_table_idx] = i + static_friction;
        --spring_table_idx;
        last_duty = i;
      }
      
      last_tps = tps;
    }
  }
  
  while (true)
  {
    usleep(10000);
    
    boardctl(BOARDIOC_ETB_DUTY, 100);
  }
  
  } while(true);
  
  return 0;
}
 
