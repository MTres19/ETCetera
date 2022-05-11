/****************************************************************************
 * apps/industry/ETCetera/safing.c
 * Electronic Throttle Controller program - safing state machine
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
#include "safing.h"
#include <sys/boardctl.h>
#include <errno.h>
#include <arch/board/board.h>
#include <unistd.h>
#include <mqueue.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>

#include "can_broadcast.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fault_entry_s
{
  uint32_t time_ms;
  uint16_t fault_code;
  uint8_t  keycycle;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int safing_check_sensor_ranges(void);
static void safing_subscription_update_dtcs_and_faults(void);
static void safing_sigusr1_handler(int signo);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define SAFING_NUM_DTC_ENTRIES 16
static struct fault_entry_s g_dtc_table[SAFING_NUM_DTC_ENTRIES] = {0};
#define SAFING_NUM_FAULT_ENTRIES 16
static struct fault_entry_s g_fault_table[SAFING_NUM_FAULT_ENTRIES] = {0};

static struct safing_subscription_s g_safing_subscr;

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void safing_arm(void)
{
  int ret;
  
  g_safing_subscr.tid = gettid();
  g_safing_subscr.faultflags = 0;
  
  struct sigaction sigusr1_action = {
    .sa_handler = safing_sigusr1_handler,
    .sa_flags = 0
  };
  sigemptyset(&sigusr1_action.sa_mask);
  sigaction(SIGUSR1, &sigusr1_action, NULL);
  
  boardctl(BOARDIOC_SAFING_SUBSCRIBE, (uintptr_t)&g_safing_subscr);
  
  /* Enable 5V0LIN_SENSE and enable receiving a signal if it later fails */
  ret = boardctl(BOARDIOC_5V0LIN_SENSE_ARM, 0);
  
  if (ret < 0)
  {
    safing_store_dtc(DTC_5V0LIN_SENSE_STG);
    goto safing_failed;
  }
  
  /* Allow sensors to settle */
  usleep(50000);
  
  /* Software check of sensor ranges and correlation */
  ret = safing_check_sensor_ranges();
  if (ret < 0)
  {
    goto safing_failed;
  }
  
  /* Enable edge interrupts for hardware out-of-range, open line, and
   * out-of-correlation faults and verify that they are not already
   * signalling a fault to the latching logic.
   */
  ret = boardctl(BOARDIOC_HW_PLAUS_CK_ARM, 0);
  if (ret < 0)
  {
    goto safing_failed;
  }
  
  /* Wait 100 ms to ensure the open-line detection circuit does not
   * find any faults.
   */
  usleep(100000);
  
  /* Read arm/safing status (must be disarmed with safing active),
   * send arming signal, enable edge interrupts to detect unexpected
   * disarming or safing assertion, and verify that system is armed
   * and safing is inactive.
   */
  
  ret = boardctl(BOARDIOC_HW_SAFING_ARM, 0);
  if (ret < 0)
  {
    goto safing_failed;
  }
  
  return;
  
safing_failed:
  safing_subscription_update_dtcs_and_faults();
  safing_store_dtc(DTC_INITIAL_ARM_FAILED);
}

static void safing_sigusr1_handler(int signo)
{
  safing_subscription_update_dtcs_and_faults();
}

static void safing_subscription_update_dtcs_and_faults(void)
{
  if (g_safing_subscr.faultflags & SAFINGSIG_5V0LIN_SENSE_STG)
    safing_store_dtc(DTC_5V0LIN_SENSE_STG);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_STP_APPS1)
    safing_store_dtc(DTC_APPS1_STP);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_STP_APPS2)
    safing_store_dtc(DTC_APPS2_STP);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_STP_BRKF)
    safing_store_dtc(DTC_BRKF_STP);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_STP_BRKR)
    safing_store_dtc(DTC_BRKR_STP);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_STP_TPS)
    safing_store_dtc(DTC_TPS_STP);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_STP_AUX)
    safing_store_dtc(DTC_5VAUX_STP);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_APPS1)
    safing_store_dtc(DTC_APPS1_OPEN);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_APPS2)
    safing_store_dtc(DTC_APPS2_OPEN);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_BRKF)
    safing_store_dtc(DTC_BRKF_OPEN);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_BRKR)
    safing_store_dtc(DTC_BRKR_OPEN);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_TPS1)
    safing_store_dtc(DTC_TPS1_OPEN);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_TPS2)
    safing_store_dtc(DTC_TPS2_OPEN);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OOC_TPS)
    safing_store_dtc(DTC_TPS_OOC);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OOC_APPS)
    safing_store_dtc(DTC_APPS_OOC);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING1_DISARMING)
    safing_store_internal_fault(FAULT_UNEXPECTED_DISARM_1);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING1_ASSERTING)
    safing_store_internal_fault(FAULT_UNEXPECTED_SAFING_1);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING2_DISARMING)
    safing_store_internal_fault(FAULT_UNEXPECTED_DISARM_2);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING2_ASSERTING)
    safing_store_internal_fault(FAULT_UNEXPECTED_SAFING_2);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_OL_INTERNAL_FAULT)
    safing_store_internal_fault(FAULT_OL_INTERNAL_FAULT);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING1_ALREADYARMED)
    safing_store_internal_fault(FAULT_ALREADY_ARMED_1);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING2_ALREADYARMED)
    safing_store_internal_fault(FAULT_ALREADY_ARMED_2);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING1_NOT_ASSERTED)
    safing_store_internal_fault(FAULT_NOT_SAFING_1);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING2_NOT_ASSERTED)
    safing_store_internal_fault(FAULT_NOT_SAFING_2);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING1_ARM_FAILED)
    safing_store_internal_fault(FAULT_ARM_FAILED_1);
  
  if (g_safing_subscr.faultflags & SAFINGSIG_SAFING2_ARM_FAILED)
    safing_store_internal_fault(FAULT_ARM_FAILED_2);
  
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
  mqd_t txmq;
  struct can_msg_s txmsg = {{0}};
  const struct mq_attr canmq_attr =
    { .mq_maxmsg = 3, .mq_msgsize = sizeof(txmsg) };
  
  txmq = mq_open(CAN_SAFING_TX_MQUEUE_NAME, O_RDWR | O_CREAT | O_NONBLOCK, 0600, &canmq_attr);
  
  txmsg.cm_hdr.ch_extid = true;
  txmsg.cm_hdr.ch_dlc = 4;
  
  safing_arm();
  
  int i = 0;
  int j = 0;
  while(true)
  {
    if (g_fault_table[i].fault_code != FAULT_INVALID)
      {
        memcpy(&txmsg.cm_data, &g_fault_table[i], sizeof(struct fault_entry_s));
        
        mq_send(txmq, (const char *)&txmsg, CAN_MSGLEN(txmsg.cm_hdr.ch_dlc), 1);
      }
    if (g_dtc_table[j].fault_code != DTC_INVALID)
      {
        memcpy(&txmsg.cm_data, &g_dtc_table[j], sizeof(struct fault_entry_s));
        mq_send(txmq, (const char *)&txmsg, CAN_MSGLEN(txmsg.cm_hdr.ch_dlc), 1);
      }
    
    if (i == SAFING_NUM_FAULT_ENTRIES - 1)
      i = 0;
    else
      ++i;
    
    if (j == SAFING_NUM_DTC_ENTRIES - 1)
      j = 0;
    else
      ++j;
    
    usleep(50000);
  }
}


void safing_store_dtc(uint16_t dtc)
{
  int i;
  struct timespec current_time;
  
  clock_gettime(CLOCK_REALTIME, &current_time);
  
  for (i = 0; i < SAFING_NUM_DTC_ENTRIES; ++i)
  {
    if (g_dtc_table[i].fault_code == dtc)
      return;
  }
  for (i = 0; i < SAFING_NUM_DTC_ENTRIES; ++i)
  {
    if (g_dtc_table[i].fault_code == DTC_INVALID)
    {
      g_dtc_table[i].fault_code = dtc;
      g_dtc_table[i].time_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;
      return;
    }
  }
}

void safing_store_internal_fault(uint16_t fault_code)
{
  int i;
  struct timespec current_time;
  
  clock_gettime(CLOCK_REALTIME, &current_time);
  
  for (i = 0; i < SAFING_NUM_FAULT_ENTRIES; ++i)
  {
    if (g_fault_table[i].fault_code == fault_code)
      return;
  }
  for (i = 0; i < SAFING_NUM_FAULT_ENTRIES; ++i)
  {
    if (g_fault_table[i].fault_code == FAULT_INVALID)
    {
      g_fault_table[i].fault_code = fault_code;
      g_fault_table[i].time_ms = current_time.tv_sec * 1000 + current_time.tv_nsec / 1000000;
      return;
    }
  }
  
  safing_store_dtc(DTC_INTERNAL_FAULT);
}

int safing_check_sensor_ranges(void)
{
  return OK;
}


