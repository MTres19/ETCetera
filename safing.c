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
static void safing_5v0lin_sense_retry_handler(int signo);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#define SAFING_NUM_DTC_ENTRIES 16
static struct fault_entry_s g_dtc_table[SAFING_NUM_DTC_ENTRIES] = {0};
#define SAFING_NUM_FAULT_ENTRIES 16
static struct fault_entry_s g_fault_table[SAFING_NUM_FAULT_ENTRIES] = {0};

static struct safing_subscription_s g_safing_subscr;


static struct sigevent g_retry_5v0lin_sense_event = {
  .sigev_notify = SIGEV_SIGNAL,
  .sigev_signo = SIGUSR2,
  .sigev_value = {.sival_ptr = NULL},
};
static const struct itimerspec g_retry_5v0lin_sense_delay =  {
  .it_value = {
    .tv_sec = 0,
    .tv_nsec = 5 * NSEC_PER_MSEC
  },
  .it_interval = {0}
};


#define RETRY_5V0LIN_SENSE_NOT_RETRYING 0
#define RETRY_5V0LIN_SENSE_WAITING 1
#define RETRY_5V0LIN_SENSE_RETRYING 2
int g_retrying_5v0lin_sense = RETRY_5V0LIN_SENSE_NOT_RETRYING;
static timer_t g_retry_5v0lin_sense_timer;

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
  struct sigaction sigusr2_action = {
    .sa_handler = safing_5v0lin_sense_retry_handler,
    .sa_flags = 0
  };
  sigemptyset(&sigusr1_action.sa_mask);
  sigemptyset(&sigusr2_action.sa_mask);
  sigaction(SIGUSR1, &sigusr1_action, NULL);
  sigaction(SIGUSR2, &sigusr2_action, NULL);
  sigset_t normal_sigmask;
  sigset_t sleep_sigmask;
  sigfillset(&sleep_sigmask);
  
  boardctl(BOARDIOC_SAFING_SUBSCRIBE, (uintptr_t)&g_safing_subscr);
  
  /* Enable 5V0LIN_SENSE and enable receiving a signal if it later fails */
  ret = boardctl(BOARDIOC_5V0LIN_SENSE_ARM, 0);
  
  if (ret < 0)
  {
    safing_store_dtc(DTC_5V0LIN_SENSE_STG);
    goto safing_failed;
  }
  
  /* Allow sensors to settle */
  sigprocmask(SIG_SETMASK, &sleep_sigmask, &normal_sigmask);
  ret = usleep(50000);
  sigprocmask(SIG_SETMASK, &normal_sigmask, NULL);
  
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
   * find any faults. Disable signals while doing this; otherwise
   * usleep may return early.
   */
  sigprocmask(SIG_SETMASK, &sleep_sigmask, &normal_sigmask);
  ret = usleep(200000);
  sigprocmask(SIG_SETMASK, &normal_sigmask, NULL);
  
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

static void safing_5v0lin_sense_retry_handler(int signo)
{
  int ret = 0;
  
  if (g_retrying_5v0lin_sense == RETRY_5V0LIN_SENSE_WAITING)
  {
    g_retrying_5v0lin_sense = RETRY_5V0LIN_SENSE_RETRYING;
    boardctl(BOARDIOC_5V0LIN_SENSE_RETRY_ARM, 0);
    timer_settime(g_retry_5v0lin_sense_timer, 0, &g_retry_5v0lin_sense_delay, NULL);
  }
  else
  {
    g_retrying_5v0lin_sense = RETRY_5V0LIN_SENSE_NOT_RETRYING;
    ret = boardctl(BOARDIOC_5V0LIN_SENSE_RETRY_CHECK, 0);
    if (ret != 0)
    {
      safing_store_dtc(DTC_5V0LIN_SENSE_STG);
    }
  }
}

static void safing_subscription_update_dtcs_and_faults(void)
{
  if (g_safing_subscr.faultflags & SAFINGSIG_5V0LIN_SENSE_STG)
  {
    if (g_retrying_5v0lin_sense == RETRY_5V0LIN_SENSE_NOT_RETRYING)
    {
      g_retrying_5v0lin_sense = RETRY_5V0LIN_SENSE_WAITING;
      
      /* Get a callback signal in a few milliseconds */
      timer_settime(g_retry_5v0lin_sense_timer, 0, &g_retry_5v0lin_sense_delay, NULL);
    }
  }
  
  /* 5V0LIN_SENSE successfully reenabled */
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
  
  if (g_safing_subscr.faultflags & SAFINGSIG_DRSBCK_STG)
    safing_store_dtc(DTC_DRSBCK_STG);
  
}

static void copy_fault_entry(struct can_msg_s *dest, struct fault_entry_s *src)
{
  dest->cm_data[0] = (uint8_t)(src->fault_code >> 8);
  dest->cm_data[1] = (uint8_t)(src->fault_code & 0xff);
  dest->cm_data[2] = src->keycycle;
  dest->cm_data[3] = (uint8_t)(src->time_ms >> 24);
  dest->cm_data[4] = (uint8_t)(src->time_ms >> 16);
  dest->cm_data[5] = (uint8_t)(src->time_ms >> 8);
  dest->cm_data[6] = (uint8_t)(src->time_ms & 0xff);
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
  txmsg.cm_hdr.ch_dlc = 8;
  
  timer_create(CLOCK_REALTIME, &g_retry_5v0lin_sense_event, &g_retry_5v0lin_sense_timer);
  safing_arm();

  int16_t *brk_f_value;
  int16_t *brk_r_value;
  int16_t *ws1;
  int16_t *ws2;
  int16_t *ws3;
  int16_t *ws4;
  
  struct chan_subscription_s brk_subscription;
  brk_subscription.tid = gettid();
  
  brk_subscription.ptr = &brk_f_value;
  boardctl(BOARDIOC_BRK_F_SUBSCRIBE, (uintptr_t)&brk_subscription);
  
  brk_subscription.ptr = &brk_r_value;
  boardctl(BOARDIOC_BRK_R_SUBSCRIBE, (uintptr_t)&brk_subscription);
  
  boardctl(BOARDIOC_WS1_SUBSCRIBE, (uintptr_t)&ws1);
  boardctl(BOARDIOC_WS2_SUBSCRIBE, (uintptr_t)&ws2);
  boardctl(BOARDIOC_WS3_SUBSCRIBE, (uintptr_t)&ws3);
  boardctl(BOARDIOC_WS4_SUBSCRIBE, (uintptr_t)&ws4);
  
  int i = 0;
  int j = 0;
  while(true)
  {
    txmsg.cm_hdr.ch_extid = true;
    
    if (g_fault_table[i].fault_code != FAULT_INVALID)
      {
        txmsg.cm_hdr.ch_id = CAN_ID_FAULT_TX;
        copy_fault_entry(&txmsg, &g_fault_table[i]);
        mq_send(txmq, (const char *)&txmsg, CAN_MSGLEN(txmsg.cm_hdr.ch_dlc), 1);
      }
    if (g_dtc_table[j].fault_code != DTC_INVALID)
      {
        txmsg.cm_hdr.ch_id = CAN_ID_DTC_TX;
        copy_fault_entry(&txmsg, &g_dtc_table[j]);
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
    
    txmsg.cm_hdr.ch_id = CAN_ID_BRAKE_TX;
    txmsg.cm_hdr.ch_extid = false;
    txmsg.cm_data[0] = (*brk_f_value) >> 8;
    txmsg.cm_data[1] = (*brk_f_value) & 0xff;
    txmsg.cm_data[2] = (*brk_r_value) >> 8;
    txmsg.cm_data[3] = (*brk_r_value) & 0xff;
    mq_send(txmq, (const char *)&txmsg, CAN_MSGLEN(txmsg.cm_hdr.ch_dlc), 1);
    
    txmsg.cm_hdr.ch_id = CAN_ID_WS_TX;
    txmsg.cm_hdr.ch_extid = false;
    txmsg.cm_data[0] = (*ws1) >> 8;
    txmsg.cm_data[1] = (*ws1) & 0xff;
    txmsg.cm_data[2] = (*ws2) >> 8;
    txmsg.cm_data[3] = (*ws2) & 0xff;
    txmsg.cm_data[4] = (*ws3) >> 8;
    txmsg.cm_data[5] = (*ws3) & 0xff;
    txmsg.cm_data[6] = (*ws4) >> 8;
    txmsg.cm_data[7] = (*ws4) & 0xff;
    mq_send(txmq, (const char *)&txmsg, CAN_MSGLEN(txmsg.cm_hdr.ch_dlc), 1);
    
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
      break;
    }
  }
  
  safing_store_dtc(DTC_INTERNAL_FAULT);
}

int safing_check_sensor_ranges(void)
{
  return OK;
}


