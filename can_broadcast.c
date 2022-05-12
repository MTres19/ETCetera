/****************************************************************************
 * apps/industry/ETCetera/can_broadcast.c
 * Electronic Throttle Controller program - CAN Broadcast service
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
#include <nuttx/can/can.h>
#include <sys/boardctl.h>
#include <mqueue.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include "can_broadcast.h"
#include "safing.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/


/****************************************************************************
 * Private Types
 ****************************************************************************/


/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void broadcast_sigusr1_sigaction(int signo, FAR siginfo_t *siginfo, FAR void *context);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *tx_mqueue_names[CAN_NUM_TX_MQUEUES] = {
  CAN_DRS_TX_MQUEUE_NAME,
  CAN_SAFING_TX_MQUEUE_NAME
};

static char *rx_mqueue_names[CAN_NUM_RX_MQUEUES] = {
  CAN_DRS_RX_MQUEUE_NAME
};

static int g_canfd;

static struct sigevent g_mq_sigusr1event = {
  .sigev_notify = SIGEV_SIGNAL,
  .sigev_signo = SIGUSR1
};

/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void broadcast_sigusr1_sigaction(int signo, FAR siginfo_t *siginfo, FAR void *context)
{
  int ret;
  mqd_t *tx_mqueue;
  struct can_msg_s msg;
  tx_mqueue = (mqd_t *)(siginfo->si_value.sival_ptr);
  
  
  do
  {
    ret = mq_receive(*tx_mqueue, (char *)&msg, sizeof(msg), NULL);
    if (ret > 0)
      {
        write(g_canfd, &msg, CAN_MSGLEN(msg.cm_hdr.ch_dlc));
      }
  }
  while (ret > 0 || (ret < 0 && errno == EINTR));
  
  /* Reregister after handling it */
  g_mq_sigusr1event.sigev_value.sival_ptr = tx_mqueue;
  mq_notify(*tx_mqueue, &g_mq_sigusr1event);
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
  int ret;
  int i;
  struct can_msg_s rxbuf;
  struct can_msg_s *rxptr;
  mqd_t tx_mqueues[CAN_NUM_TX_MQUEUES];
  mqd_t rx_mqueues[CAN_NUM_RX_MQUEUES];
  const struct mq_attr canmq_attr =
    { .mq_maxmsg = 3, .mq_msgsize = sizeof(struct can_msg_s) };
  
  g_canfd = open("/dev/can0", O_RDWR);
  if (g_canfd < 0)
  {
    safing_store_internal_fault(FAULT_CAN_OPEN_FAILED);
    return -1;
  }
  
  
  /* Initialize tx message queues and attach to SIGUSR1 */
  
  for (i = 0; i < CAN_NUM_TX_MQUEUES; ++i)
  {
    tx_mqueues[i] = mq_open(tx_mqueue_names[i], O_RDWR | O_NONBLOCK | O_CREAT, 0600, &canmq_attr);
    g_mq_sigusr1event.sigev_value.sival_ptr = &tx_mqueues[i];
    mq_notify(tx_mqueues[i], &g_mq_sigusr1event);
  }
  
  /* Register SIGUSR1 handler */
  struct sigaction sigusr1_action = {
    .sa_sigaction = &broadcast_sigusr1_sigaction,
    .sa_flags = SA_SIGINFO
  };
  sigemptyset(&sigusr1_action.sa_mask);
  sigaction(SIGUSR1, &sigusr1_action, NULL);
  
  /* Initialize rx message queues */
  for (i = 0; i < CAN_NUM_RX_MQUEUES; ++i)
  {
    rx_mqueues[i] = mq_open(rx_mqueue_names[i], O_RDWR | O_NONBLOCK | O_CREAT, 0600, &canmq_attr);
  }
  
  do
    {
      ret = read(g_canfd, &rxbuf, sizeof(rxbuf));
      if (ret > 0)
        {
          rxptr = &rxbuf;
          do
            {
              if (rxptr->cm_hdr.ch_id == CAN_ID_DRS_CONTROL_RX)
                {
                  mq_send(rx_mqueues[CAN_DRS_RX_MQUEUE_IDX], (const char *)rxptr,
                              CAN_MSGLEN(rxptr->cm_hdr.ch_dlc),
                              1);
                }
              rxptr = (struct can_msg_s *)((uint8_t *)rxptr + CAN_MSGLEN(rxptr->cm_hdr.ch_dlc));
            } while (rxptr < &rxbuf + sizeof(rxbuf));
        }
    } while (true);
  
  return 0;
}
