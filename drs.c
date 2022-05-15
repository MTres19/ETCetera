/****************************************************************************
 * apps/industry/ETCetera/drs.c
 * Electronic Throttle Controller program - DRS Control
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
  int ret = 0;
  struct can_msg_s txmsg;
  struct can_msg_s rxmsg;
  
  mqd_t txmq;
  mqd_t rxmq;
  
  struct timespec mq_timeout;
  const struct timespec canmq_wait_time =
    { .tv_sec = 0, .tv_nsec = 500 * NSEC_PER_MSEC };
  const struct mq_attr canmq_attr =
    { .mq_maxmsg = 3, .mq_msgsize = sizeof(rxmsg) };
  bool drs_powered = false;
  
  txmsg.cm_hdr.ch_id = CAN_ID_DRS_STATUS_TX;
  txmsg.cm_hdr.ch_extid = true;
  txmsg.cm_hdr.ch_dlc = 4;
  txmsg.cm_hdr.ch_rtr = 0;
#ifdef CONFIG_CAN_ERRORS
  txmsg.cm_hdr.ch_error = 0;
#endif
  
  txmq = mq_open(CAN_DRS_TX_MQUEUE_NAME, O_RDWR | O_CREAT | O_NONBLOCK, 0600, &canmq_attr);
  rxmq = mq_open(CAN_DRS_RX_MQUEUE_NAME, O_RDWR | O_CREAT, 0600, &canmq_attr);
  
  while(true)
    {
      clock_gettime(CLOCK_REALTIME, &mq_timeout);
      clock_timespec_add(&canmq_wait_time, &mq_timeout, &mq_timeout);
      ret = mq_timedreceive(rxmq, (char *)&rxmsg, sizeof(rxmsg), NULL, &mq_timeout);
      if (ret < 0 && errno == ETIMEDOUT)
      {
        // Control based on wheel speed and steering angle
        boardctl(BOARDIOC_DRS_STOP, 0);
        drs_powered = false;
        txmsg.cm_data[0] = 0;
        
      }
      else if (ret < 0 && errno == EINTR)
      {
        continue;
      }
      else if (ret < 0)
      {
        safing_store_internal_fault(FAULT_DRS_SOFTWARE);
        return -errno;
      }
      else
      {
        /* Parse the message and follow its instructions */
        if (rxmsg.cm_hdr.ch_dlc != 4)
          continue;
        
        if (rxmsg.cm_data[0] == 1)
        {
          txmsg.cm_data[0] = 0xff;
          boardctl(BOARDIOC_DRS_ANGLE, *(uint16_t *)(&(rxmsg.cm_data[1])));
          if (!drs_powered)
          {
            ret = boardctl(BOARDIOC_DRS_START, 0);
            if (ret < 0)
            {
              safing_store_dtc(DTC_DRSBCK_STG);
              return -1;
            }
            drs_powered = true;
          }
        }
      }
      
      mq_send(txmq, (const char *)&txmsg, CAN_MSGLEN(txmsg.cm_hdr.ch_dlc), 1);
    }
  
  return 0;
}
 
