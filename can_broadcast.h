/****************************************************************************
 * apps/industry/ETCetera/can_broadcast.h
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

#ifndef APPS_INDUSTRY_ETCETERA_CAN_BROADCAST_H
#define APPS_INDUSTRY_ETCETERA_CAN_BROADCAST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <nuttx/can/can.h>
#include <sys/boardctl.h>

#include "nshlib/nshlib.h"
#include "safing.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAN_ID_DRS_STATUS_TX    0xAAAA1
#define CAN_ID_DRS_CONTROL_RX   0xAAAA2
#define CAN_ID_DTC_TX           0xBBBB0
#define CAN_ID_FAULT_TX         0xBBBB1

#define CAN_NUM_RX_MQUEUES          1
#define CAN_DRS_RX_MQUEUE_NAME      "/can.drs.rx"
#define CAN_DRS_RX_MQUEUE_IDX       0

#define CAN_NUM_TX_MQUEUES          2
#define CAN_DRS_TX_MQUEUE_NAME      "/can.drs.tx"
#define CAN_DRS_TX_MQUEUE_IDX       0
#define CAN_SAFING_TX_MQUEUE_NAME   "/can.safing.tx"
#define CAN_SAFING_TX_MQUEUE_IDX    1

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

#endif /* APPS_INDUSTRY_ETCETERA_CAN_BROADCAST_H */
