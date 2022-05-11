/****************************************************************************
 * apps/industry/ETCetera/safing.c
 * Electronic Throttle Controller program - safing thread
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

#ifndef APPS_INDUSTRY_ETCETERA_SAFING_H
#define APPS_INDUSTRY_ETCETERA_SAFING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DTC_LETTER_MASK 0xc000
#define DTC_NUMBER_MASK 0x3fff
#define DTC_P(n) (0x0000 | ((n) & DTC_NUMBER_MASK))
#define DTC_C(n) (0x4000 | ((n) & DTC_NUMBER_MASK))
#define DTC_B(n) (0x8000 | ((n) & DTC_NUMBER_MASK))
#define DTC_U(n) (0xc000 | ((n) & DTC_NUMBER_MASK))

#define DTC_INTERNAL_FAULT DTC_U(3000)

#define SAFING_STATE_PRE_PROVEOUT     0
#define SAFING_STATE_ONBOARD_PROVEOUT 1
#define SAFING_STATE_BSPD_PROVEOUT    2
#define SAFING_STATE_ETB_RELEARN      3
#define SAFING_STATE_ACTIVE           4
#define SAFING_STATE_PAUSED           5
#define SAFING_STATE_SOFT_FAULT       6
#define SAFING_STATE_HARD_FAULT       7

#define SAFING_ONBOARD_PROVEOUT_INPROGRESS      0
#define SAFING_ONBOARD_PROVEOUT_AWAIT_ETB_CLOSE 1
#define SAFING_ONBOARD_PROVEOUT_SUCCESSFUL      2

#define SAFING_BSPD_PROVEOUT_INPROGRESS         0
#define SAFING_BSPD_PROVEOUT_AWAIT_BSPD_CLOSE   1
#define SAFING_BSPD_PROVEOUT_SUCCESSFUL         2

#define SAFING_ETB_RELEARN_INPROGRESS           0
#define SAFING_ETB_RELEARN_AWAIT_CMS_CLOSE      1
#define SAFING_ETB_RELEARN_AWAIT_BOTS_CLOSE     2
#define SAFING_ETB_RELEARN_AWAIT_BRAKE_RELEASE  3

#define SAFING_ACTIVE_OK                        0

#define SAFING_PAUSED_SOFTWARE_REQUEST          0
#define SAFING_PAUSED_CMS_OPEN                  1
#define SAFING_PAUSED_BOTS_OPEN                 2
#define SAFING_PAUSED_THROTTLE_STUCK            3
#define SAFING_PAUSED_ARM_FAILED                4

#define DTC_INVALID                 DTC_P(0)
#define DTC_5V0LIN_SENSE_STG        DTC_P(1)
#define DTC_APPS1_STP               DTC_P(2)
#define DTC_APPS2_STP               DTC_P(3)
#define DTC_BRKF_STP                DTC_P(4)
#define DTC_BRKR_STP                DTC_P(5)
#define DTC_TPS_STP                 DTC_P(6)
#define DTC_APPS1_LOW               DTC_P(7)
#define DTC_APPS1_HIGH              DTC_P(8)
#define DTC_APPS2_LOW               DTC_P(9)
#define DTC_APPS2_HIGH              DTC_P(10)
#define DTC_BRKF_LOW                DTC_P(11)
#define DTC_BRKF_HIGH               DTC_P(12)
#define DTC_BRKR_LOW                DTC_P(13)
#define DTC_BRKR_HIGH               DTC_P(14)
#define DTC_TPS1_LOW                DTC_P(15)
#define DTC_TPS1_HIGH               DTC_P(16)
#define DTC_TPS2_LOW                DTC_P(17)
#define DTC_TPS2_HIGH               DTC_P(18)
#define DTC_APPS_OOC                DTC_P(19)
#define DTC_TPS_OOC                 DTC_P(20)

#define DTC_APPS_HARDOOC            DTC_P(21)
#define DTC_TPS_HARDOOC             DTC_P(22)
#define DTC_APPS1_OPEN              DTC_P(23)
#define DTC_APPS2_OPEN              DTC_P(24)
#define DTC_BRKF_OPEN               DTC_P(25)
#define DTC_BRKR_OPEN               DTC_P(26)
#define DTC_TPS1_OPEN               DTC_P(27)
#define DTC_TPS2_OPEN               DTC_P(28)
#define DTC_LOCALBSPD               DTC_P(29)
#define DTC_BSPD_OPEN               DTC_P(30)
#define DTC_BSPD_REARMED            DTC_P(31)
#define DTC_INITIAL_ARM_FAILED      DTC_P(32)
#define DTC_5VAUX_STP               DTC_P(33)

#define FAULT_INVALID               0
#define FAULT_CAN_OPEN_FAILED       1
#define FAULT_DRS_SOFTWARE          2
#define FAULT_UNEXPECTED_DISARM_1   3
#define FAULT_UNEXPECTED_SAFING_1   4
#define FAULT_UNEXPECTED_DISARM_2   5
#define FAULT_UNEXPECTED_SAFING_2   6
#define FAULT_OL_INTERNAL_FAULT     7
#define FAULT_ALREADY_ARMED_1       8
#define FAULT_ALREADY_ARMED_2       9
#define FAULT_NOT_SAFING_1          10
#define FAULT_NOT_SAFING_2          11
#define FAULT_ARM_FAILED_1          12
#define FAULT_ARM_FAILED_2          13

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct safing_status_s
{
  uint32_t fault_flags;
  uint8_t state;
  uint8_t reason;
};

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

void safing_store_dtc(uint16_t dtc);
void safing_store_internal_fault(uint16_t fault_code);

#endif /* APPS_INDUSTRY_ETCETERA_SAFING_H */
