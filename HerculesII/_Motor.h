/**
  ******************************************************************************
  * File Name          : _Motor.h
  * Description        :
  ******************************************************************************
  *
  * Copyright (c) 2017 CLNST.COM
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of CLNST nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for CLNST.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY CLNST AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL CLNST OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H__
#define  __MOTOR_H__

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include "BoardConfig.h"
#include "IncPid.h"
#include "RosTopic.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define _FAULHABER2342L012                  true
#define _GM37_520                           false

#define HALL_TRIGGER_TYPE                   FALLING
#if (HALL_TRIGGER_TYPE==FALLING)||(HALL_TRIGGER_TYPE==RISING)
#define HALL_TRIGGER_INC                    1
#elif (HALL_TRIGGER_TYPE==CHANGE)
#define HALL_TRIGGER_INC                    2
#else
#define HALL_TRIGGER_INC                    0
#endif

/* Exported variable -------------------------------------------------------- */
typedef enum{
    eMotorLF = 0,       // M3 7
    eMotorRF = 1,       // M2 8
    eMotorLB = 2,       // M4 6
    eMotorRB = 3,       // M1 9
    eMotorMax = 4,
}MotorDrv_t;

typedef enum{
    eMotorDir_Forward = 0,
    eMotorDir_Backward = 1,
}MotorDir_t;

class HerculesMotor{
public:
    HerculesMotor(void);
    ~HerculesMotor(void);

    void init(void);
    void deinit(void);

    boolean update(void);

private:
    typedef enum{
        eMotorDrv_PwrPin = 64,

        eMotorLF_DirPin = 54,
        eMotorRF_DirPin = 55,
        eMotorLB_DirPin = 56,
        eMotorRB_DirPin = 57,

        eMotorLF_EpaPin = 58,
        eMotorRF_EpaPin = 59,
        eMotorLB_EpaPin = 60,
        eMotorRB_EpaPin = 61,

        eMotorLF_PwmPin = 7,
        eMotorRF_PwmPin = 8,
        eMotorLB_PwmPin = 6,
        eMotorRB_PwmPin = 9,
    }MotorIo_t;

    typedef struct{
        void (*handler)(void);
    }MotorIrq_t;

    MotorIrq_t  irq[eMotorMax];

    HerculesPID     pid;

    int linear;
    int angular;

    void setPwmFreq(uint32_t  clockA_freq);
    void setPwmPin( uint32_t  pin );
    void setPwmDuty( uint32_t  pin,  uint32_t  duty );
    void setDir(MotorDrv_t drv, MotorDir_t dir);
    void setSpd(MotorDrv_t drv, uint16_t spd);

};

/* Exported functions ------------------------------------------------------- */

#endif/*__MOTOR_H__*/
/************************ (C) COPYRIGHT clnst *****END OF FILE****/

