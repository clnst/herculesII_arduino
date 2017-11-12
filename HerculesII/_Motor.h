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

#define _X_LENG                             200
#define _Y_LENG                             150
#define _BODY_PARAM                         (_X_LENG+_Y_LENG)

#if _FAULHABER2342L012
/*Motor Parameter*/
#define WHEEL_RADIUS                        50
#define DEC_RATIO                           64
#define RPM                                 120
#define CPR                                 12
/*
    ACP = CPR * DEC_RATIO = 768             -- a cycle pulse
    PERIMETER = WHEEL_RADIUS * 2 * pi = 314.15926 (mm)
    HTD = Perimeter/ACP = 0.409 (mm)        -- hall transformation distance
    STP =                                   -- speed transformation pwm
*/
#define HTD                                 0.409
#define STP                                 1
#endif // _FAULHABER2342L012

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

    void setPulseA(MotorDrv_t drv);
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
        eMotorLB_PwmPin = 9,
        eMotorRB_PwmPin = 6,
    }MotorIo_t;

    typedef struct{
        void (*handler)(void);
    }MotorIrq_t;

    MotorIrq_t              irq[eMotorMax];

    uint8_t                 pwm[eMotorMax];

    volatile uint16_t       pulseA[eMotorMax];
    volatile uint16_t       pulseB[eMotorMax];
    float                   pre_spd[eMotorMax];
    float                   act_spd[eMotorMax];
    float                   move_dis[eMotorMax];

    float                   linearX;
    float                   linearY;
    float                   angularZ;

    HerculesPID             pid;

    void setPwmFreq(uint32_t  clockA_freq);
    void setPwmPin(uint8_t  pin);
    void setPwmDuty(uint8_t  pin,  uint8_t  duty);
    void setDir(MotorDrv_t drv, MotorDir_t dir);
    void setSpd(float spd_x, float spd_y, float spd_z);

};

extern HerculesMotor motor;

/* Exported functions ------------------------------------------------------- */

#endif/*__MOTOR_H__*/
/************************ (C) COPYRIGHT clnst *****END OF FILE****/

