/**
  ******************************************************************************
  * File Name          : _Motor.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "_Motor.h"

/* Public variables ---------------------------------------------------------*/
HerculesMotor motor;

/* Private variables ---------------------------------------------------------*/


/**
  * @brief
  * @param
  * @retval
  */
void MotorLF_IRQHandler(void) {
    motor.setPulseA(eMotorLF);
}

/**
  * @brief
  * @param
  * @retval
  */
void MotorRF_IRQHandler(void) {
    motor.setPulseA(eMotorRF);
}

/**
  * @brief
  * @param
  * @retval
  */
void MotorLB_IRQHandler(void) {
    motor.setPulseA(eMotorLB);
}

/**
  * @brief
  * @param
  * @retval
  */
void MotorRB_IRQHandler(void) {
    motor.setPulseA(eMotorRB);
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::setPwmFreq(uint32_t  clockA_freq) {
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
    PWMC_ConfigureClocks( clockA_freq*TC_MAX_DUTY_CYCLE, 0, VARIANT_MCK );
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::setPwmPin(uint8_t  pin) {
    uint32_t  chan = g_APinDescription[pin].ulPWMChannel;
	if (pin < 6 || pin > 9)
		return;
	PIO_Configure(g_APinDescription[pin].pPort,
	              g_APinDescription[pin].ulPinType,
				  g_APinDescription[pin].ulPin,
				  g_APinDescription[pin].ulPinConfiguration);
	PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
	PWMC_SetPeriod(PWM_INTERFACE, chan, TC_MAX_DUTY_CYCLE);
	PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);
	PWMC_EnableChannel(PWM_INTERFACE, chan);
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::setPwmDuty(uint8_t  pin,  uint8_t  duty ) {
    if (pin < 6 || pin > 9)
		return;
	PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[pin].ulPWMChannel, duty);
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::setDir(MotorDrv_t drv, MotorDir_t dir) {
    switch(drv){
        case eMotorLF:
        case eMotorRF:
            if(dir==eMotorDir_Backward){
                digitalWrite(eMotorLF_DirPin+drv, HIGH);
            }else{
                digitalWrite(eMotorLF_DirPin+drv, LOW);
            }
            break;
        case eMotorLB:
        case eMotorRB:
            if(dir==eMotorDir_Backward){
                digitalWrite(eMotorLF_DirPin+drv, LOW);
            }else{
                digitalWrite(eMotorLF_DirPin+drv, HIGH);
            }
            break;
        default:
            return;
    }
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::setSpd(float spd_x, float spd_y, float spd_z) {

    pre_spd[eMotorLF] = spd_x + spd_y - spd_z*_BODY_PARAM;
    if(pre_spd[eMotorLF]>=0){
        setDir(eMotorLF, eMotorDir_Forward);
    }else{
        setDir(eMotorLF, eMotorDir_Backward);
    }

    pre_spd[eMotorRF] = spd_x - spd_y + spd_z*_BODY_PARAM;
    if(pre_spd[eMotorRF]>=0){
        setDir(eMotorRF, eMotorDir_Forward);
    }else{
        setDir(eMotorRF, eMotorDir_Backward);
    }

    pre_spd[eMotorLB] = spd_x - spd_y - spd_z*_BODY_PARAM;
    if(pre_spd[eMotorLB]>=0){
        setDir(eMotorLB, eMotorDir_Forward);
    }else{
        setDir(eMotorLB, eMotorDir_Backward);
    }

    pre_spd[eMotorRB] = spd_x + spd_y + spd_z*_BODY_PARAM;
    if(pre_spd[eMotorRB]>=0){
        setDir(eMotorRB, eMotorDir_Forward);
    }else{
        setDir(eMotorRB, eMotorDir_Backward);
    }
}

/**
  * @brief
  * @param
  * @retval
  */
HerculesMotor::HerculesMotor(void) {
    irq[eMotorLF].handler = MotorLF_IRQHandler;
    irq[eMotorRF].handler = MotorRF_IRQHandler;
    irq[eMotorLB].handler = MotorLB_IRQHandler;
    irq[eMotorRB].handler = MotorRB_IRQHandler;
}

/**
  * @brief
  * @param
  * @retval
  */
HerculesMotor::~HerculesMotor(void) {

}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::init(void) {
    setPwmFreq(20000);
    for(uint8_t i=0; i<eMotorMax; i++) {
        pinMode((eMotorLF_DirPin + i), OUTPUT);
        setDir((MotorDrv_t)i,eMotorDir_Forward);
        delay(10);
        setPwmPin(eMotorRB_PwmPin + i);
        pinMode((eMotorLF_EpaPin + i), INPUT_PULLUP);
        attachInterrupt(eMotorLF_EpaPin + i, irq[i].handler, HALL_TRIGGER_TYPE);
    }
    delay(10);
    pinMode(eMotorDrv_PwrPin, OUTPUT);
    digitalWrite(eMotorDrv_PwrPin, HIGH);

    pid.init();
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::deinit(void) {
    digitalWrite(eMotorDrv_PwrPin, LOW);
    for(uint8_t i=0; i<eMotorMax; i++) {
        detachInterrupt(eMotorLF_EpaPin + i);
        setPwmDuty((eMotorRB_PwmPin + i),0);
        delay(10);
        setDir((MotorDrv_t)i,eMotorDir_Forward);
    }
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesMotor::setPulseA(MotorDrv_t drv) {
    motor.pulseA[drv] += HALL_TRIGGER_INC;
}

/**
  * @brief
  * @param
  * @retval
  */
boolean HerculesMotor::update(void) {
    setSpd(rostopic.getLinearX(), rostopic.getLinearY(), rostopic.getAngularZ());

    return true;
}

/************************ (C) COPYRIGHT clnst *****END OF FILE****/




