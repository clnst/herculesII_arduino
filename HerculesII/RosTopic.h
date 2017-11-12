/**
  ******************************************************************************
  * File Name          : RosTopic.h
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
#ifndef __ROS_TOPIC_H__
#define  __ROS_TOPIC_H__

/* Includes ------------------------------------------------------------------*/
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <hercules_msgs/BaseOdom.h>
#include <hercules_msgs/BaseTeleop.h>
#include <sensor_msgs/BatteryState.h>
#include "BoardConfig.h"
#include "_Battery.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BMS_FRAM        "/base_bms"
#define ODOM_FRAME      "/base_odom"

#define TWIST_TOPIC     "/cmd_vel"
#define TELEOP_TOPIC    "/teleop_vel"

#define BMS_TOPIC       BMS_FRAM
#define ODOM_TOPIC      ODOM_FRAME


/* Exported variable -------------------------------------------------------- */
class HerculesTopic{
public:
    HerculesTopic(void);
    ~HerculesTopic(void);

    hercules_msgs::BaseOdom odom;
    sensor_msgs::BatteryState bms;
    geometry_msgs::Twist cmd_vel;

    void init(void);
    void deinit(void);
    boolean update(void);

    void setLinearX(float val);
    void setLinearY(float val);
    void setAngularZ(float val);

    float getLinearX(void);
    float getLinearY(void);
    float getAngularZ(void);

    void setVoltage(float val);

private:

    ros::NodeHandle nh;
    ros::Time ct;

    geometry_msgs::Point site;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;

    boolean bms_update(void);
    boolean odom_update(void);

    float voltage;
};

extern HerculesTopic rostopic;

/* Exported functions ------------------------------------------------------- */

#endif/*__ROS_TOPIC_H__*/
/************************ (C) COPYRIGHT clnst *****END OF FILE****/
