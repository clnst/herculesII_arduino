/**
  ******************************************************************************
  * File Name          : RosTopic.cpp
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
#include "RosTopic.h"

/* Public variables ----------------------------------------------------------*/
HerculesTopic rostopic;

/* Public functions ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void twist_cb(const geometry_msgs::Twist& msg);
static void teleop_cb(const hercules_msgs::BaseTeleop& msg);

ros::Subscriber<geometry_msgs::Twist> twist_sub(TWIST_TOPIC, twist_cb);
ros::Subscriber<hercules_msgs::BaseTeleop> teleop_sub(TELEOP_TOPIC, teleop_cb);

ros::Publisher bms_pub(BMS_TOPIC, &rostopic.bms);
ros::Publisher odom_pub(ODOM_TOPIC, &rostopic.odom);


/**
  * @brief
  * @param
  * @retval
  */
static void twist_cb(const geometry_msgs::Twist& msg) {
    rostopic.cmd_vel.linear.x = msg.linear.x;
    rostopic.cmd_vel.linear.y = msg.linear.y;
    rostopic.cmd_vel.linear.z = msg.linear.z;

    rostopic.cmd_vel.angular.x = msg.angular.x;
    rostopic.cmd_vel.angular.y = msg.angular.y;
    rostopic.cmd_vel.angular.z = msg.angular.z;
}

/**
  * @brief
  * @param
  * @retval
  */
static void teleop_cb(const hercules_msgs::BaseTeleop& msg) {

}

/**
  * @brief
  * @param
  * @retval
  */
HerculesTopic::HerculesTopic(void) {
    // odom parameter
    site.x = 0.0;
    site.y = 0.0;
    site.z = 0.0;

    linear.x = 0.0;
    linear.y = 0.0;
    linear.z = 0.0;

    angular.x = 0.0;
    angular.y = 0.0;
    angular.z = 0.0;

    // bms parameter
    bms.current = 25;
    bms.charge = 0;
    bms.capacity = 2.6;
    bms.design_capacity = 2.0;
    bms.power_supply_status = bms.POWER_SUPPLY_STATUS_FULL;
    bms.power_supply_health = bms.POWER_SUPPLY_HEALTH_GOOD;
    bms.power_supply_technology = bms.POWER_SUPPLY_TECHNOLOGY_LIPO;
    bms.present = true;
    bms.location = "plug";
    bms.serial_number = "123456";
}

/**
  * @brief
  * @param
  * @retval
  */
HerculesTopic::~HerculesTopic(void) {

}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesTopic::init(void) {
    nh.initNode();
    nh.advertise(bms_pub);
    nh.advertise(odom_pub);

    nh.subscribe(twist_sub);
    nh.subscribe(teleop_sub);
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesTopic::deinit(void) {

}

/**
  * @brief
  * @param
  * @retval
  */
boolean HerculesTopic::bms_update(void) {
    bms.header.stamp = ct;
    bms.header.frame_id = BMS_FRAM;

    bms_pub.publish(&bms);
}

/**
  * @brief
  * @param
  * @retval
  */
boolean HerculesTopic::odom_update(void) {
    odom.header.stamp = ct;
    odom.header.frame_id = ODOM_FRAME;
    //data conversion

    //set the position
    odom.pose.position.x = site.x;
    odom.pose.position.y = site.y;
    odom.pose.position.z = site.z;
    odom.pose.orientation = tf::createQuaternionFromYaw(site.z);

    //set the velocity
    odom.twist.linear.x = linear.x;
    odom.twist.linear.y = linear.y;
    odom.twist.linear.z = linear.z;
    odom.twist.angular.x = angular.x;
    odom.twist.angular.y = angular.y;
    odom.twist.angular.z = angular.z;

    //publish the message
    odom_pub.publish(&odom);
}

/**
  * @brief
  * @param
  * @retval
  */
boolean HerculesTopic::update(void) {
    ct = nh.now();
    odom_update();
    bms_update();
    nh.spinOnce();
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesTopic::setLinearX(float val) {
    linear.x = val;
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesTopic::setLinearY(float val) {
    linear.y = val;
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesTopic::setAngularZ(float val) {
    angular.z = val;
}

/**
  * @brief
  * @param
  * @retval
  */
float HerculesTopic::getLinearX(void) {
    return rostopic.cmd_vel.linear.x;
}

/**
  * @brief
  * @param
  * @retval
  */
float HerculesTopic::getLinearY(void) {
    return rostopic.cmd_vel.linear.y;
}

/**
  * @brief
  * @param
  * @retval
  */
float HerculesTopic::getAngularZ(void) {
    return rostopic.cmd_vel.angular.z;
}

/**
  * @brief
  * @param
  * @retval
  */
void HerculesTopic::setVoltage(float val) {
    bms.voltage = val;
}


/************************ (C) COPYRIGHT clnst *****END OF FILE****/







