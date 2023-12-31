#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include "logic/Automation.hpp"
#include "logic/AutomationTypes.hpp"

/** @file
 * 
 * */

void Automation::setNode(rclcpp::Node::SharedPtr node){
    this->node=node;
    driveLeftSpeedPublisher= this->node->create_publisher<std_msgs::msg::Float32>("drive_left_speed",1);
    driveRightSpeedPublisher= this->node->create_publisher<std_msgs::msg::Float32>("drive_right_speed",1);
    dumpBinSpeedPublisher = this->node->create_publisher<std_msgs::msg::Float32>("dump_bin_speed",1);
    shoulderSpeedPublisher = this->node->create_publisher<std_msgs::msg::Float32>("shoulder_speed",1);
    excavationArmPublisher = this->node->create_publisher<std_msgs::msg::Float32>("excavationArm",1);
    excavationDrumPublisher = this->node->create_publisher<std_msgs::msg::Float32>("excavationDrum",1);
    servoStatePublisher = this->node->create_publisher<std_msgs::msg::Bool>("servo_state", 1);
    goPublisher = this->node->create_publisher<std_msgs::msg::Empty>("GO", 1);
}


void Automation::setPosition(Position position){
    this->position = position;
    this->orientationQuaternion.x=position.ox;
    this->orientationQuaternion.y=position.oy;
    this->orientationQuaternion.z=position.oz;
    this->orientationQuaternion.w=position.ow;
    this->orientation=toEulerAngles(this->orientationQuaternion);
}


void Automation::changeSpeed(float left, float right){
    if(currentLeftSpeed==left && currentRightSpeed==right) return;
    currentLeftSpeed=left;
    currentRightSpeed=right;
    std_msgs::msg::Float32 speedLeft;
    std_msgs::msg::Float32 speedRight;
    speedLeft.data=left;
    speedRight.data=right;
    driveLeftSpeedPublisher->publish(speedLeft);
    driveRightSpeedPublisher->publish(speedRight);
}

EulerAngles Automation::toEulerAngles(Quaternion q) {
    EulerAngles angles;

     // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void Automation::changeDumpBinSpeed(float speed){
    std_msgs::msg::Float32 dumpBinSpeed;
    dumpBinSpeed.data = speed;
    dumpBinSpeedPublisher->publish(dumpBinSpeed);
}

void Automation::changeShoulderSpeed(float speed){
    std_msgs::msg::Float32 shoulderSpeed;
    shoulderSpeed.data = speed;
    shoulderSpeedPublisher->publish(shoulderSpeed);
}

void Automation::changeArmSpeed(float speed){
    std_msgs::msg::Float32 armSpeed;
    armSpeed.data = speed;
    excavationArmPublisher->publish(armSpeed);
}

void Automation::changeDrumSpeed(float speed){
    std_msgs::msg::Float32 drumSpeed;
    drumSpeed.data = speed;
    excavationDrumPublisher->publish(drumSpeed);
}

void Automation::setDumpState(bool state){
    std_msgs::msg::Bool dumpState;
    dumpState.data = state;
    servoStatePublisher->publish(dumpState);
}

void Automation::setGo(){
   std_msgs::msg::Empty empty;
   goPublisher->publish(empty);
}
