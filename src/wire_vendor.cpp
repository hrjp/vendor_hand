/**
* @file manual_control.cpp
* @brief manual control node
* @author Shunya Hara
* @date 2023.5.15
* @details 
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <dynamixel_wrapper/dynamixel_wrapper.h>
size_t joy_size=20;
sensor_msgs::Joy joy_msg;
sensor_msgs::Joy pre_joy,diff_joy;
void joy_callback(const sensor_msgs::Joy& msg){
    joy_msg=msg;
    for(int i=0;i<joy_size;i++){
        diff_joy.axes[i]=joy_msg.axes[i]-pre_joy.axes[i];
        diff_joy.buttons[i]=joy_msg.buttons[i]-pre_joy.buttons[i];
    }
    pre_joy=joy_msg;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "wire_vendor_node");
    ros::NodeHandle n;
    double rate=20.0;
    //制御周期10Hz
    ros::Rate loop_rate(rate);

    //param setting
    ros::NodeHandle pn("~");
    double arm_unit_length = pn.param<double>("arm_unit_length", 0.013);
    int vendor_num = pn.param<int>("vendor_num", 1);
    
    std::string port_name("/dev/ttyUSB0");
    int baudrate=1000000;
    dynamixel_wrapper::Mode mode=dynamixel_wrapper::Mode::Velocity;
    dynamixel_wrapper::dynamixel_wrapper_base dxl_base(port_name,baudrate);
    dynamixel_wrapper::dynamixel_wrapper motor0(0,dxl_base,dynamixel_wrapper::XM430_W350_R,mode);
    dynamixel_wrapper::dynamixel_wrapper motor1(1,dxl_base,dynamixel_wrapper::XM430_W350_R,mode);
    dynamixel_wrapper::dynamixel_wrapper motor2(2,dxl_base,dynamixel_wrapper::XM430_W350_R,mode);
    dynamixel_wrapper::dynamixel_wrapper motor3(3,dxl_base,dynamixel_wrapper::XM430_W350_R,dynamixel_wrapper::Mode::Current);

    ros::NodeHandle lSubscriber("");
    ros::Subscriber joy_sub = lSubscriber.subscribe("/joy", 50, &joy_callback);
    joy_msg.axes.resize(joy_size);
    joy_msg.buttons.resize(joy_size);
    pre_joy.axes.resize(joy_size);
    pre_joy.buttons.resize(joy_size);
    diff_joy.axes.resize(joy_size);
    diff_joy.buttons.resize(joy_size);

    motor0.setTorqueEnable(false);
    motor1.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);

    motor0.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
    motor1.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
    motor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
    
    motor0.setCurrentLimit(60.0);
    motor1.setCurrentLimit(10.0);
    motor2.setCurrentLimit(10.0);
    motor3.setCurrentLimit(350.0);
    
    const std::vector<double> init_angle={
        motor0.getPresentPosition(),
        motor1.getPresentPosition(),
        motor2.getPresentPosition()};

    ros::Publisher now_angle_pub = n.advertise<std_msgs::Float32>("now_angle", 1);
    ros::Publisher now_linear_pos_pub = n.advertise<std_msgs::Float32>("now_linear_pos", 1);
    ros::Publisher start_pub = n.advertise<std_msgs::Empty>("start", 1);
    ros::Publisher start_pub2 = n.advertise<std_msgs::Empty>("start2", 1);

    bool is_manual=true;
    // subscriber
    ros::Subscriber angular_sub=n.subscribe<std_msgs::Float32>("angular_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        const auto target_angular_vel=msg->data*30.0/M_PI;
        motor0.setGoalVelocity(target_angular_vel);
    });
    ros::Subscriber linear_vel_sub=n.subscribe<std_msgs::Float32>("linear_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        const auto target_rpm=30.0/M_PI*msg->data*2.0/0.04;
        motor1.setGoalVelocity(target_rpm);
        motor2.setGoalVelocity(-target_rpm);
    });
    ros::Subscriber angle_sub=n.subscribe<std_msgs::Float32>("angle",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        motor0.setGoalPosition(-msg->data*180.0/M_PI+init_angle[0]);
    });

    //manual control
    ros::Subscriber manual_angular_sub=n.subscribe<std_msgs::Float32>("manual/angular_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(!is_manual){return;}
        const auto target_angular_vel=msg->data*30.0/M_PI;
        motor0.setGoalVelocity(target_angular_vel);
    });
    ros::Subscriber manual_linear_vel_sub=n.subscribe<std_msgs::Float32>("manual/linear_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(!is_manual){return;}
        const auto target_rpm=30.0/M_PI*msg->data*2.0/0.04;
        motor1.setGoalVelocity(target_rpm);
        motor2.setGoalVelocity(-target_rpm);
    });
    
    while (n.ok())  {
        //motor3.setGoalPosition(200.0*(joy_msg.axes[0])+offset_angle[3]);
        //ROS_INFO("Positions:{%lf, %lf, %lf}",motor0.getPresentPosition(),motor1.getPresentPosition(),motor2.getPresentPosition());
        //circle
        if(joy_msg.buttons[1]){
            //set auto mode
            if(is_manual){
                is_manual=false;
                motor0.setTorqueEnable(false);
                motor1.setTorqueEnable(false);
                motor2.setTorqueEnable(false);
                motor0.setOperatingMode(dynamixel_wrapper::Mode::CurrentBasePosition);
                motor1.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor0.setTorqueEnable(true);
                motor1.setTorqueEnable(true);
                motor2.setTorqueEnable(true);
                motor3.setTorqueEnable(true);         
            }
        }
        //closs
        if(joy_msg.buttons[0]){
            motor0.setTorqueEnable(false);
            motor1.setTorqueEnable(false);
            motor2.setTorqueEnable(false);
            
        }
        //triangle
        if(joy_msg.buttons[2]){
            motor3.setTorqueEnable(false);
        }
        //square
        if(joy_msg.buttons[3]){
            // set manual mode
            if(!is_manual){
                is_manual=true;
                motor0.setTorqueEnable(false);
                motor1.setTorqueEnable(false);
                motor2.setTorqueEnable(false);
                motor0.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor1.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor0.setTorqueEnable(true);
                motor1.setTorqueEnable(true);
                motor2.setTorqueEnable(true);
                motor3.setTorqueEnable(true);
            }
        }
       
        //right
        if(joy_msg.axes[6]<0){
          
        }

        //up
        if(joy_msg.axes[7]>0){
            
        }

        //left
        if(joy_msg.axes[6]>0){
            
        }
        //down
        if(joy_msg.axes[7]<0){
           
        }

        //L1
        if(joy_msg.buttons[4]){
            motor3.setTorqueEnable(true);
            motor3.setGoalCurrent(340.0);
        }
        //R1
        if(joy_msg.buttons[5]){
            motor3.setTorqueEnable(true);
            motor3.setGoalCurrent(50.0);
        }
        //share
        if(joy_msg.buttons[8]){
            start_pub.publish(std_msgs::Empty());
        }
        //share
        if(joy_msg.buttons[9]){
            start_pub2.publish(std_msgs::Empty());
        }


        std_msgs::Float32 now_angle_msg;
        now_angle_msg.data = -(motor0.getPresentPosition()-init_angle[0])*M_PI/180.0;
        now_angle_pub.publish(now_angle_msg);

        std_msgs::Float32 now_linear_pos_msg;
        now_linear_pos_msg.data = (motor1.getPresentPosition()-init_angle[1])*M_PI/180.0*0.04/2.0 +arm_unit_length * vendor_num;
        now_linear_pos_pub.publish(now_linear_pos_msg);
        
        for(int i=0;i<joy_size;i++){
            diff_joy.axes[i]=0;
            diff_joy.buttons[i]=0;
        }
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }

    return 0;
}