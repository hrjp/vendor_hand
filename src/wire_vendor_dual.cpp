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
    
    ros::init(argc, argv, "wire_vendor_dual_node");
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
    dynamixel_wrapper::dynamixel_wrapper motor2(2,dxl_base,dynamixel_wrapper::XM430_W350_R,mode);
    dynamixel_wrapper::dynamixel_wrapper motor3(4,dxl_base,dynamixel_wrapper::XM430_W350_R,dynamixel_wrapper::Mode::Current);

    dynamixel_wrapper::dynamixel_wrapper rmotor0(5,dxl_base,dynamixel_wrapper::XM430_W350_R,mode);
    dynamixel_wrapper::dynamixel_wrapper rmotor2(1,dxl_base,dynamixel_wrapper::XM430_W350_R,mode);
    dynamixel_wrapper::dynamixel_wrapper rmotor3(3,dxl_base,dynamixel_wrapper::XM430_W350_R,dynamixel_wrapper::Mode::Current);

    ros::NodeHandle lSubscriber("");
    ros::Subscriber joy_sub = lSubscriber.subscribe("/joy", 50, &joy_callback);
    joy_msg.axes.resize(joy_size);
    joy_msg.buttons.resize(joy_size);
    pre_joy.axes.resize(joy_size);
    pre_joy.buttons.resize(joy_size);
    diff_joy.axes.resize(joy_size);
    diff_joy.buttons.resize(joy_size);

    motor0.setTorqueEnable(false);
    motor2.setTorqueEnable(false);
    motor3.setTorqueEnable(false);

    rmotor0.setTorqueEnable(false);
    rmotor2.setTorqueEnable(false);
    rmotor3.setTorqueEnable(false);
    
    motor0.setCurrentLimit(60.0);
    motor2.setCurrentLimit(10.0);
    motor3.setCurrentLimit(350.0);

    rmotor0.setCurrentLimit(60.0);
    rmotor2.setCurrentLimit(10.0);
    rmotor3.setCurrentLimit(350.0);
    
    const std::vector<double> init_angle={
        motor0.getPresentPosition(),
        motor2.getPresentPosition()};
    const std::vector<double> rinit_angle={
        rmotor0.getPresentPosition(),
        rmotor2.getPresentPosition()};

    // Publisher left
    ros::Publisher now_angle_pub = n.advertise<std_msgs::Float32>("left/now_angle", 1);
    ros::Publisher now_linear_pos_pub = n.advertise<std_msgs::Float32>("left/now_linear_pos", 1);
    ros::Publisher start_pub = n.advertise<std_msgs::Empty>("left/start", 1);
    ros::Publisher start_pub2 = n.advertise<std_msgs::Empty>("left/start2", 1);

    // Publisher right
    ros::Publisher rnow_angle_pub = n.advertise<std_msgs::Float32>("right/now_angle", 1);
    ros::Publisher rnow_linear_pos_pub = n.advertise<std_msgs::Float32>("right/now_linear_pos", 1);
    ros::Publisher rstart_pub = n.advertise<std_msgs::Empty>("right/start", 1);
    ros::Publisher rstart_pub2 = n.advertise<std_msgs::Empty>("right/start2", 1);

    bool is_manual=true;
    enum class ManualMode{
        None,
        Left,
        Right,
        All
    };
    ManualMode manual_state=ManualMode::All;
    // subscriber left
    ros::Subscriber angular_sub=n.subscribe<std_msgs::Float32>("left/angular_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        const auto target_angular_vel=msg->data*30.0/M_PI;
        motor0.setGoalVelocity(target_angular_vel);
    });
    ros::Subscriber linear_vel_sub=n.subscribe<std_msgs::Float32>("left/linear_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        const auto target_rpm=30.0/M_PI*msg->data*2.0/0.04;
        motor2.setGoalVelocity(-target_rpm);
    });
    ros::Subscriber angle_sub=n.subscribe<std_msgs::Float32>("left/angle",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        motor0.setGoalPosition(-msg->data*180.0/M_PI+init_angle[0]);
    });

    // subscriber right
    ros::Subscriber rangular_sub=n.subscribe<std_msgs::Float32>("right/angular_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        const auto target_angular_vel=msg->data*30.0/M_PI;
        rmotor0.setGoalVelocity(target_angular_vel);
    });
    ros::Subscriber rlinear_vel_sub=n.subscribe<std_msgs::Float32>("right/linear_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        const auto target_rpm=30.0/M_PI*msg->data*2.0/0.04;
        rmotor2.setGoalVelocity(target_rpm);
    });
    ros::Subscriber rangle_sub=n.subscribe<std_msgs::Float32>("right/angle",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(is_manual){return;}
        rmotor0.setGoalPosition(-msg->data*180.0/M_PI+rinit_angle[0]);
    });

    //manual control
    ros::Subscriber manual_angular_sub=n.subscribe<std_msgs::Float32>("manual/angular_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(!is_manual){return;}
        const auto target_angular_vel=msg->data*30.0/M_PI;
        if(manual_state==ManualMode::Left){
            motor0.setGoalVelocity(target_angular_vel);
        }
        else if(manual_state==ManualMode::Right){
            rmotor0.setGoalVelocity(-target_angular_vel);
        }
        else{
            motor0.setGoalVelocity(target_angular_vel);
            rmotor0.setGoalVelocity(target_angular_vel);
        }
    });
    ros::Subscriber manual_linear_vel_sub=n.subscribe<std_msgs::Float32>("manual/linear_vel",10,[&](const std_msgs::Float32::ConstPtr& msg){
        if(!is_manual){return;}
        const auto target_rpm=30.0/M_PI*msg->data*2.0/0.04;
        if(manual_state==ManualMode::Left){
            motor2.setGoalVelocity(-target_rpm);
        }
            
        else if(manual_state==ManualMode::Right){
            rmotor2.setGoalVelocity(target_rpm);
        }
        else{
            motor2.setGoalVelocity(-target_rpm);
            rmotor2.setGoalVelocity(target_rpm);
        }
    });
    
    while (n.ok())  {

        //circle
        if(joy_msg.buttons[1]){
            //set auto mode
            if(is_manual){
                is_manual=false;
                motor0.setTorqueEnable(false);
                motor2.setTorqueEnable(false);
                motor0.setOperatingMode(dynamixel_wrapper::Mode::CurrentBasePosition);
                motor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor0.setTorqueEnable(true);
                motor2.setTorqueEnable(true);
                
                rmotor0.setTorqueEnable(false);
                rmotor2.setTorqueEnable(false);
                rmotor0.setOperatingMode(dynamixel_wrapper::Mode::CurrentBasePosition);
                rmotor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                rmotor0.setTorqueEnable(true);
                rmotor2.setTorqueEnable(true);
            }
            else{
                motor0.setTorqueEnable(true);
                motor2.setTorqueEnable(true);
                rmotor0.setTorqueEnable(true);
                rmotor2.setTorqueEnable(true);
            }
        }
        //closs
        if(joy_msg.buttons[0]){
            // torque off
            motor0.setTorqueEnable(false);
            motor2.setTorqueEnable(false);
            rmotor0.setTorqueEnable(false);
            rmotor2.setTorqueEnable(false);
            
        }
        //triangle
        if(joy_msg.buttons[2]){
            // jamming off
            motor3.setTorqueEnable(false);
            rmotor3.setTorqueEnable(false);
        }
        //square
        if(joy_msg.buttons[3]){
            // set manual mode
            if(!is_manual){
                is_manual=true;
                motor0.setTorqueEnable(false);
                motor2.setTorqueEnable(false);
                motor0.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                motor0.setTorqueEnable(true);
                motor2.setTorqueEnable(true);

                rmotor0.setTorqueEnable(false);
                rmotor2.setTorqueEnable(false);
                rmotor0.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                rmotor2.setOperatingMode(dynamixel_wrapper::Mode::Velocity);
                rmotor0.setTorqueEnable(true);
                rmotor2.setTorqueEnable(true);
            }
            else{
                motor0.setTorqueEnable(true);
                motor2.setTorqueEnable(true);
                rmotor0.setTorqueEnable(true);
                rmotor2.setTorqueEnable(true);
            }
        }
       
        //right
        if(joy_msg.axes[6]<0){
          manual_state=ManualMode::Right;
        }

        //up
        if(joy_msg.axes[7]>0){
            manual_state=ManualMode::All;
        }

        //left
        if(joy_msg.axes[6]>0){
            manual_state=ManualMode::Left;
        }
        //down
        if(joy_msg.axes[7]<0){
           
        }

        //L1
        if(joy_msg.buttons[4]){
            motor3.setTorqueEnable(true);
            motor3.setGoalCurrent(-100.0);
        }
        //R1
        if(joy_msg.buttons[5]){
            rmotor3.setTorqueEnable(true);
            rmotor3.setGoalCurrent(100.0);
        }
        //L2
        if(joy_msg.buttons[6]){
            motor3.setTorqueEnable(true);
            motor3.setGoalCurrent(-340.0);
        }
        //R2
        if(joy_msg.buttons[7]){
            rmotor3.setTorqueEnable(true);
            rmotor3.setGoalCurrent(340.0);
        }
        //share
        if(joy_msg.buttons[8]){
            start_pub.publish(std_msgs::Empty());
        }
        //share
        if(joy_msg.buttons[9]){
            rstart_pub.publish(std_msgs::Empty());
        }

        // publish left status
        std_msgs::Float32 now_angle_msg;
        now_angle_msg.data = -(motor0.getPresentPosition()-init_angle[0])*M_PI/180.0;
        now_angle_pub.publish(now_angle_msg);

        std_msgs::Float32 now_linear_pos_msg;
        now_linear_pos_msg.data = -(motor2.getPresentPosition()-init_angle[1])*M_PI/180.0*0.04/2.0 +arm_unit_length * vendor_num;
        now_linear_pos_pub.publish(now_linear_pos_msg);

        // publish right status
        std_msgs::Float32 rnow_angle_msg;
        rnow_angle_msg.data = -(rmotor0.getPresentPosition()-rinit_angle[0])*M_PI/180.0;
        rnow_angle_pub.publish(rnow_angle_msg);

        std_msgs::Float32 rnow_linear_pos_msg;
        rnow_linear_pos_msg.data = (rmotor2.getPresentPosition()-rinit_angle[1])*M_PI/180.0*0.04/2.0 +arm_unit_length * vendor_num;
        rnow_linear_pos_pub.publish(rnow_linear_pos_msg);
        
        for(int i=0;i<joy_size;i++){
            diff_joy.axes[i]=0;
            diff_joy.buttons[i]=0;
        }
        ros::spinOnce();//subsucriberの割り込み関数はこの段階で実装される
        loop_rate.sleep();
        
    }

    return 0;
}