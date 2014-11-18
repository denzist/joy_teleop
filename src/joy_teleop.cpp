#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>

class JoyTeleop
{
public:
  JoyTeleop();

private:
  void callback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle* nh_;
  //absolute values > 0.0
  double max_lin_vel_;
  double max_ang_vel_;
  double lin_step_;
  double ang_step_;
  double cur_lin_vel_; 
  double cur_ang_vel_;
  bool cruise_control_;
  ros::Time last_update_;
  std::string cmd_topic_;
  geometry_msgs::Twist vel_;
  geometry_msgs::TwistStamped vel_stamped_;

  ros::Publisher vel_pub_;
  ros::Publisher vel_stamped_pub_;
  ros::Subscriber joy_sub_;
};


JoyTeleop::JoyTeleop():
cur_lin_vel_(0.3),
cur_ang_vel_(0.5),
lin_step_(0.05),
ang_step_(0.05),
max_lin_vel_(1.3),
max_ang_vel_(1.0),
cmd_topic_("/cmd_vel"),
cruise_control_(false)
{
  nh_ = new ros::NodeHandle("~");
  last_update_ = ros::Time::now();
  nh_->param("vel_step", lin_step_, lin_step_);
  nh_->param("ang_step", ang_step_, ang_step_);
  nh_->param("max_lin_vel", max_lin_vel_, max_lin_vel_);
  nh_->param("max_ang_vel", max_ang_vel_, max_ang_vel_);
  nh_->param<std::string>("cmd_topic", cmd_topic_, cmd_topic_);
  vel_pub_ = nh_->advertise<geometry_msgs::Twist>(cmd_topic_, 1);
  vel_stamped_pub_ = nh_->advertise<geometry_msgs::TwistStamped>(cmd_topic_ + "_stamped", 1);
  joy_sub_ = nh_->subscribe<sensor_msgs::Joy>("/joy", 100, &JoyTeleop::callback, this);
}

void JoyTeleop::callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  ros::Duration delta = ros::Time::now() - last_update_;
  if(delta.toSec() > 0.1)
  {
    last_update_ = ros::Time::now();
    if(joy->buttons[5])
    {
      cruise_control_ = !cruise_control_;
      ROS_INFO_STREAM("cruise_control_ = " << cruise_control_);
    }
    if(joy->buttons[1] && cur_lin_vel_ - lin_step_ >= 0.0 ) //A
    {
      cur_lin_vel_ -= lin_step_;
      ROS_INFO_STREAM("cur_lin_vel_ = "<<cur_lin_vel_);
    }
    if(joy->buttons[3] && cur_lin_vel_ + lin_step_ < max_lin_vel_ + lin_step_) //Y
    {
      cur_lin_vel_ += lin_step_;
      ROS_INFO_STREAM("cur_lin_vel_ = "<<cur_lin_vel_);
    }
    if(joy->buttons[0] && cur_ang_vel_ - ang_step_ >= 0.0 ) //X
    {
      cur_ang_vel_ -= ang_step_;
      ROS_INFO_STREAM("cur_ang_vel_ = "<<cur_ang_vel_);
    }
    if(joy->buttons[2] && cur_ang_vel_ + ang_step_ < max_ang_vel_ + lin_step_) //B
    {
      cur_ang_vel_ +=ang_step_;
      ROS_INFO_STREAM("cur_ang_vel_ = "<<cur_ang_vel_);
    }
  }
  if(cruise_control_)
  {
    vel_pub_.publish(vel_);
    vel_stamped_.header.stamp = ros::Time::now();
    vel_stamped_pub_.publish(vel_stamped_);
    return;
  }
  vel_.linear.x = cur_lin_vel_ * joy->axes[1];
  vel_.angular.z = cur_ang_vel_ * joy->axes[0];
  vel_stamped_.twist = vel_;
  vel_stamped_.header.stamp = ros::Time::now();
  vel_pub_.publish(vel_);
  vel_stamped_pub_.publish(vel_stamped_);
  return;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  JoyTeleop j_teleop;
  ros::spin();
  return 0;
}

