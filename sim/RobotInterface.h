#ifndef ROBOTINTERFACE_H__
#define ROBOTINTERFACE_H__

#include <iostream>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Display.hpp>
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <webots/ImageRef.hpp>
#include <webots/Keyboard.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <memory>

using namespace webots;
using namespace std;

class RobotInterface : public Supervisor {
public:
  RobotInterface() {
    //create supervisor and get the robot node
    robot_node = getFromDef("MY_ROBOT");

    timeStep = (int)getBasicTimeStep(); // set the control time step
    // get device tags from webots
    left_wheel = getMotor("wheel_left_joint"));
    right_wheel = getMotor("wheel_right_joint"));
    head_joints.push_back(getMotor("head_1_joint"));
    head_joints.push_back(getMotor("head_2_joint"));
    arm_joints.push_back(getMotor("arm_1_joint"));
    arm_joints.push_back(getMotor("arm_2_joint"));
    arm_joints.push_back(getMotor("arm_3_joint"));
    arm_joints.push_back(getMotor("arm_4_joint"));
    arm_joints.push_back(getMotor("arm_5_joint"));
    arm_joints.push_back(getMotor("arm_6_joint"));
    arm_joints.push_back(getMotor("arm_7_joint"));

    // initialize joins
    SetLeftMotorSpeed(left_wheel->getMaxVelocity()/2.0);
    SetRightMotorSpeed(right_wheel->getMaxVelocity()/2.0);
    SetHeadPosition(0,0.0);
    SetHeadPosition(1,0.0);
    SetTorsoJointAngle(0.09);
    SetArmJointPosition(0,0.07);
    SetArmJointPosition(1,1.02);
    SetArmJointPosition(2,-3.16);
    SetArmJointPosition(3,1.27);
    SetArmJointPosition(4,1.32);
    SetArmJointPosition(5,0.0);
    SetArmJointPosition(6,1.41);

    display = getDisplay("display");

    lidar = getLidar("Hokuyo URG-04LX-UG01");
    lidar->enable(timeStep);
    lidar->enablePointCloud();
    lidar_offsets.resize(LIDAR_ANGLE_BINS-82*2);
    lidar_offsets.at(0) = -(LIDAR_ANGLE_RANGE/2.0)+((LIDAR_ANGLE_RANGE/LIDAR_ANGLE_BINS)*82);
    for(uint ii=1; ii<lidar_offset.size(); ii++){
      lidar_offsets.at(ii) += lidar_offsets.at(ii-1)+(LIDAR_ANGLE_RANGE/LIDAR_ANGLE_BINS);
    }

    gps = getGPS("gps");
    gps->enable(timeStep);

    compass = getCompass("compass");
    compass->enable(timeStep);

    keyboard = robot_node->getKeyboard();
    keyboard->enable(timeStep);
  }

  void SetLeftMotorSpeed(double speed){
    left_wheel->setPosition(INFINITY);
    left_wheel->setVelocity(speed);
  }

  void SetRightMotorSpeed(double speed){
    right_wheel->setPosition(INFINITY);
    right_wheel->setVelocity(speed);
  }

  void SetTorsoJointAngle(double angle){
    torso_lift_joint->setPosition(angle);
    torso_lift_joint->setVelocity(torso_lift_joint->getMaxVelocity()/2.0);
  }
  void SetHeadPosition(uint joint_index, double angle){
    head_joints.at(joint_index)->setPosition(angle);
    head_joints.at(joint_index)->setVelocity(head_joints.at(joint_index)->getMaxVelocity()/2.0);
  }

  void SetArmJointPosition(uint joint_index, double angle){
    arm_joints.at(joint_index)->setPosition(angle);
    arm_joints.at(joint_index)->setVelocity(arm_joints.at(joint_index)->getMaxVelocity()/2.0);
  }

  void GetLidarRangeImage(cv::Mat& image){
    image.release();
    image = cv::Mat(lidar->getNumberOfPoints(),lidar->getHorizontalResolution(),CV_32F, const_cast<float*>(lidar->getRangeImage())).clone();
  }

  void GetLidarData(vector<double> data){
    //TODO
    //    lidar_sensor_readings = lidar.getRangeImage()
    // lidar_sensor_readings = lidar_sensor_readings[83:len(lidar_sensor_readings)-83]
  }

  void SaveDisplayImageToFile(const string& filename){
    display->imageSave(NULL,filename);
  }

  void GetGPSValues(vector<double>& vals){
    if(vals.size()!==3)
      vals.resize(3);
    vals.at(0) = gps->getValues()[0];
    vals.at(1) = gps->getValues()[1];
    vals.at(2) = gps->getValues()[2];
  }

  void DisplayRobotPose(){
    vector<double> gpsdata;
    vector<double> lidardata;

    GetGPSValues(gpsdata);
    GetLidarData(lidardata);

    double py = gpsdata[2];
    double px = gpsdata[0];
    double pth = GetCompassHeading();

    for (auto ii:lidardata){
      double alpha = lidar_offsets[i];
      double rho = lidardata[ii];
      if(rho>LIDAR_SENSOR_MAX_RANGE)
        continue;
      
      // The Webots coordinate system doesn't match the robot-centric axes we're used to
      double rx = cos(alpha)*rho;
      double ry = -sin(alpha)*rho;

      // Convert detection from robot coordinates into world coordinates
      double wx =  cos(pth)*rx - sin(pth)*ry + px;
      double wy =  -(sin(pth)*rx + cos(pth)*ry) + py;

      //TODO
    //  if rho < LIDAR_SENSOR_MAX_RANGE:
      // Part 1.3: visualize map gray values.

      // You will eventually REPLACE the following 2 lines with a more robust version of the map
      // with a grayscale drawing containing more levels than just 0 and 1.
    //  display.setColor(0xFFFFFF)
    //  display.drawPixel(360-int(wy*30),int(wx*30))

    }
        // Draw the robot's current pose on the 360x360 display
    display->setColor(int(0xFF0000));
    display->drawPixel(360-int(py*30),int(px*30));
    
  }

  double GetCompassHeading(){
    // return inj radians
    return -atan2(compass->getValues()[0],compass->getValues()[2])+1.5708;
  }

  int StepSim() {
    return step(timeStep);
  }

  int GetTimeStep(){
    return timeStep;
  }
private:
  int timeStep;
  Motor* left_wheel;
  Motor* right_wheel;
  vector<Motor*> head_joints;
  Motor* torso_lift_joint;
  vector<Motor*> arm_jonits;  
  Node* robot_node;
  Display* display;
  Lidar* lidar;
  vector<double>lidar_offsets;
  GPS* gps;
  Compass* compass;
  Keyboard* keyboard;

  const double LIDAR_ANGLE_BINS = 667.0;
  const double LIDAR_SENSOR_MAX_RANGE = 2.75; // Meters
  const double LIDAR_ANGLE_RANGE = 240*(PI/180); // degree to radians
};

#endif // ROBOTINTERFACE_H__