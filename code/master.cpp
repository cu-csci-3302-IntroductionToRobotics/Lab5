#include "master.h"

master::master(){
    // attach to robot interface
    robot.reset(new RobotInterface);
}

void master::Run(){

double MAX_SPEED = 7.0;  // [rad/s]
double MAX_SPEED_MS = 0.633; // [m/s]
double AXLE_LENGTH = 0.4044; // m
int MOTOR_LEFT = 10;
int MOTOR_RIGHT = 11;
int N_PARTS = 12;

// Odometry
double pose_x = 0;
double pose_y = 0;
double pose_theta = 0;

double vL = 0;
double vR = 0;

map = None

////////////////////////////////////////// IMPORTANT //////////////////////////////////////////
// Set the mode here. Please change to 'autonomous' before submission
mode_t mode = mode_t::manual; // Part 1.1: manual mode
//mode_t mode = mode_t::autonomous;
//mode_t mode = mode_t::planner;

//////////////////////////////////////
//
// Planner
//
//////////////////////////////////////
switch (mode)
{
case mode_t::planner :
    // Part 2.3: Provide start and end in world coordinate frame and convert it to map's frame
    //start_w = None; // (Pose_X, Pose_Z) in meters
    //end_w = None; // (Pose_X, Pose_Z) in meters

    // Convert the start_w and end_w from the webots coordinate frame into the map frame
    //start = None // (x, y) in 360x360 map
    //end = None // (x, y) in 360x360 map

    // Part 2.3: Implement A* or Dijkstra's Algorithm to find a path
    //def path_planner(map, start, end):
    //    :param map: A 2D numpy array of size 360x360 representing the world's cspace with 0 as free space and 1 as obstacle
    //    :param start: A tuple of indices representing the start cell in the map
    //    :param end: A tuple of indices representing the end cell in the map
    //    :return: A list of tuples as a path from the given start to the given end in the given maze

    // Part 2.1: Load map (map.npy) from disk and visualize it


    // Part 2.2: Compute an approximation of the “configuration space”


    // Part 2.3 continuation: Call path_planner


    // Part 2.4: Turn paths into waypoints and save on disk as path.npy and visualize it
    //waypoints = []

    ////////////////////////////////////////////
    //
    // Map Initialization
    //
    ////////////////////////////////////////////

    // Part 1.2: Map Initialization

    // Initialize your map data structure here as a 2D floating point array
    //map = None // Replace None by a numpy 2D floating point array
    //waypoints = []

    break;
case: mode_t::planner:
    break;

case: mode_t::autonomous:
    // Part 3.1: Load path from disk and visualize it
    //waypoints = [] // Replace with code to load your path
    break;
default:
    break;
}

state = 0 // use this to iterate through your path


while((robot->StepSim() != -1)&& (mode!=mode_t::planner)) {  

    //////////////////////////////////////
    //
    // Mapping
    //
    //////////////////////////////////////

    //////////////////////////////// v [Begin] Do not modify v ////////////////////////////////////
    // Ground truth pose
    vector<double> gps_vals;
    robot->getGPSValues(gps_vals);
    pose_y = gps_vals[2];
    pose_x = gps.vals[0];

    pose_theta = robot->getCompassHeading();

    vector<double> lidardata;
    robot->GetLidarData(lidardata);

    //////////////////////////////////////
    //
    // Controller
    //
    //////////////////////////////////////
    if(mode == mode_t::manual){
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            // Part 1.4: Filter map and save to filesystem

            print("Map file saved")
        elif key == ord('L'):
            // You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load("map.npy")
            print("Map loaded")
        else: // slow down
            vL *= 0.75
            vR *= 0.75
    else: // not manual mode
        // Part 3.2: Feedback controller
        //STEP 1: Calculate the error
        rho = 0
        alpha = -(math.atan2(waypoint[state][1]-pose_y,waypoint[state][0]-pose_x) + pose_theta)


        //STEP 2: Controller
        dX = 0
        dTheta = 0

        //STEP 3: Compute wheelspeeds
        vL = 0
        vR = 0

        // Normalize wheelspeed
        // (Keep the wheel speeds a bit less than the actual platform MAX_SPEED to minimize jerk)


    // Odometry code. Don't change vL or vR speeds after this line.
    // We are using GPS and compass for this lab to get a better pose but this is how you'll do the odometry
    pose_x += (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.cos(pose_theta)
    pose_y -= (vL+vR)/2/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0*math.sin(pose_theta)
    pose_theta += (vR-vL)/AXLE_LENGTH/MAX_SPEED*MAX_SPEED_MS*timestep/1000.0

    // print("X: %f Z: %f Theta: %f" % (pose_x, pose_y, pose_theta))

    // Actuator commands
    robot_parts[MOTOR_LEFT].setVelocity(vL)
    robot_parts[MOTOR_RIGHT].setVelocity(vR)




























state_t state = state_t::follow_line;

double LIDAR_SENSOR_MAX_RANGE = 3; // Meters
double LIDAR_ANGLE_BINS = 21; // 21 Bins to cover the angular range of the lidar, centered at 10
double LIDAR_ANGLE_RANGE = 1.5708; // 90 degrees, 1.5708 radians

// These are your pose values that you will update by solving the odometry equations
double pose_x = 0.197;
double pose_y = 0.678;
double pose_theta = 0;

// ePuck Constants
double EPUCK_AXLE_DIAMETER = 0.053; // ePuck's wheels are 53mm apart.
double MAX_SPEED = 6.28;


////////// Part 1: Setup Data structures
//
// Create an empty list for your lidar sensor readings here,
// as well as an array that contains the angles of each ray 
// in radians. The total field of view is LIDAR_ANGLE_RANGE,
// and there are LIDAR_ANGLE_BINS. An easy way to generate the
// array that contains all the angles is to use linspace from
// the numpy package.



//////// End of Part 1 //////////

double vL = 0; // Left wheel velocity in rad/s
double vR = 0; // Right wheel velocity in rad/s

double EPUCK_MAX_WHEEL_SPEED;
double dsr;
double dsl;
double ds;

// vector to store ground sensor readings
vector<double> gsens;
 
// Main Control Loop:
while (robot->StepSim() != -1) {  
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                 Sensing                           //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Read ground sensors
    robot->getGroundSensors(gsens);

    // Read Lidar data        
    cv::Mat lidar_readings;
    robot->getLidarRangeImage(lidar_readings);

    ////////// Part 2: Turn world coordinates into map coordinates
    //
    // Come up with a way to turn the robot pose (in world coordinates)
    // into coordinates on the map. Draw a red dot using display.drawPixel()
    // where the robot moves.
    

    
    
    ////////// Part 3: Convert Lidar data into world coordinates
    //
    // Each Lidar reading has a distance rho and an angle alpha.
    // First compute the corresponding rx and ry of where the lidar
    // hits the object in the robot coordinate system. Then convert
    // rx and ry into world coordinates wx and wy. 
    // The arena is 1x1m2 and its origin is in the top left of the arena. 
    

    
    
    ////////// Part 4: Draw the obstacle and free space pixels on the map
 
    
          

    
 

    
    // DO NOT MODIFY THE FOLLOWING CODE
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                 Robot controller                  //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////

    switch (state)
    {
    case state_t::follow_line :
        if(gsens[1]<350 && gsens[0]>400 && gsens[2]>400){
            vL = MAX_SPEED*0.3;
            vR = MAX_SPEED*0.3;
        } else if(gsens[0]<500 && gsens[1]<500 && gsens[2]<500){
            vL = MAX_SPEED*0.3;
            vR = MAX_SPEED*0.3;
            // save currently displayed image to file
            robot->SaveDisplayImageToFile("map.png");
        } else if(gsens[2]<650){
            vL = MAX_SPEED*0.2;
            vR = -MAX_SPEED*0.05;
        } else if(gsens[0]<650){
            vL = -MAX_SPEED*0.05;
            vR = MAX_SPEED*0.2;
        }
        break;
    case state_t::stop :
        vL = 0.0;
        vR = 0.0;
        break;
    default:
        vL = 0.0;
        vR = 0.0;
        break;
    }

    robot->SetLeftMotorSpeed(vL);
    robot->SetRightMotorSpeed(vR);

}
}