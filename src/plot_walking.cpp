#include <ros/ros.h>
#include <Eigen/Eigen>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

using namespace std;
Eigen::Vector2d angle;
Eigen::Vector2d angularVelocity;
Eigen::Vector2d _angle;
Eigen::Vector2d _angularVelocity;
ros::Subscriber bipedStateSub;
ros::Publisher robotPlotPub;
ros::Publisher terrainPlotPub;
Eigen::Matrix<double,3,3> Rt;
// double terrainAngle = -2*M_PI/180;
double terrainAngle;
Eigen::Matrix<double,4,4> Tt;
double length = 1.0;

void bipedStateCallback(std_msgs::Float32MultiArray msg){
    _angle = angle;
    _angularVelocity = angularVelocity;
    angle << msg.data[0],msg.data[1];
    angularVelocity << msg.data[2],msg.data[3];
}


void supportTransfer(){
    if(abs(angle(0)-_angle(0)) > 0.3){// 发生碰撞
        double walkingDistanceX = (sin(_angle(0))*length-sin(_angle(1))*length);
        double walkingDistanceY = (cos(_angle(0))*length-cos(_angle(1))*length);
        Eigen::Matrix<double,4,4> Tstep;
        Tstep << 1,0,0,walkingDistanceX,
                 0,1,0,walkingDistanceY,
                 0,0,1,0,
                 0,0,0,1;
        Tt = Tt * Tstep;
    }
}

void initT(){
    Rt << cos(terrainAngle) ,0 ,-sin(terrainAngle),
       0                 ,1 ,0                 ,
       sin(terrainAngle) ,0 ,cos(terrainAngle) ;
    Tt << cos(terrainAngle) ,0 ,-sin(terrainAngle),0,
        0                 ,1 ,0                 ,0,
        sin(terrainAngle) ,0 ,cos(terrainAngle) ,0,
        0                 ,0 ,0                 ,1;
    Eigen::Matrix<double,4,4> T;
    T << 1,0,0,-30,
            0,1,0,0,
            0,0,1,0,
            0,0,0,1;
    Tt = Tt * T;
}

void plotRobot(){
    visualization_msgs::MarkerArray markerArray;
    Eigen::Matrix<double,4,4> Ttst;
    Ttst << cos(M_PI/2-angle(0)), 0,-sin(M_PI/2-angle(0))  , 0 ,
           0                   , 1, 0                     , 0 ,
           sin(M_PI/2-angle(0)), 0, cos(M_PI/2-angle(0))  , 0 ,
           0                   , 0, 0                     , 1 ;
    Eigen::Matrix<double,4,4> Tst = Tt*Ttst;
    Eigen::Vector4d stCenter;
    stCenter = Tst*Eigen::Vector4d(0.5*length,0,0,1);
    // cout << stCenter(1) - cos(angle(0))*length/2 << endl;
    Eigen::Quaterniond quaternionst(Tst.block<3,3>(0,0));
    Eigen::Matrix<double,4,4> Tstsw;
    Tstsw << cos(-M_PI+angle(0)-angle(1)) ,0,-sin(-M_PI+angle(0)-angle(1)), length,
             0                            ,1,0                            , 0     ,
             sin(-M_PI+angle(0)-angle(1)) ,0,cos(-M_PI+angle(0)-angle(1)) , 0     ,
             0                            ,0,0                            , 1     ;
    Eigen::Vector4d swCenter;
    Eigen::Matrix<double,3,3> Rtsw;
    Rtsw << cos(M_PI/2-angle(1)) ,0 ,-sin(M_PI/2-angle(1)),
           0                    ,1 ,0                    ,
           sin(M_PI/2-angle(1)) ,0 ,cos(M_PI/2-angle(1)) ;
    Eigen::Matrix<double,3,3> Rsw = Rt*Rtsw;
    Eigen::Quaterniond quaternionsw(Rsw);
    swCenter = Tst*Tstsw*Eigen::Vector4d(0.5*length,0,0,1);
    // cout << "sw" << swCenter(0) << endl;
    // cout << swCenter(0) - (sin(angle(0))*length-sin(angle(1))*length/2) << endl;
    for(int i = 0;i < 2;i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = i;
        marker.type =  visualization_msgs::Marker::CUBE;
        marker.action =  visualization_msgs::Marker::ADD;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        
        if(i == 0){
            marker.pose.position.x = stCenter(0);
            marker.pose.position.y = stCenter(1);
            marker.pose.position.z = stCenter(2);
            marker.pose.orientation.x = quaternionst.x();
            marker.pose.orientation.y = quaternionst.y();
            marker.pose.orientation.z = quaternionst.z();
            marker.pose.orientation.w = quaternionst.w();
        }else{
            marker.pose.position.x = swCenter(0);
            marker.pose.position.y = swCenter(1);
            marker.pose.position.z = swCenter(2);
            marker.pose.orientation.x = quaternionsw.x();
            marker.pose.orientation.y = quaternionsw.y();
            marker.pose.orientation.z = quaternionsw.z();
            marker.pose.orientation.w = quaternionsw.w();
        }
        marker.scale.x = length;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        markerArray.markers.push_back(marker);    
    }
    robotPlotPub.publish(markerArray);
}

void plotTerrain(){
    Eigen::Quaterniond quaterniont(Rt);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = quaterniont.x();
    marker.pose.orientation.y = quaterniont.y();
    marker.pose.orientation.z = quaterniont.z();
    marker.pose.orientation.w = quaterniont.w();

    marker.scale.x = 100.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.001;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1;
    terrainPlotPub.publish(marker);
}

void plot(){
    ros::NodeHandle n;
    bipedStateSub = n.subscribe("biped_state", 10, bipedStateCallback);
    terrainPlotPub = n.advertise<visualization_msgs::Marker>("terrain_marker",10);
    robotPlotPub = n.advertise<visualization_msgs::MarkerArray>("biped_marker",10);
    ros::Rate loop_rate(1000);
    while(ros::ok()){
        supportTransfer();
        plotRobot();
        plotTerrain();
        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc,char **argv){
    ros::init(argc,argv,"plot_walking");
    double slope;
    ros::param::get("slope", slope);
    terrainAngle = -slope*M_PI/180;
    initT();// 初始化斜面R和T
    plot();
    return 0;
}