#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <ctime>
#include <math.h>
#include "serialtest/sensor_value.h"
#include <geodesy/utm.h>


#include <novatel_gps_msgs/Inspvax.h>
#include <sensor_msgs/NavSatFix.h>

#define pi 3.14159

int sen_num1=0;
double sensorx1=0;
double sensory1=0;

int sen_num2=0;
double sensorx2=0;
double sensory2=0;

int sen_num3=0;
double sensorx3=0;
double sensory3=0;

using namespace std;


double d1=0.09;  //上车体原点到动臂
double a2=5.681;  //动臂
double a3=2.925;  //斗杆
double a4=1.494;   //铲斗



double Xe_last;
double Pe_last;
double YAW;





void write_callback(const novatel_gps_msgs::InspvaxPtr& msg);
void sensor_data(int argc, char **argv);
double cal_digger_angle(double *,double *,int num);
void tf2_publisher(double *position2,double *posture2,double * angle,double * partLength,tf2_ros::TransformBroadcaster br);
double yaw_kalman(double east_v,double north_v);



void tf_publish(string part1,string part2,double pos_x,double pos_y,double pos_z,double angle1,double angle2,double angle3,geometry_msgs::TransformStamped p2p,tf2::Quaternion q2q,tf2_ros::TransformBroadcaster br)
{

// part1--------->part2

	  p2p.header.frame_id = part1;
	  p2p.child_frame_id = part2;


	  p2p.transform.translation.x = pos_x;
	  p2p.transform.translation.y = pos_y;
	  p2p.transform.translation.z = pos_z;


	  q2q.setRPY(angle1, angle2, angle3);
	  p2p.transform.rotation.x = q2q.x();
	  p2p.transform.rotation.y = q2q.y();
	  p2p.transform.rotation.z = q2q.z();
	  p2p.transform.rotation.w = q2q.w();


          p2p.header.stamp = ros::Time::now();
          br.sendTransform(p2p);

}



void   control_points(double utm_easing,double utm_northing,double utm_altitude,double pitch,double roll,double azimuth,double speed_yaw,tf2_ros::TransformBroadcaster br)
{

         geometry_msgs::TransformStamped odom2baselink,baselink2uppart,baselink2downpart,down1,domn2,domn3,domn4,up1,up2,up3,up4,up5,up6;

	 tf2::Quaternion q_odom2baselink,q_baselink2uppart,q_baselink2downpart,q_down1,q_down2,q_down3,q_down4,q_up1,q_up2,q_up3,q_up4,q_up5,q_up6;

          double pitch2=pitch*pi/180;
          double roll2=roll*pi/180;
          double yaw2=azimuth*pi/180;


         ROS_INFO("UTM    easting: %f  northing:%f  azimuth:%f  speed_yaw:%f",utm_easing,utm_northing,yaw2,speed_yaw);


// map--------->odom

         tf_publish("map","odom",utm_easing,utm_northing,utm_altitude,pitch2,roll2,0,odom2baselink,q_odom2baselink,br);

// odom--------->baselink

         tf_publish("odom","baselink",0,0,-1,0,0,0,odom2baselink,q_odom2baselink,br);   //调整传感器的安装位置
// baselink--------->up_part


         tf_publish("baselink","up_part",0,0,1.5,0,0,yaw2,baselink2uppart,q_baselink2uppart,br);

// up_part--------->up1-up6


       tf_publish("up_part","up1",-0.3,1.3,0,0,0,0,up1,q_up1,br);
       tf_publish("up_part","up2",-1.5,1.3,0,0,0,0,up2,q_up2,br);
       tf_publish("up_part","up3",-1.5,-2.6,0,0,0,0,up3,q_up3,br);
       tf_publish("up_part","up4",0,-2.9,0,0,0,0,up4,q_up4,br);
       tf_publish("up_part","up5",1.4,-2.6,0,0,0,0,up5,q_up5,br);
       tf_publish("up_part","up6",1.4,0.85,0,0,0,0,up6,q_up6,br);


// baselink--------->down_part

       tf_publish("baselink","down_part",0,0,1,0,0,speed_yaw,baselink2downpart,q_baselink2downpart,br);

// down_part--------->down1-down4

       tf_publish("down_part","down1",1.5,2.2,0,0,0,0,up1,q_up1,br);          //履带外围长4317，宽2980  mm
       tf_publish("down_part","down2",-1.5,2.2,0,0,0,0,up2,q_up2,br);
       tf_publish("down_part","down3",-1.5,-2.2,0,0,0,0,up3,q_up3,br);
       tf_publish("down_part","down4",1.5,-2.2,0,0,0,0,up4,q_up4,br);

}
















void tf2_publisher(double *position2,double *posture2,double * angle,double * partLength,tf2_ros::TransformBroadcaster br)
{

          
          geometry_msgs::TransformStamped odom2baselink,baselink2dongbi,dongbi2dougan,dougan2chandou;

	  tf2::Quaternion q_odom2baselink,q_baselink2dongbi,q_dongbi2dougan,q_dougan2chandou;

// odom--------->baselink

	  odom2baselink.header.frame_id = "odom";
	  odom2baselink.child_frame_id = "baselink";


	  odom2baselink.transform.translation.x = position2[0];
	  odom2baselink.transform.translation.y = position2[1];
	  odom2baselink.transform.translation.z = position2[2];


	  q_odom2baselink.setRPY(posture2[0], posture2[1], posture2[2]);
	  odom2baselink.transform.rotation.x = q_odom2baselink.x();
	  odom2baselink.transform.rotation.y = q_odom2baselink.y();
	  odom2baselink.transform.rotation.z = q_odom2baselink.z();
	  odom2baselink.transform.rotation.w = q_odom2baselink.w();


          odom2baselink.header.stamp = ros::Time::now();
          br.sendTransform(odom2baselink);





// baselink--------->动臂
	  baselink2dongbi.header.frame_id = "baselink";
	  baselink2dongbi.child_frame_id = "dongbi";


	  baselink2dongbi.transform.translation.x = partLength[0];
	  baselink2dongbi.transform.translation.y = 0.0;
	  baselink2dongbi.transform.translation.z = 0.0;


	  q_baselink2dongbi.setRPY(0, angle[0], 0);
	  baselink2dongbi.transform.rotation.x = q_baselink2dongbi.x();
	  baselink2dongbi.transform.rotation.y = q_baselink2dongbi.y();
	  baselink2dongbi.transform.rotation.z = q_baselink2dongbi.z();
	  baselink2dongbi.transform.rotation.w = q_baselink2dongbi.w();


         baselink2dongbi.header.stamp = ros::Time::now();
         br.sendTransform(baselink2dongbi);

// 动臂--------->斗杆
	  dongbi2dougan.header.frame_id = "dongbi";
	  dongbi2dougan.child_frame_id = "dougan";


	  dongbi2dougan.transform.translation.x = partLength[1];
	  dongbi2dougan.transform.translation.y = 0.0;
	  dongbi2dougan.transform.translation.z = 0.0;

	  q_dongbi2dougan.setRPY(0, angle[1], 0);
	  dongbi2dougan.transform.rotation.x = q_dongbi2dougan.x();
	  dongbi2dougan.transform.rotation.y = q_dongbi2dougan.y();
	  dongbi2dougan.transform.rotation.z = q_dongbi2dougan.z();
	  dongbi2dougan.transform.rotation.w = q_dongbi2dougan.w();


         dongbi2dougan.header.stamp = ros::Time::now();
         br.sendTransform(dongbi2dougan);

// 斗杆--------->铲斗
	  dougan2chandou.header.frame_id = "dougan";
	  dougan2chandou.child_frame_id = "chandou";


	  dougan2chandou.transform.translation.x = partLength[2];
	  dougan2chandou.transform.translation.y = 0.0;
	  dougan2chandou.transform.translation.z = 0.0;



	  q_dougan2chandou.setRPY(0, angle[2], 0);

	  dougan2chandou.transform.rotation.x = q_dougan2chandou.x();
	  dougan2chandou.transform.rotation.y = q_dougan2chandou.y();
	  dougan2chandou.transform.rotation.z = q_dougan2chandou.z();
	  dougan2chandou.transform.rotation.w = q_dougan2chandou.w();


         dougan2chandou.header.stamp = ros::Time::now();
         br.sendTransform(dougan2chandou);
}









double cal_digger_angle(double *roll,double *pitch,int num)
{


        double angle=0;

        double pitch0=pitch[0];
        double roll0=roll[0];
        double pitch1=pitch[1];
        double roll1=roll[1];
/*
        int np=1;
        if(pitch0*roll0 < 0)
          np=-1;

        int np2=1;
        if(pitch1*roll1 < 0)
          np2=-1;

*/
        int np=1;
        if(pitch0 < 0)
          np=-1;

        int np2=1;
        if(pitch1 < 0)
          np2=-1;





      return angle=sqrt(pitch0*pitch0+roll0*roll0)*np-sqrt(pitch1*pitch1+roll1*roll1)*np2;

      //  return angle=sqrt(pitch0*pitch0+roll0*roll0);
      //  return angle=pitch0/cos(roll0/57.3);

    //  position,double *posture
   


}



//回调函数
void write_callback(const novatel_gps_msgs::InspvaxPtr& msg)
{

 //   ROS_INFO("latitude: %f  longitude:%f  altitude:%f pitch:%f roll:%f yaw:%f n_ve:%f e_ve:%f u_ve:%f" ,msg->latitude,msg->longitude,msg->altitude,msg->pitch,msg->roll,msg->azimuth,msg->north_velocity,msg->east_velocity,msg->up_velocity);

// ROS_INFO("WGS84  latitude: %f  longitude:%f  altitude:%f",msg->latitude,msg->longitude,msg->altitude);


  geographic_msgs::GeoPoint geo_pt;
  geo_pt.latitude = msg->latitude;
  geo_pt.longitude = msg->longitude;
  geo_pt.altitude = msg->longitude;
  geodesy::UTMPoint utm_pt(geo_pt);

  double utm_easing = utm_pt.easting;
  double utm_northing = utm_pt.northing;
  double utm_altitude = utm_pt.altitude;
  double utm_zone = utm_pt.zone;
  double utm_band = utm_pt.band;


  tf2_ros::TransformBroadcaster br;



  double speed_yaw = yaw_kalman(msg->north_velocity,msg->east_velocity);  //对南北速度进行了对调


//  ROS_INFO("UTM    easting: %f  northing:%f  altitude:%f  speed_yaw:%f",utm_easing,utm_northing,utm_altitude,speed_yaw);

  control_points(utm_easing,utm_northing,utm_altitude,msg->pitch,msg->roll,msg->azimuth,speed_yaw,br);

}


double yaw_kalman(double east_v,double north_v)       //使用卡尔曼滤波对速度角度进行滤波优化
{

    double YAW;
    double YAW_b2;
    double Q;       //
    double R;
    double Xp;      //预测
    double Pe;       //协方差
    double K;        //增益
    double Pp;    //   协方差矩阵
    double Xe;    //估计

    double theta=atan(north_v/east_v);          // 关于角度的零点需要实验重新验证
    double Average_V=sqrt(north_v*north_v + east_v*east_v);  


    double old_YAW=YAW;

    if (east_v > 0)  
    {                           //判断gps解算方位角的范围 （正东为0度，逆时针增大）
        if  (theta > 0)
            YAW= theta;
        else if (theta <= 0)
            YAW=pi*2+theta;
        else
            YAW=pi*2+theta;
    }       
    else if (east_v < 0)
        YAW=theta+pi;
    else 
        YAW=old_YAW;


    YAW_b2 = YAW*180/pi;
     

    if (Average_V <= 0.16)
    { 
        Q = 0.0625/10000;
        R = 9*10000;
    }    
    else
    {    
        Q = 0.08;    //预测
        R = 7;          //测量
    }    

   // 对角度卡尔曼滤波
    
    Xp=Xe_last;                //根据估计值计算预测值
    Pp=Pe_last+Q;             //计算预测值与真实值之间误差的协方差矩阵
    K=Pp/(Pp+R);              //计算卡尔曼增益
         
  //  err=YAW_b2-Xp;

    Xe=Xp+K*(sin((YAW_b2-Xp)/57.3)*40);   //计算估计值  Xp->bias
    Pe=Pp-K*Pp;                //计算估计值与真实值之间误差的协方差矩阵
    
    Xe_last=Xe;
    Pe_last=Pe;  




    int fg=Xe*100;
    double test2=fg%36000;
    double test=test2/100;

    if(test < 0)
       test=test+360;
     
    double yaw_ang=test*pi/180;


    ROS_INFO("data    Xe: %f  test:%f   yaw_ang:%f",Xe,test,yaw_ang);
   
    return yaw_ang;

}







void sensor_data(int argc, char **argv)
{

 //   int argc;
 //   char **argv;  
    //初始化节点
    ros::init(argc, argv, "bewis_sensor_read");
    //声明节点句柄
    ros::NodeHandle nh;



    ros::Subscriber sub = nh.subscribe("/inspvax",1000,write_callback);
   // ros::Subscriber sub = nh.subscribe("/inspvax",1000,tf2_publisher);
    tf2_ros::TransformBroadcaster br;
   // tf2_ros::TransformBroadcaster br2;


    ros::spin();

/*
    ros::Rate r(50); // 调整频率 hz
    while (ros::ok())
    {


         double digger_angle;

         double pitch1=sensorx1;     
         double pitch2=sensorx2; 
         double pitch3=sensorx2; 

         double roll1 =sensory1;     
         double roll2=sensory2; 
         double roll3=sensory2; 



         double roll[2]={roll1,roll2};
         double pitch[2]={pitch1,pitch2};

         digger_angle=cal_digger_angle(roll,pitch,2);

   //      printf("extern data sensor:----Sensor number: %d  ---Angle_x: %3.3f  ---Angle_y:  %3.3f\r\n",sen_num1,sensorx1,sensory1);
  //       printf("extern data sensor:----Sensor number: %d  ---Angle_x: %3.3f  ---Angle_y:  %3.3f\r\n",sen_num2,sensorx2,sensory2);
   //      printf("extern data sensor:----Sensor number: %d  ---Angle_x: %3.3f  ---Angle_y:  %3.3f\r\n",sen_num3,sensorx3,sensory3);

         printf("digger_angle:----Sensor number: %3.3f  \r\n",digger_angle);

         digger_angle=digger_angle/57.3;



         double angleP[3]={digger_angle,digger_angle,digger_angle};
         double partLengthP[3]={5.7,6.8,1.8};

         double position[3]={0,0,0};  //rtk位置（转换为m）
         double posture[3]={0,0,0};   //rtk姿态

         tf2_publisher(position,posture,angleP,partLengthP,br);



         r.sleep();

    }


*/


}

