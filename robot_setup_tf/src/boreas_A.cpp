#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tinyxml.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_boreas");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  tf::Quaternion q0, q1, q2, q3;
  q0 = tf::Quaternion(0,0,0,1);

  while(n.ok()){

    q1.setEuler(0, 0, -M_PI/2);		// -90 um z
    q2.setEuler(-M_PI/2, 0, 0);		// -90 um y
    q3.setEuler(0, M_PI/2, 0);		// -90 um x

    //broadcaster.sendTransform(
      //tf::StampedTransform(
        //tf::Transform(q3, tf::Vector3(0.0, 0.0, 0.0)),
        //ros::Time::now(), "B_INIT", "b_center"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q0, tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(), "c2c", "b_center"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q0, tf::Vector3(0.0, 0.1590, 0.1800)),
        ros::Time::now(), "b_center", "b_front"));

    q1.setEuler(-M_PI/2, 0, 0);		// -90 um y
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00170805311731, -0.710246489572, 0.000929089546105, 0.703950384188), tf::Vector3(-0.205393020942, 0.000999206289185, -0.179882410109)),
        ros::Time::now(), "b_front", "b_right"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00170805311731, -0.710246489572, 0.000929089546105, 0.703950384188), tf::Vector3(-0.205393020942, 0.000999206289185, -0.179882410109)) *
    		tf::Transform(tf::Quaternion(0.00305976536884, -0.710746807118, 0.000907014114909, 0.703440680749), tf::Vector3(-0.186236715416, -0.000604960294292, -0.202045462026)),
        ros::Time::now(), "b_front", "b_back"));

    q1.setEuler(M_PI/2, 0, 0);		// 90 um y
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00224344529337, 0.71061150032, -0.00486901720005, 0.7035641799), tf::Vector3(0.199856805676, -0.000468722798459, -0.185993278449)),
        ros::Time::now(), "b_front", "b_left"));

//  HAND-EYE-Calibration ----> mit 45 Grad

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.0110223, -0.000916933, 0.999814, 0.0158018), tf::Vector3(0.0709266, 0.331716, 0.176129)),
        ros::Time::now(),"b_center", "b_cam_left"));

        broadcaster.sendTransform(
          tf::StampedTransform(
        tf::Transform(tf::Quaternion(-0.0015736078, -0.001099246, 0.0009769036, 0.99999768), tf::Vector3(0.13877713, -0.00172176, -0.000747077)),
        ros::Time::now(),"b_cam_left", "b_cam_right"));


    r.sleep();
  }
}
