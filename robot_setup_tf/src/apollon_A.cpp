#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tinyxml.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_apollon");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  tf::Quaternion q0, q1, q2, q3;
  q0 = tf::Quaternion(0,0,0,1);

  while(n.ok()){

    q1.setEuler(0, 0, -M_PI/2);		// -90 um z
    q2.setEuler(-M_PI/2, 0, 0);		// -90 um y
    q3.setEuler(0, M_PI/2, 0);		// -90 um x

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q3, tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(), "A_INIT", "a_center"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q3, tf::Vector3(0.0, 0.0, 0.0)),
        ros::Time::now(), "W_INIT", "GLOBAL_W"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(q0, tf::Vector3(0.0, 0.1605, 0.1795)),
        ros::Time::now(), "a_center", "a_front"));

    q1.setEuler(-M_PI/2, 0, 0);		// -90 um y
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00170805311731, -0.710246489572, 0.000929089546105, 0.703950384188), tf::Vector3(-0.205393020942, 0.000999206289185, -0.179882410109)),
        ros::Time::now(), "a_front", "a_right"));

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00170805311731, -0.710246489572, 0.000929089546105, 0.703950384188), tf::Vector3(-0.205393020942, 0.000999206289185, -0.179882410109)) *
    		tf::Transform(tf::Quaternion(0.00305976536884, -0.710746807118, 0.000907014114909, 0.703440680749), tf::Vector3(-0.186236715416, -0.000604960294292, -0.202045462026)),
        ros::Time::now(), "a_front", "a_back"));

    q1.setEuler(M_PI/2, 0, 0);		// 90 um y
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0.00224344529337, 0.71061150032, -0.00486901720005, 0.7035641799), tf::Vector3(0.199856805676, -0.000468722798459, -0.185993278449)),
        ros::Time::now(), "a_front", "a_left"));

//  HAND-EYE-Calibration ----> mit 45 Grad

    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(-0.0127584, -0.00300246, 0.999895, 0.00620332), tf::Vector3(0.0641093, 0.333541, 0.180437)),
        ros::Time::now(),"a_center", "a_cam_left"));

        broadcaster.sendTransform(
          tf::StampedTransform(
        tf::Transform(tf::Quaternion(-0.0020987449, 0.000085006, 0.0017530716, 0.99999626), tf::Vector3(0.13513077136, -0.0023260125, -0.00113476)),
        ros::Time::now(),"a_cam_left", "a_cam_right"));


    r.sleep();
  }
}
