#include <ros/ros.h>
#include "PublisherSubscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>

#define RAD2DEG(x) ((x)*180./M_PI)

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template<>
void PublisherSubscriber<std_msgs::UInt16, sensor_msgs::LaserScan>::subscriberCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
        double actualV = 0;
        std_msgs::UInt16 msg;

        // Anzahl der Messungen pro frame
        int count = scan->scan_time / scan->time_increment;

        // minimalen Abstand mit Randbedingungen finden
        for(int i = 0; i < count; i++)
        {
                // Werte nur die Winkeln zwischen -10deg und 10deg aus
                if(RAD2DEG(scan->angle_min + scan->angle_increment * i) < -5 || RAD2DEG(scan->angle_min + scan->angle_increment * i) > 5)
                {
                        continue;
                }
                else
                {
                        // Distanzmessung "inf" ignorieren
                        if(scan->ranges[i] == std::numeric_limits<float>::infinity())
                        {
                                continue;
                        }
                        // Notfallbremse wenn das Vorderfahrzeug zu nah oder zu weitentfernt ist
                        else if(scan->ranges[i] > 1 || scan->ranges[i] < 0.8)
                        {
                                actualV = 0;
                                ROS_INFO(": [%f, %f]", scan->ranges[i], actualV);
                                msg.data = actualV;
                                publisherObject.publish(msg);
                        }
                        // Berechnung des skalierten Geschwindigkeitswertes aus der eingegebenen Distanzmessung
                        else
                        {
                                actualV = map(scan->ranges[i], 0.8, 1, 550, 786);
                                ROS_INFO(": [%f, %f]", scan->ranges[i], actualV);
                                msg.data = actualV;
                                publisherObject.publish(msg);
                        }
                }
        }
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "geschwindigkeitsregulierungNode");
        PublisherSubscriber<std_msgs::UInt16,sensor_msgs::LaserScan>geschwindigkeitRegeln("/velocity","/scan",1000);
        ros::spin();

        return 0;
}
