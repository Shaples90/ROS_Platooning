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
        // Parameter für Arbeitsflächeäc
        double distanceMin = 0.8;
        double distanceMax = 1;
        double angleLeftMax = 10;
        double angleRightMax = -10;

        double distanceMeasureMin = 12;
        double angle = 0;
        double actualV = 0;
        int lv = 0;
        std_msgs::UInt16 msg;

        // Anzahl der Messungen pro frame
        int count = scan->scan_time / scan->time_increment;

        // minimalen Abstand mit Randbedingungen finden
        for(int i = 0; i < count; i++)
        {
                if(        scan->ranges[i] < distanceMeasureMin
                        && RAD2DEG(scan->angle_min + scan->angle_increment * i) >= angleRightMax
                        && RAD2DEG(scan->angle_min + scan->angle_increment * i) <= angleLeftMax
                        && scan->ranges[i] != std::numeric_limits<float>::infinity())
                {
                        distanceMeasureMin = scan->ranges[i];
                        lv = i;
                }
                else
                {
                        continue;
                }
        }

        // zum minimalen Abstand dazugehöriger Winkeln
        angle = RAD2DEG(scan->angle_min + scan->angle_increment * lv);
        
        // Notfallbremse wenn das Vorderfahrzeug zu nah oder zu weitentfernt ist
        if(distanceMeasureMin > distanceMax || distanceMeasureMin < distanceMin)
        {
                actualV = 0;
        }

        // Berechnung des skalierten Geschwindigkeitswertes aus der eingegebenen Distanzmessung
        else
        {
                actualV = map(distanceMeasureMin, distanceMin, distanceMax, 550, 786);
        }

        // Daten an /velocity publishen
        ROS_INFO(": [%f, %f]",angle ,distanceMeasureMin);
        msg.data = actualV;
        publisherObject.publish(msg);
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "geschwindigkeitsregulierungNode");
        PublisherSubscriber<std_msgs::UInt16,sensor_msgs::LaserScan>geschwindigkeitRegeln("/velocity","/scan",1000);
        ros::spin();

        return 0;
}
