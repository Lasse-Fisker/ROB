#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;
std_msgs::Bool grabbedSomethingMsg;
boolean grabbedSomething = false;
ros::Publisher grabber_strain_gauge_publisher("grabber_something", &grabbedSomethingMsg);

void setup()
{
  nh.initNode();
  nh.advertise(grabber_strain_gauge_publisher);
}

void loop()
{
  checkStrain();
  delay(10);
  nh.spinOnce();
 
}

void checkStrain() {
    //TODO: check how high the strain is
    
    
    if(grabbedSomething) {
        publish();
    }
}

void publish() {
    grabbedSomethingMsg.data = true;
    grabber_strain_gauge_publisher.publish( &grabbedSomethingMsg );
}


