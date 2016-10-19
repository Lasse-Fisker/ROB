#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;
std_msgs::Bool grabbedSomethingMsg;

boolean grabbedSomething = false;
boolean shouldCheck = false;

ros::Publisher grabber_strain_gauge_publisher("grabbed_something", &grabbedSomethingMsg);
int strainGaugePin = A0;

void handleCheckPressureCb(const std_msgs::Bool& msg) {  
      shouldCheck = msg.data;
}

ros::Subscriber<std_msgs::Bool> grabber_strain_gauge_subscriber("grabber_check_pressure", &handleCheckPressureCb);

void setup()
{
  nh.initNode();
  nh.advertise(grabber_strain_gauge_publisher);
}

void loop()
{
    if(shouldCheck){
        checkStrain();
    }
    
    nh.spinOnce();
  
    delay(10);
}

void checkStrain() {
    
    //read pressure, publish if high enough. TODO: test values
    if(analogRead(strainGaugePin) > 400) {
        publish();
    }
}

void publish() {
    grabbedSomethingMsg.data = true;
    grabber_strain_gauge_publisher.publish( &grabbedSomethingMsg );
}


