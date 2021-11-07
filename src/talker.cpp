/******************************************************************************
 * MIT License
 * 
 * Copyright (c) 2021 Mayank Joshi
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 ******************************************************************************/

/**
 * @file talker.cpp
 * @author mayankJoshi (mayankjoshi63@gmail.com)
 * @brief Initializes a "talker_node" which publishes on topic "/chatter"
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <beginner_tutorials/custom_string.h>

#include <sstream>


/**
 * Default String published by talker
 */
extern std::string message = "Hi There, you're awesome";

/**
 * Default loop frequency, Hz
 */
int default_freq = 10;

/**
 * @brief Function to call service
 * @param request Input string to service
 * @param response Response sent by service 
 * @return bool
 */
bool modify_msg(beginner_tutorials::custom_string::Request  &request,
         beginner_tutorials::custom_string::Response &response) {
  response.updated_string = request.input_string;
  message = response.updated_string;
  ROS_INFO_STREAM("Default message by talker modified to: " << message);
  return true;
}

int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker_node");

  // variable to store the loop frequency, set to default_freq
  int talker_freq = default_freq;

  if ( argc > 1 ) {
    // set frequency according to parsed argument
    talker_freq = atoi(argv[1]);
  }

  if ( talker_freq > 0 ) {
    ROS_DEBUG_STREAM("talker publishing at "<< talker_freq << " Hz");

  } else if ( talker_freq == 0 ) {
    ROS_ERROR_STREAM("talker expects non-zero frequency");
    ROS_WARN_STREAM("talker frequency set to default value of "
    << default_freq << " Hz");

    // set loop frequency to default value
    talker_freq = default_freq;

  } else if ( talker_freq < 0 ) {
    ROS_FATAL_STREAM("talker expects positive value of frequency");
    ROS_WARN_STREAM("talker frequency set to default value of "
    << default_freq << " Hz");

    // set loop frequency to default value
    talker_freq = default_freq;
  }

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
 * The advertise() function is how you tell ROS that you want to
 * publish on a given topic name. This invokes a call to the ROS
 * master node, which keeps a registry of who is publishing and who
 * is subscribing. After this advertise() call is made, the master
 * node will notify anyone who is trying to subscribe to this topic name,
 * and they will in turn negotiate a peer-to-peer connection with this
 * node.  advertise() returns a Publisher object which allows you to
 * publish messages on that topic through a call to publish().  Once
 * all copies of the returned Publisher object are destroyed, the topic
 * will be automatically unadvertised.
 *
 * The second parameter to advertise() is the size of the message queue
 * used for publishing messages.  If messages are published more quickly
 * than we can send them, the number here specifies how many messages to
 * buffer up before throwing some away.
 */

  ros::Publisher chatter_pub =
  n.advertise<std_msgs::String>("/chatter", 10);


  auto server = n.advertiseService("custom_string", modify_msg);

  ros::Rate loop_rate(talker_freq);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << message << " " << count;
    msg.data = ss.str();

    ROS_INFO_STREAM(msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
