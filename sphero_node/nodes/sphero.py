#!/usr/bin/python
#***********************************************************
# * Software License Agreement (BSD License)
# *
# *  Copyright (c) 2012, Melonee Wise
# *  All rights reserved.
# *
# *  Redistribution and use in source and binary forms, with or without
# *  modification, are permitted provided that the following conditions
# *  are met:
# *
# *   * Redistributions of source code must retain the above copyright
# *     notice, this list of conditions and the following disclaimer.
# *   * Redistributions in binary form must reproduce the above
# *     copyright notice, this list of conditions and the following
# *     disclaimer in the documentation and/or other materials provided
# *     with the distribution.
# *
# *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# *  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************
# author: Melonee Wise

import rospy

import math
import sys
import tf
import PyKDL

from sphero_driver import sphero_driver
# import dynamic_reconfigure.server

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, TwistWithCovariance, Vector3, Pose2D
from sphero_node.msg import SpheroCollision
from std_msgs.msg import ColorRGBA, Float32, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
# from sphero_node.cfg import ReconfigConfig
from hgext.gpg import sign


class SpheroNode(object):
  battery_state = {1:"Battery Charging",
                    2:"Battery OK",
                    3:"Battery Low",
                    4:"Battery Critical"}


  ODOM_POSE_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3]


  ODOM_TWIST_COVARIANCE = [1e-3, 0, 0, 0, 0, 0,
                           0, 1e-3, 0, 0, 0, 0,
                           0, 0, 1e6, 0, 0, 0,
                           0, 0, 0, 1e6, 0, 0,
                           0, 0, 0, 0, 1e6, 0,
                           0, 0, 0, 0, 0, 1e3]

  def __init__(self):
    rospy.init_node('sphero')
    
    self.is_connected = False
    self._init_pubsub()
    self._init_params()
    self.robot = sphero_driver.Sphero()
    self.imu = Imu()
    self.imu.orientation_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
    self.imu.angular_velocity_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
    self.imu.linear_acceleration_covariance = [1e-6, 0, 0, 0, 1e-6, 0, 0, 0, 1e-6]
    self.last_cmd_vel_time = self.last_diagnostics_time = rospy.Time.now()
    self.cmd_heading = 0
    self.cmd_speed = 0
    self.target_speed = 0.0
    self.current_speed = 0.0
    self.power_state_msg = "No Battery Info"
    self.power_state = 0

  def _init_pubsub(self):
    self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
    self.imu_pub = rospy.Publisher('imu_data', Imu, queue_size=1)
    self.collision_pub = rospy.Publisher('collision', SpheroCollision, queue_size=1)
    self.diag_pub = rospy.Publisher('diagnostics', DiagnosticArray, queue_size=1)
    self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel, queue_size=1)
    self.color_sub = rospy.Subscriber('set_color', ColorRGBA, self.set_color, queue_size=1)
    self.back_led_sub = rospy.Subscriber('set_back_led', Float32, self.set_back_led, queue_size=1)
    self.stabilization_sub = rospy.Subscriber('disable_stabilization', Bool, self.set_stabilization, queue_size=1)
    self.heading_abs_sub = rospy.Subscriber('set_heading', Float32, self.set_heading, queue_size=1)
    self.heading_rad_sub = rospy.Subscriber('heading', Float32, self.heading_radians, queue_size=1)
    self.heading_deg_sub = rospy.Subscriber('heading_degrees', Float32, self.heading_degrees, queue_size=1)
    self.speed_sub = rospy.Subscriber('speed', Float32, self.speed, queue_size=1)
    self.cfg_locator_sub = rospy.Subscriber('configure_locator', Pose2D, self.configure_locator, queue_size=1)
#     self.current_speed_pub = rospy.Publisher('current_speed', Float32, queue_size=1)
#     self.reconfigure_srv = dynamic_reconfigure.server.Server(ReconfigConfig, self.reconfigure)
    self.transform_broadcaster = tf.TransformBroadcaster()

  def _init_params(self):
    self.rate = rospy.get_param('~rate', 50.0)
    self.sampling_divisor = int(math.ceil(400 / self.rate))

    self.connect_color_red = rospy.get_param('~connect_red', 0)
    self.connect_color_blue = rospy.get_param('~connect_blue', 0)
    self.connect_color_green = rospy.get_param('~connect_green', 0)
    self.cmd_vel_timeout = rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
    self.diag_update_rate = rospy.Duration(rospy.get_param('~diag_update_rate', 1.0))
    self.reset_odom = rospy.get_param('~reset_odom', True);
#     self.min_speed_diff = rospy.get_param('~min_speed_diff', 0.05)
#     self.speed_per_unit = rospy.get_param('~speed_per_unit', 0.0091)
#     self.kp = rospy.get_param('~proportional_gain', 1.0)
#     self.ki = rospy.get_param('~integral_gain', 0.0)
#     self.kd = rospy.get_param('~derivative_gain', 0.0)
#     self.antiw = rospy.get_param('~anti_windup', 1000.0)
#     self.cutoff_freq = rospy.get_param('~cutoff_frequency', 12.5)
#     self.cutoff_changed = True

  def normalize_angle_positive(self, angle):
    return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi);

  def start(self):
    try:
      self.is_connected = self.robot.connect()
      rospy.loginfo("Connect to Sphero with address: %s" % self.robot.bt.target_address)
    except:
      rospy.logerr("Failed to connect to Sphero.")
      sys.exit(1)
      
    # reset locator if necessary
    if self.reset_odom:
      self.robot.configure_locator(0, 0, 0)
    
    # setup streaming    
    self.robot.set_filtered_data_strm(self.sampling_divisor, 1 , 0, True)
    self.robot.add_async_callback(sphero_driver.IDCODE['DATA_STRM'], self.parse_data_strm)
    # setup power notification
    self.robot.set_power_notify(True, False)
    self.robot.add_async_callback(sphero_driver.IDCODE['PWR_NOTIFY'], self.parse_power_notify)
    # setup collision detection
    self.robot.config_collision_detect(1, 45, 110, 45, 110, 100, False)
    self.robot.add_async_callback(sphero_driver.IDCODE['COLLISION'], self.parse_collision)
    # set the ball to connection color
    self.robot.set_rgb_led(self.connect_color_red, self.connect_color_green, self.connect_color_blue, 0, False)
    # now start receiving packets
    self.robot.start()

  def spin(self):
    r = rospy.Rate(self.rate)
#     dt = 1.0/self.rate
#     error = [0, 0, 0]
#     filtered_error = [0, 0, 0]
#     integral = 0
#     derivative = [0, 0, 0]
#     filtered_derivative = [0, 0, 0]
#     c = 1.0
#     loops = 0
    
    while not rospy.is_shutdown():
      now = rospy.Time.now()
      # slower than 0.2 will not move
#       if self.target_speed < 0.2 or (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
      if (now - self.last_cmd_vel_time) > self.cmd_vel_timeout:
#         self.target_speed = 0
        self.cmd_speed = 0
      else:
        self.cmd_speed = self.robot.clamp(int(self.target_speed), 0, 255)
      
#       if self.target_speed != 0:
#         if self.cutoff_changed:
#           c = 1/math.tan((self.cutoff_freq * 6.2832) * dt / 2)
#           print "cutoff recalculated"
#           self.cutoff_changed = False
#         
#         #update error
#         error[2] = error[1]
#         error[1] = error[0]
#         error[0] = self.target_speed - self.current_speed
#         
#         #calculate integral and apply anti-windup
#         integral += error[0] * dt
#         integral = self.robot.clamp(integral, -abs(self.antiw), abs(self.antiw))
#         
#         #filter error
#         filtered_error[2] = filtered_error[1]
#         filtered_error[1] = filtered_error[0]
#         filtered_error[0] = (1/(1+c*c+1.414*c))*(error[2]+2*error[1]+error[0]-(2-1.414)*filtered_error[2])
#         
#         #calculate derivative
#         derivative[2] = derivative[1]
#         derivative[1] = derivative[0]
#         derivative[0] = (error[0] - error[1]) / dt
#         
#         #filter derivative
#         filtered_derivative[2] = filtered_derivative[1]
#         filtered_derivative[1] = filtered_derivative[0]
#         
#         if loops > 2:
#           filtered_derivative[0] = (1/(1+c*c+1.414*c))*(derivative[2]+2*derivative[1]+derivative[0]-(2-1.414)*filtered_derivative[2])
#         else:
#           ++loops
#         
#         self.cmd_speed = int(self.robot.clamp(self.kp * filtered_error[0] + self.ki * integral + self.kd * filtered_derivative[0], 0, 255))
#       else:
#         self.cmd_speed = 0
#         error = [0, 0, 0]
#         filtered_error = [0, 0, 0]
#         integral = 0
#         derivative = [0, 0, 0]
#         filtered_derivative = [0, 0, 0]
      
      self.robot.roll(self.cmd_speed, self.cmd_heading, self.robot.clamp(self.cmd_speed, 0, 1))
      
#       self.current_speed_pub.publish(self.current_speed)
      
      if (now - self.last_diagnostics_time) > self.diag_update_rate:
        self.last_diagnostics_time = now
        self.publish_diagnostics(now)
      r.sleep()
                  
  def stop(self):    
    # tell the ball to stop moving before quiting
    self.robot.roll(0, self.cmd_heading, 0, False)
    self.robot.shutdown = True
    rospy.sleep(1.0)
    self.is_connected = self.robot.disconnect()
    self.robot.join()

  def publish_diagnostics(self, time):
    diag = DiagnosticArray()
    diag.header.stamp = time
    
    stat = DiagnosticStatus(name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
    if self.power_state == 3:
      stat.level = DiagnosticStatus.WARN
    if self.power_state == 4:
      stat.level = DiagnosticStatus.ERROR
    diag.status.append(stat)

    self.diag_pub.publish(diag)


  def parse_collision(self, data):
    if self.is_connected:
      now = rospy.Time.now()
      collision = SpheroCollision()
      collision.header.stamp = now
      collision.x = data["X"]
      collision.y = data["Y"]
      collision.z = data["Z"]
      collision.axis = int(data["Axis"])
      collision.x_magnitude = data["xMagnitude"]
      collision.y_magnitude = data["yMagnitude"]
      collision.speed = data["Speed"]
      collision.timestamp = data["Timestamp"]
      
      self.collision = collision
      self.collision_pub.publish(self.collision)
          

  def parse_power_notify(self, data):
    if self.is_connected:
      self.power_state = data
      self.power_state_msg = self.battery_state[data]

  def parse_data_strm(self, data):
    if self.is_connected:
      now = rospy.Time.now()

      quat = [ data["QUATERNION_Q1"] / 10000.0, #x
               data["QUATERNION_Q2"] / 10000.0, #y
               data["QUATERNION_Q3"] / 10000.0, #z
               data["QUATERNION_Q0"] / 10000.0 ]#w

      imu = Imu(header=rospy.Header(frame_id="imu_link"))
      imu.header.stamp = now
      imu.orientation.x = quat[0]
      imu.orientation.y = quat[1]
      imu.orientation.z = quat[2]
      imu.orientation.w = quat[3]
      imu.linear_acceleration.x = data["ACCEL_X_FILTERED"] / 4096.0 * 9.8
      imu.linear_acceleration.y = data["ACCEL_Y_FILTERED"] / 4096.0 * 9.8
      imu.linear_acceleration.z = data["ACCEL_Z_FILTERED"] / 4096.0 * 9.8
      imu.angular_velocity.x = data["GYRO_X_FILTERED"] * 10.0 * math.pi / 180.0
      imu.angular_velocity.y = data["GYRO_Y_FILTERED"] * 10.0 * math.pi / 180.0
      imu.angular_velocity.z = data["GYRO_Z_FILTERED"] * 10.0 * math.pi / 180.0

      self.imu = imu
      self.imu_pub.publish(self.imu)

      pos = [ data["ODOM_X"] / 100.0,
              data["ODOM_Y"] / 100.0,
              0.0 ]

      odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_footprint')
      odom.header.stamp = now
      odom.pose.pose = Pose(Point(pos[0], pos[1], pos[2]), Quaternion(0.0, 0.0, 0.0, 1.0))
      odom.twist.twist = Twist(Vector3(data["VELOCITY_X"] / 1000.0, data["VELOCITY_Y"] / 1000.0, 0), Vector3(0, 0, imu.angular_velocity.z))
      odom.pose.covariance = self.ODOM_POSE_COVARIANCE
      odom.twist.covariance = self.ODOM_TWIST_COVARIANCE
      self.odom_pub.publish(odom)

      # need to publish this trasform to show the roll, pitch, and yaw properly
      self.transform_broadcaster.sendTransform((0.0, 0.0, 0.0381), quat, odom.header.stamp, "base_link", "base_footprint")
      self.transform_broadcaster.sendTransform(pos, (0.0, 0.0, 0.0, 1.0), odom.header.stamp, "base_footprint", "odom")
#       self.current_speed = math.sqrt(math.pow(odom.twist.twist.linear.x, 2) + math.pow(odom.twist.twist.linear.y, 2))

  def cmd_vel(self, msg):
    if self.is_connected:
      self.last_cmd_vel_time = rospy.Time.now()
      self.cmd_heading = int(self.normalize_angle_positive(math.atan2(msg.linear.x, msg.linear.y)) * 180 / math.pi)
      self.target_speed = math.sqrt(math.pow(msg.linear.x, 2) + math.pow(msg.linear.y, 2))
      #self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), 1, False)
      
  def heading_degrees(self, msg):
    self.heading(msg.data * math.pi / 180.0)
      
  def heading_radians(self, msg):
    self.heading(msg.data)
      
  def heading(self, rad):
    if self.is_connected:
      self.cmd_heading = int(self.normalize_angle_positive(rad) * 180 / math.pi)
      #self.robot.roll(self.cmd_speed, int(self.cmd_heading), 1, False)
      
  def speed(self, msg):
    if self.is_connected:
      self.last_cmd_vel_time = rospy.Time.now()
      self.target_speed = msg.data
      #self.robot.roll(self.cmd_speed, int(self.cmd_heading), 1, False)
      
  def set_color(self, msg):
    if self.is_connected:
      self.robot.set_rgb_led(int(msg.r * 255), int(msg.g * 255), int(msg.b * 255), 0, False)

  def set_back_led(self, msg):
      if self.is_connected:
          self.robot.set_back_led(int(msg.data * 255), False)

  def set_stabilization(self, msg):
    if self.is_connected:
      if not msg.data:
        self.robot.set_stabilization(1, False)
      else:
        self.robot.set_stabilization(0, False)

  def set_heading(self, msg):
    if self.is_connected:
      heading_deg = int(self.normalize_angle_positive(msg.data) * 180.0 / math.pi)
      self.robot.set_heading(heading_deg, False)

  def configure_collision_detect(self, msg):
    pass

#   def reconfigure(self, config, level):
#     self.min_speed_diff = config['min_speed_diff']
#     self.speed_per_unit = config['speed_per_unit']
#     self.kp = config['proportional_gain']
#     self.ki = config['integral_gain']
#     self.kd = config['derivative_gain']
#     self.antiw = config['anti_windup']
#     if level == 1:
#       self.cutoff_freq = config['cutoff_frequency']
#       self.cutoff_changed = True
#       
#     return config
      
  def configure_locator(self, msg):
    if self.is_connected:
      self.robot.configure_locator(int(msg.x * 100), int(msg.y * 100), int(self.normalize_angle_positive(msg.theta) * 180.0 / math.pi))     
        
if __name__ == '__main__':
  s = SpheroNode()
  s.start()
  s.spin()
  s.stop()
