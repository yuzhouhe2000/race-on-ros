#!/usr/bin/env python
# license removed for brevity

import rospy
from geometry_msgs.msg import Pose
from raceon.msg import AckermannDrive

SERVO_MIN = -900
SERVO_MIDDLE = 0
SERVO_MAX = 900


class Controller():
    
    def __init__(self):
        self.topic_name_pos_err = rospy.get_param("topic_name_position_error", "position/error")
        self.topic_name_control = rospy.get_param("topic_name_control", "control")
        
        # Parameters for control
        self.motor_speed = rospy.get_param("~motor_speed", 200)
        self.target = rospy.get_param("~target", 0)
        self.kp = rospy.get_param("~kp", 10)
        
        # Yuzhou Added
        self.Ki = rospy.get_param('~Ki',10)
        self.Kd = rospy.get_param('~Kd',0.001)
        self.prev_pid_time = rospy.Time.now()
        self.integral = 0
        self.derivative = 0
        self.previous_error = 0
        self.state = 0
        


        
    def start(self):
        self.sub_pos_err = rospy.Subscriber(self.topic_name_pos_err, Pose, self.pos_err_callback)
        self.pub_control = rospy.Publisher(self.topic_name_control, AckermannDrive, queue_size=10)
        rospy.spin()

    def pos_err_callback(self, pos_err_msg):
        pos_err = self.target - pos_err_msg.position.x
        
        rospy.loginfo("Current error: pos_err = " + str(pos_err))
        
        servo_pos = self.control_servo(pos_err)
        motor_speed = self.motor_speed
        
        rospy.loginfo("Control command: servo_pos = " + str(servo_pos) + ", motor_speed = " + str(motor_speed))
        
        control_msg = AckermannDrive()
        control_msg.speed = motor_speed
        control_msg.steering_angle = servo_pos
        self.pub_control.publish(control_msg)
        
    # TODO: Implement PID
    def pid(self, error):
        if (abs(error) >= 100) and (self.state != 2):
            self.Kp = 8000
            self.Kd = 300
            self.Ki = 250
            self.state = 2
            motor.duty_cycle = MOTOR_BRAKE + 270000
        
        if (abs(error) >= 30) and (self.state != 1):
            self.Kp = 7200
            self.Kd = 180
            self.Ki = 90
            self.state = 1
            motor.duty_cycle = MOTOR_BRAKE + 270000
            self.integral = 0

        if (abs(error) < 30) and (self.state != 0):
            self.Kp = 7500
            self.Kd = 200
            self.Ki = 120
            self.state = 0
            motor.duty_cycle = MOTOR_BRAKE + 320000
            self.integral = 0
        
        pid_dt_duration = rospy.Time.now() - self.prev_pid_time
        self.pid_dt = pid_dt_duration.to_sec()
        self.prev_pid_time = rospy.Time.now()
        self.derivative = (error - self.previous_error) / self.pid_dt
        self.integral = self.integral + (error * self.pid_dt)
        self.previous_error = error
        self.Kpid = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
     
        return self.Kpid
        
        
        

        

    def control_servo(self, error):
        correction = self.pid(error)
        servo_pos = SERVO_MIDDLE + correction

        if servo_pos > SERVO_MAX:
            servo_pos = SERVO_MAX
            
            # Yuzhou Added
            self.integral = self.integral - (error * self.pid_dt)
            
        if servo_pos < SERVO_MIN:
            servo_pos = SERVO_MIN
            
            #Yuzhou Added
            self.integral = self.integral - (error * self.pid_dt)

        return servo_pos

if __name__ == "__main__":
    rospy.init_node("control")
    controller = Controller()
    try:
        controller.start()
    except rospy.ROSInterruptException:
        pass
