import rospy 
import time
from sensor_msgs.msg import Joy
import numpy as np
from std_msgs.msg import Float32MultiArray
from utils import get_quaternion_from_euler
class Bimanual_Feedback():
    def __init__(self):
        rospy.Subscriber("/spacenav_left/joy", Joy, self.teleop_callback_left, queue_size=1)
        rospy.Subscriber("/spacenav_left/joy", Joy, self.btns_callback_left, queue_size=1)
        rospy.Subscriber("/spacenav_right/joy", Joy, self.teleop_callback_right, queue_size=1)
        rospy.Subscriber("/spacenav_right/joy", Joy, self.btns_callback_right, queue_size=1)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

    def joy_callback(self, data):
        """"
        Callback function for the joystick, written for the Logitech F710
        """
        # B
        if data.buttons[2] == 1:
            self.end = True
            self.Panda_left.end = True
            self.Panda_right.end = True
        # LB
        if data.buttons[4] == 1:
            self.Panda_right.pause = not self.Panda_right.pause
            print("Panda right toggled, state: {self.Panda_left.pause}")

        # RB
        if data.buttons[5] == 1:
            self.Panda_left.pause = not self.Panda_left.pause
            print("Panda left pause toggled, state: {self.Panda_left.pause}")

        # Dpad up/down
        if data.buttons[6] == 1:
            self.execution_factor += 0.2
            self.client.update_configuration({"execution_factor": self.execution_factor})
        if data.buttons[7] == 1:
            self.execution_factor -= 0.2
            self.client.update_configuration({"execution_factor": self.execution_factor})


    def teleop_callback_left(self, data):
        """
        Spacemouse callback for the left Panda
        :param data: callback data
        """
        # self.Panda_left.feedback =[data.x, data.y, data.z]

        self.Panda_left.offset[0]= 0.002*data.axes[0]
        self.Panda_left.offset[1]= 0.002*data.axes[1]
        self.Panda_left.offset[2]= 0.002*data.axes[2]
        self.Panda_left.offset[3]= 0.02*data.axes[3]
        self.Panda_left.offset[4]= 0.02*data.axes[4] 
        self.Panda_left.offset[5]= 0.02*data.axes[5]
        if self.Panda_left.enable_corr and np.sum(self.Panda_left.feedback) != 0:
            if self.Panda_left.correction_mode == 0:
                self.Panda_left.execution_traj, self.Panda_left.execution_ori = self.Panda_left.correct_attractor(self.Panda_left.index, self.Panda_left.execution_traj, self.Panda_left.execution_ori, self.Panda_left.feedback_factor_pos) #index, traj_pos, traj_ori, magnitude
                time.sleep(0.01)
            # else:
            #     self.Panda_left.execution_stiff_lin = self.Panda_left.correct_stiffness(self.Panda_left.index, self.Panda_left.execution_traj, self.Panda_left.execution_stiff_lin, self.Panda_left.feedback_factor_stiff_lin)
            #     time.sleep(0.01)

    def btns_callback_left(self, data):
        """
        Spacemouse button callback for the left Panda
        :param data: callback data
        """
        self.Panda_left.btn = [data.buttons[0], data.buttons[1]]

    def teleop_callback_right(self, data):
        """
        Spacemouse callback for the right Panda
        :param data: callback data
        """

        # self.Panda_right.feedback = [data.x, data.y, data.z]

        self.Panda_right.offset[0]= 0.002*data.axes[0]
        self.Panda_right.offset[1]= 0.002*data.axes[1]
        self.Panda_right.offset[2]= 0.002*data.axes[2]
        self.Panda_right.offset[3]= 0.02*data.axes[3]
        self.Panda_right.offset[4]= 0.02*data.axes[4] 
        self.Panda_right.offset[5]= 0.02*data.axes[5]

        if self.Panda_right.enable_corr and np.sum(self.Panda_right.feedback) != 0:
            if self.Panda_right.correction_mode == 0:
                self.Panda_right.execution_traj, self.Panda_right.execution_ori  = self.Panda_right.correct_attractor(self.Panda_right.index, self.Panda_right.execution_traj, self.Panda_right.execution_ori, self.Panda_right.feedback_factor_pos) #index, traj_pos, traj_ori, magnitude
                time.sleep(0.01)
            # else:
            #     self.Panda_right.execution_stiff_lin = self.Panda_right.correct_stiffness(self.Panda_right.index, self.Panda_right.execution_traj, self.Panda_right.execution_stiff_lin, self.Panda_right.feedback_factor_stiff_lin)
            #     time.sleep(0.01)

        # spacemouse buttons subscriber
    def btns_callback_right(self, data):
        """
        Spacemouse button callback for the left Panda
        :param data: callback data
        """
        self.Panda_right.btn = [data.buttons[0], data.buttons[1]]

class Feedback():
    def __init__(self) -> None:
        pass    
    
    def teleoperate(self, attractor_pos, attractor_ori):
        traj_error = np.linalg.norm(self.execution_traj.T - np.array(self.cart_pos), axis=1)
        beta=1
        traj_errror_total=traj_error

        i = np.min([int(np.argmin(traj_errror_total))+4, self.execution_traj.shape[1]-1]) #this 4 can be any other number, ideally it should be 1 but if you put one the robot is not going to move
        self.index=np.min([int(np.argmin(traj_errror_total))+1, self.execution_traj.shape[1]-1])
        attractor_pos_new = [attractor_pos[0] + self.offset[0], attractor_pos[1]+ self.offset[1], attractor_pos[2]+ self.offset[2]]
        attractor_quat =np.quaternion( attractor_ori[0], attractor_ori[1], attractor_ori[2], attractor_ori[3])  

        q_delta_array=get_quaternion_from_euler(self.offset[3], self.offset[4], self.offset[5])
        q_delta=np.quaternion(q_delta_array[0],q_delta_array[1],q_delta_array[2],q_delta_array[3]) 
        attractor_quat_new=q_delta*attractor_quat

        attractor_ori_new = [attractor_quat_new.w, attractor_quat_new.x, attractor_quat_new.y, attractor_quat_new.z]
        stiff_msg = Float32MultiArray()
        stiff_msg.data =beta * np.array([self.execution_stiff_lin[0][i], self.execution_stiff_lin[1][i], self.execution_stiff_lin[2][i], self.execution_stiff_ori[0][i],self.execution_stiff_ori[1][i],self.execution_stiff_ori[2][i], 0.0]).astype(np.float32)

        return  i, attractor_pos_new, attractor_ori_new, stiff_msg, beta