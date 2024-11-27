#!/usr/bin/env python3.8

"""
  @author Simone Cottiga
  @email simone.cottiga@phd.units.it
  @email smcotti@gmail.com
"""

import rospy
import numpy as np
import joblib
import os
from scipy.spatial.transform import Rotation as Rot
from robot4ws_msgs.msg import Vector3Array
from robot4ws_msgs.msg import JointState1
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Vector3
from robot4ws_msgs.msg import neural_network_in_out
from std_msgs.msg import Float64
import warnings
from custom_slip_function import slipFunction

# ignore warning for alpha = 0°
warnings.filterwarnings("ignore",message="Gimbal lock detected. Setting third angle to zero since it is not possible to uniquely determine all angles.")

class NeuralNetworkNode:
    def __init__(self) -> None:
        self.joint_state_topic_name = 'Archimede/joint_states'
        self.link_state_topic_name = 'gazebo/link_states'
        self.publish_topic_name = 'terrain_neural_network_out'
        self.node_name = 'terrain_neural_network'
        self.joint_state_subscriber = None
        self.link_state_subscriber = None
        self.publisher = None
        self.last_pose = None
        self.wheel_radius = 0.085

        # steer link in [BR, FR, BL, FL] order, as they are named in the link_states msg
        self.steer_link_names = ["Archimede::Archimede_br_steer_link","Archimede::Archimede_fr_steer_link","Archimede::Archimede_bl_steer_link","Archimede::Archimede_fl_steer_link"]
        # drive joint in [BR, FR, BL, FL] order, as they are named in the joint_states msg
        self.drive_joint_names = ["Archimede_br_drive_joint","Archimede_fr_drive_joint","Archimede_bl_drive_joint","Archimede_fl_drive_joint"]
        self.is_1st_joint_msg = True
        self.is_1st_link_msg = True
        self.link_input_order = [] # steers from the link_states message to [BR, FR, BL, FL] order, will be filled at 1st callback
        self.joint_input_order = [] # drives from the joint_states message to [BR, FR, BL, FL] order, will be filled at 1st callback
        # self.link_input_order = [-6,-2,-8,-4] # steers from the /gazebo/link_states message to [BR, FR, BL, FL] order
        # self.joint_input_order = [2,6,0,4] # drives from the /Archimede/joint_states message to [BR, FR, BL, FL] order

        self.last_wheels_orientations = None
        self.pi_rot = Rot.from_euler('Z', np.pi, degrees=False) # 180 deg rotation around Z axis

        self.NN_model_name = 'best_rf_sw'

        # add new MLmodels here, in the _joint_state_callback ifs conditions and at the end on initialize if needed
        self.valid_NN_model_names = ['best_bagging_all_var_OLD',
                                     'best_bagging_angle_mod_OLD',
                                     'best_bagging_all_wheels',
                                     'best_bagging_single_wheel',
                                     'best_rf_all_var',
                                     'best_rf_sw',
                                     'best_rf_simple_slip',
                                     'best_rf_sin_cos',
                                     'none',
                                     'use_simple_slip_function']

        self.validate_plugin = True # if true, publish also all inputs and outputs of NN in a specific topic


    def initialize(self) -> None:
        rospy.loginfo('Starting artificial slip Neural Network node')
        self.node = rospy.init_node(self.node_name, anonymous=False)
        rospy.set_param("neural_network_output_topic", self.publish_topic_name)

        #Initialize subscribers
        self.joint_state_subscriber = rospy.Subscriber(self.joint_state_topic_name, JointState, self._joint_state_callback,queue_size=1)
        self.link_state_subscriber = rospy.Subscriber(self.link_state_topic_name, LinkStates, self._link_state_callback,queue_size=1)

        # Initialize publisher
        self.publisher = rospy.Publisher(self.publish_topic_name, Vector3Array, queue_size=100)
        wheels = ["BR_wheel", "FR_wheel", "BL_wheel", "FL_wheel"]
        self.output_msg = Vector3Array(names=wheels,vectors=[Vector3(),Vector3(),Vector3(),Vector3()])

        # get/set MLmodel param
        if rospy.has_param("neural_network_model") :
            self.NN_model_name = rospy.get_param("neural_network_model")
        else :
            rospy.set_param("neural_network_model", self.NN_model_name)

        # initialize validation plugin publisher
        if self.validate_plugin:
            self.valid_plug_pub = rospy.Publisher("plugin_validation_NN", neural_network_in_out, queue_size=100)
            self.valid_plug_msg = neural_network_in_out()
            self.valid_plug_msg.header.frame_id = '[BR, FR, BL, FL]'

        # Initialize prediction models
        path_this = os.path.dirname(__file__)
        if self.NN_model_name in self.valid_NN_model_names and not self.NN_model_name in ['best_bagging_angle_mod_OLD','none','use_simple_slip_function']:
            self.NN_model = joblib.load(os.path.join(path_this,'../Modelli_DT_robot',self.NN_model_name + '.pkl'))
        elif self.NN_model_name == 'best_bagging_angle_mod_OLD':
            self.angle_model = joblib.load(path_this,'../Modelli_DT_robot/best_bagging_angle_OLD.pkl')
            self.mod_model = joblib.load(path_this,'../Modelli_DT_robot/best_bagging_mod_OLD.pkl')
        elif not self.NN_model_name in ['none','use_simple_slip_function']:
                rospy.loginfo(f'Unrecognized ML model [{self.NN_model_name}], shutting down [{self.node_name}]...')
                rospy.signal_shutdown('Neural Network model not recognized! Shutting down artificial slip Neural Network node')

        rospy.loginfo(f'Artificial slip Neural Network node initialized with [{self.NN_model_name}] model')


    def _joint_state_callback(self, msg_in):
        # if 1st msg received, fill the joints order
        if self.is_1st_joint_msg:
            for joint in self.drive_joint_names:
                self.joint_input_order.append(msg_in.name.index(joint))
            self.is_1st_joint_msg = False

        # gimbal lock warning that raises for alpha = 0° is ignored. Angles are still correct
        euler = self.last_wheels_orientations.as_euler('ZYZ', degrees=True)

        map2ground_rots_arr = np.empty(4, dtype=Rot)
        all_input_array = np.empty(12)
        outputs_array = np.empty(8)

        for n in range(4):
            # import message data
            drive_velocity = msg_in.velocity[self.joint_input_order[n]]
            prev_drive_vel = drive_velocity # delete, just for checking
            if n==0 or n==1:
                drive_velocity = -drive_velocity
            cmd_vel = drive_velocity * self.wheel_radius

            # take the alpha and beta angles [deg]
            alpha = euler[n][1]
            beta = euler[n][2]
            map2ground_rot = Rot.from_euler('ZY',[euler[n][0],alpha],degrees=True)
            if alpha > 0:
                alpha = -alpha
                beta = beta + 180
                map2ground_rot = Rot.from_euler('ZY',[euler[n][0]+180,alpha],degrees=True)
                if beta >= 180:
                    beta = beta - 360

            # correct negative commanded velocities so that the input vels for the NN are positive 
            if cmd_vel < 0:
                cmd_vel = -cmd_vel
                if beta < 0:
                    beta = beta + 180
                else:
                    beta = beta - 180

            # store values in array
            map2ground_rots_arr[n] = map2ground_rot
            all_input_array[n*3] = alpha
            all_input_array[n*3+1] = beta
            all_input_array[n*3+2] = cmd_vel

            if self.NN_model_name in ['best_bagging_all_var_OLD','best_bagging_all_wheels','best_rf_all_var','use_simple_slip_function']:
                continue

            elif self.NN_model_name == 'best_bagging_angle_mod_OLD':
                # run the neural network
                # in: [alpha, cmd_X_ground, cmd_Y_ground]  or        [alpha, beta, cmd_vel]        (vel in ground frame)
                # out:   [real_X_ground, real_Y_ground]    or  [real_dir_ground, real_mod_ground]  (vel in ground frame)
                input_array = np.array([alpha, beta, cmd_vel])
                input_array = input_array[np.newaxis, :]

                real_dir_ground = self.angle_model.predict(input_array)[0]
                real_mod_ground = self.mod_model.predict(input_array)[0]

            elif self.NN_model_name in ['best_bagging_single_wheel','best_rf_sw','best_rf_simple_slip']:
                input_array = np.array([alpha, beta, cmd_vel])
                input_array = input_array[np.newaxis, :]

                [real_dir_ground, real_mod_ground] = self.NN_model.predict(input_array)[0]

            elif self.NN_model_name in ['best_rf_sin_cos']:
                input_array = np.array([np.sin(np.deg2rad(alpha)), np.cos(np.deg2rad(alpha)), np.sin(np.deg2rad(beta)), np.cos(np.deg2rad(beta)), cmd_vel])
                input_array = input_array[np.newaxis, :]

                [s_real_dir_ground, c_real_dir_ground, real_mod_ground] = self.NN_model.predict(input_array)[0]
                real_dir_ground = np.rad2deg(np.arctan2(s_real_dir_ground,c_real_dir_ground))

            elif self.NN_model_name == 'none':
                real_dir_ground = beta
                real_mod_ground = cmd_vel

            # store results [angle mod] in [BR FR BL FL] wheels order
            outputs_array[n*2] = real_dir_ground
            outputs_array[n*2+1] = real_mod_ground

        all_input_array = all_input_array[np.newaxis, :]

        if self.NN_model_name in ['best_bagging_all_var_OLD','best_bagging_all_wheels','best_rf_all_var']:
            # inputs in [BR FR BL FL] order
            outputs_array = self.NN_model.predict(all_input_array)[0]
        elif self.NN_model_name in ['use_simple_slip_function']:
            outputs_array = slipFunction(all_input_array)

        for n in range(4):
            real_X_ground = outputs_array[n*2+1] * np.cos(np.pi/180*outputs_array[n*2])
            real_Y_ground = outputs_array[n*2+1] * np.sin(np.pi/180*outputs_array[n*2])
            real_vel_map = map2ground_rots_arr[n].apply([real_X_ground,real_Y_ground,0]) 

            self.output_msg.vectors[n].x = real_vel_map[0]
            self.output_msg.vectors[n].y = real_vel_map[1]
            self.output_msg.vectors[n].z = real_vel_map[2]

        self.output_msg.header.stamp = rospy.Time.now()
        # publish the output
        self.publisher.publish(self.output_msg)

        if self.validate_plugin:
            self.valid_plug_msg.header.stamp = rospy.Time.now()
            for i,inp in enumerate(all_input_array[0]):
                self.valid_plug_msg.NN_inputs[i] = Float64(inp)
            for i,out in enumerate(outputs_array):
                self.valid_plug_msg.NN_outputs[i] = Float64(out)
            self.valid_plug_pub.publish(self.valid_plug_msg)


    def _link_state_callback(self, msg):
        # if 1st msg received, fill the links order
        if self.is_1st_link_msg:
            for link in self.steer_link_names:
                self.link_input_order.append(msg.name.index(link))
            self.is_1st_link_msg = False

        # import message data
        BR_x = msg.pose[self.link_input_order[0]].orientation.x
        BR_y = msg.pose[self.link_input_order[0]].orientation.y
        BR_z = msg.pose[self.link_input_order[0]].orientation.z
        BR_w = msg.pose[self.link_input_order[0]].orientation.w

        FR_x = msg.pose[self.link_input_order[1]].orientation.x
        FR_y = msg.pose[self.link_input_order[1]].orientation.y
        FR_z = msg.pose[self.link_input_order[1]].orientation.z
        FR_w = msg.pose[self.link_input_order[1]].orientation.w

        BL_x = msg.pose[self.link_input_order[2]].orientation.x
        BL_y = msg.pose[self.link_input_order[2]].orientation.y
        BL_z = msg.pose[self.link_input_order[2]].orientation.z
        BL_w = msg.pose[self.link_input_order[2]].orientation.w

        FL_x = msg.pose[self.link_input_order[3]].orientation.x
        FL_y = msg.pose[self.link_input_order[3]].orientation.y
        FL_z = msg.pose[self.link_input_order[3]].orientation.z
        FL_w = msg.pose[self.link_input_order[3]].orientation.w

        self.last_wheels_orientations = Rot.from_quat([[BR_x, BR_y, BR_z, BR_w],[FR_x, FR_y, FR_z, FR_w],[BL_x, BL_y, BL_z, BL_w],[FL_x, FL_y, FL_z, FL_w]])

        # post-rotate BR and FR frames by 180° around Z-axis. NN was trained with all wheels frames allined with base frame (for zero steering),
        # while in simulation all Y axis point outside the rover (BR and FR are inverted)
        # BR and FR commanded velocities will be inverted too
        self.last_wheels_orientations[0] = self.last_wheels_orientations[0]*self.pi_rot
        self.last_wheels_orientations[1] = self.last_wheels_orientations[1]*self.pi_rot



def main():
    node = NeuralNetworkNode()
    node.initialize()
    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down artificial slip Neural Network node')
