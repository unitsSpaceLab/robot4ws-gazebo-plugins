#!/usr/bin/env python3

import rospy
import numpy as np
import joblib
import os
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Header, Int32MultiArray
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from robot4ws_msgs.msg import Vector3Array
import warnings
import pandas as pd

warnings.filterwarnings("ignore")

class MLVelocityPredictor:
    def __init__(self):
        self.pub_raw_model_outputs = True # ground frame

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_pathvxg = os.path.join(script_dir, '../Modelli_DT_robot', 'model_artifactsX')
        self.model_pathvyg = os.path.join(script_dir, '../Modelli_DT_robot', 'model_artifactsY')
        self.wheel_radius = 0.085

        # Joint and link names in [BR, FR, BL, FL] order
        self.steer_link_names = [
            "Archimede::Archimede_br_steer_link",
            "Archimede::Archimede_fr_steer_link", 
            "Archimede::Archimede_bl_steer_link",
            "Archimede::Archimede_fl_steer_link"
        ]
        self.steer_joint_names = [
            "Archimede_br_steer_joint",
            "Archimede_fr_steer_joint",
            "Archimede_bl_steer_joint",
            "Archimede_fl_steer_joint"
        ]
        self.drive_joint_names = ["Archimede_br_drive_joint","Archimede_fr_drive_joint","Archimede_bl_drive_joint","Archimede_fl_drive_joint"]

        self.base_link_name = "Archimede::Archimede_base_link"

        # Data storage
        self.last_wheels_orientations = None
        self.last_base_orientation = None
        self.terrain_colors = [0, 0, 0, 0]

        self.steer_input_order = []
        self.joint_input_order = []
        self.link_input_order = []
        self.base_indx = []
        self.is_1st_joint_msg = True
        self.is_1st_link_msg = True

        # Data for wheel load computation
        self.gravity = 9.81
        self.wheel_masses = np.array([2.0, 2.0, 2.0, 2.0])
        self.other_link_masses = np.array([1.07, 0, 0])  # exclude legs
        self.wheel_signes = np.array([[-1, -1],[1, -1],[-1, 1],[1, 1]])
        self.wheel_height_in_model = np.array([-0.07368, -0.07368, -0.07368, -0.07368])
        self.rover_wheelbase = np.array([0.36574, 0.22150]) * 2
        self.other_link_pos_in_model = np.array([[0, 0, 0],[0, 0.133, -0.05],[0, -0.133, -0.05]])

        # 180 degree rotation for BR/FR wheels
        self.pi_rot = Rot.from_euler('Z', np.pi, degrees=False)

        # Load models
        self.load_models()

    def load_models(self):
        try:
            vxg_model_path = os.path.join(self.model_pathvxg, 'model_vxg_ground_truth.pkl')
            vxg_scaler_path = os.path.join(self.model_pathvxg, 'scaler_vxg_ground_truth.pkl')
            vyg_model_path = os.path.join(self.model_pathvyg, 'model_vyg_ground_truth.pkl')
            vyg_scaler_path = os.path.join(self.model_pathvyg, 'scaler_vyg_ground_truth.pkl')

            self.vxg_model = joblib.load(vxg_model_path)
            self.vxg_scaler = joblib.load(vxg_scaler_path)
            self.vyg_model = joblib.load(vyg_model_path)
            self.vyg_scaler = joblib.load(vyg_scaler_path)

            rospy.loginfo(f"Loaded vxg and vyg models from {self.model_pathvxg}, {self.model_pathvyg}")

        except Exception as e:
            rospy.logerr(f"Failed to load models: {e}")
            raise

    def initialize(self):
        rospy.init_node('ml_velocity_predictor', anonymous=False)

        # Subscribers
        self.joint_state_sub = rospy.Subscriber(
            'Archimede/joint_states', JointState, self.joint_state_callback, queue_size=1
        )
        self.link_state_sub = rospy.Subscriber(
            'gazebo/link_states', LinkStates, self.link_state_callback, queue_size=1
        )
        self.terrain_color_sub = rospy.Subscriber(
            'wheel_terrain_ids', Int32MultiArray, self.terrain_color_callback, queue_size=1
        )

        # Publisher
        self.velocity_pub = rospy.Publisher(
            'ml_velocity_predictions', Vector3Array, queue_size=10
        )
        if self.pub_raw_model_outputs: # use same output_msg ad velocity_pub
            self.raw_outs_pub = rospy.Publisher(
                'ml_raw_predictions', Vector3Array, queue_size=10
            )

        # Initialize output message
        wheel_names = ["BR_wheel", "FR_wheel", "BL_wheel", "FL_wheel"]
        self.output_msg = Vector3Array(
            names=wheel_names,
            vectors=[Vector3(), Vector3(), Vector3(), Vector3()]
        )

        # Wheels to other links positions
        wheels_pos_in_model = self.wheel_signes * self.rover_wheelbase
        wheels_pos_in_model = np.hstack((wheels_pos_in_model,self.wheel_height_in_model.reshape(-1,1)))
        self.wheels_to_other_links = np.zeros((len(self.other_link_masses),3,4))
        for ii in range(4):
            self.wheels_to_other_links[:,:,ii] = np.abs(self.other_link_pos_in_model - wheels_pos_in_model[ii,:])

        rospy.loginfo("ML Velocity Predictor initialized")

    def terrain_color_callback(self, msg):
        if len(msg.data) >= 4:
            self.terrain_colors = msg.data[:4]

    def link_state_callback(self, msg):
        if self.is_1st_link_msg:
            self.base_indx = msg.name.index(self.base_link_name)
            for link in self.steer_link_names:
                self.link_input_order.append(msg.name.index(link))
            self.is_1st_link_msg = False

        # Import message data
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

        # Post-rotate BR and FR frames by 180° around Z-axis
        self.last_wheels_orientations[0] = self.last_wheels_orientations[0]*self.pi_rot
        self.last_wheels_orientations[1] = self.last_wheels_orientations[1]*self.pi_rot

        self.last_base_orientation = Rot.from_quat([msg.pose[self.base_indx].orientation.x,
                                                    msg.pose[self.base_indx].orientation.y,
                                                    msg.pose[self.base_indx].orientation.z,
                                                    msg.pose[self.base_indx].orientation.w])

    def joint_state_callback(self, msg):
        if self.last_wheels_orientations is None:
            return

        if self.is_1st_joint_msg:
            for joint in self.drive_joint_names:
                self.joint_input_order.append(msg.name.index(joint))
            for steer in self.steer_joint_names:
                self.steer_input_order.append(msg.name.index(steer))    
            self.is_1st_joint_msg = False

        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            euler_angles = self.last_wheels_orientations.as_euler('ZYZ', degrees=True)

        predictions = []
        raw_predictions = []

        for wheel_idx in range(4):
            result = self.extract_wheel_features(msg, wheel_idx, euler_angles[wheel_idx])

            if result is not None:
                features, map2ground_rot = result
                vxg_pred, vyg_pred = self.predict_velocity(features)


                # Transform from ground frame to map frame
                real_vel_map = map2ground_rot.apply([vxg_pred, vyg_pred, 0])                
                predictions.append([real_vel_map[0], real_vel_map[1], real_vel_map[2]])  # Publish map frame!

                if self.pub_raw_model_outputs:
                    raw_predictions.append([vxg_pred, vyg_pred, 0])
            else:
                predictions.append((0.0, 0.0, 0.0))

        self.publish_predictions(predictions, False)

        if self.pub_raw_model_outputs:
            self.publish_predictions(raw_predictions, True)

    def extract_wheel_features(self, joint_msg, wheel_idx, euler):
        try:                
            drive_velocity = joint_msg.velocity[self.joint_input_order[wheel_idx]]

            # Invert velocity for BR and FR wheels (indices 0 and 1)
            if wheel_idx in [0, 1]:
                drive_velocity = -drive_velocity

            commanded_velocity = drive_velocity * self.wheel_radius

            # Get steer angle
            steer_angle = joint_msg.position[self.steer_input_order[wheel_idx]]

            # Extract slope (alpha) and approach angle (beta)
            alpha = euler[1]  # Y rotation (slope)
            beta = euler[2]   # Z rotation (approach angle)
            map2ground_rot = Rot.from_euler('ZY', [euler[0]+180, -alpha], degrees=True)

            beta = euler[2] - 180    # Center around 0
            # Ensure proper [-180, 180] wrapping
            while beta > 180:
                beta -= 360
            while beta <= -180:
                beta += 360
            beta = beta * np.pi / 180  # Convert to radians

            # Get terrain color
            color = self.terrain_colors[wheel_idx]

            # Get wheel load
            wheel_load = self.compute_wheel_loads(wheel_idx)

            # Base features
            features = {
                'slope': alpha,
                'color': color, 
                'approach_angles': beta,
                'commanded_velocities': commanded_velocity,
                'steer_angle': steer_angle,
                'wheel_loads': wheel_load
            }


            return features, map2ground_rot

        except Exception as e:
            rospy.logwarn(f"Error extracting features for wheel {wheel_idx}: {e}")
            return None

    def predict_velocity(self, features):
        try:
            # Create base dataframe
            feature_df = pd.DataFrame([features])

            # VXG feature engineering
            feature_df['forward_component'] = feature_df['commanded_velocities'] * np.cos(feature_df['approach_angles']) * feature_df['wheel_loads']
            feature_df['angle_sin'] = np.sin(feature_df['approach_angles'])
            feature_df['angle_cos'] = np.cos(feature_df['approach_angles'])
            feature_df['color_slope'] = feature_df['color'] * feature_df['slope']

            features_vxg = ['slope', 'color', 'commanded_velocities', 'angle_sin', 'angle_cos', 'forward_component', 'color_slope']

            # VYG feature engineering
            feature_df['lateral_component'] = feature_df['commanded_velocities'] * np.sin(feature_df['approach_angles'])
            feature_df['slope_sin'] = np.sin(np.radians(feature_df['slope']))
            feature_df['slope_cos'] = np.cos(np.radians(feature_df['slope']))
            feature_df['lateral_traction'] = feature_df['wheel_loads'] * np.abs(feature_df['lateral_component']) * feature_df['slope_sin']
            feature_df['terrain_lateral_traction'] = feature_df['color'] * np.abs(feature_df['lateral_component']) * feature_df['slope_sin']

            features_vyg = ['slope_sin', 'slope_cos', 'commanded_velocities', 'angle_sin', 'angle_cos', 'lateral_component', 'lateral_traction', 'terrain_lateral_traction']

            # Make predictions
            X_vxg = feature_df[features_vxg]
            X_vxg_scaled = self.vxg_scaler.transform(X_vxg)
            #print(f"VXG features needed: {features_vxg}")
            #print(f"VXG features available: {list(feature_df.columns)}")
            #print(f"VXG feature values: {feature_df[features_vxg].iloc[0].to_dict()}")
            vxg_pred = float(self.vxg_model.predict(X_vxg_scaled)[0])
            #print(f"VYG features needed: {features_vyg}")
            #print(f"VYG features available: {list(feature_df.columns)}")
            #print(f"VYG feature values: {feature_df[features_vyg].iloc[0].to_dict()}")
            X_vyg = feature_df[features_vyg]
            X_vyg_scaled = self.vyg_scaler.transform(X_vyg)
            vyg_pred = float(self.vyg_model.predict(X_vyg_scaled)[0])

            ### START - TO REMOVE: hardcode zero output velocities if commanded velocity is near zero
            if (features['commanded_velocities'] <= 0.001):
                vxg_pred = 0.0
                vyg_pred = 0.0
            ### END - TO REMOVE

            # print(f"Slope: {features['slope']:.1f}°, Angle: {features['approach_angles']*180/np.pi:.1f}°, Color: {features['color']}, CmdVel: {features['commanded_velocities']:.3f}, VXG: {vxg_pred:.3f}, VYG: {vyg_pred:.3f}")
            rospy.loginfo(f"Slope: {features['slope']:.1f}°, Angle: {features['approach_angles']*180/np.pi:.1f}°, Color: {features['color']}, CmdVel: {features['commanded_velocities']:.3f}, VXG: {vxg_pred:.3f}, VYG: {vyg_pred:.3f}")

            return vxg_pred, vyg_pred

        except Exception as e:
            rospy.logwarn(f"Prediction failed: {e}")
            return 0.0, 0.0

    def publish_predictions(self, predictions, raw_publisher=False):
        self.output_msg.header.stamp = rospy.Time.now()

        for i, (vx, vy, vz) in enumerate(predictions):
            self.output_msg.vectors[i].x = vx
            self.output_msg.vectors[i].y = vy
            self.output_msg.vectors[i].z = vz

        if raw_publisher:
            self.output_msg.header.frame_id = 'ground frame'
            self.raw_outs_pub.publish(self.output_msg)
        else:
            self.output_msg.header.frame_id = 'map frame'
            self.velocity_pub.publish(self.output_msg)

    def compute_wheel_loads(self, wheel_idx):
        base_orient_euler = self.last_base_orientation.as_euler('XYZ', degrees=False)

        theta = - base_orient_euler[1] # positive -> forward is uphill
        alpha = base_orient_euler[0]   # positive -> left    is uphill

        wheel_load = self.wheel_masses[wheel_idx]
        for ll in range(len(self.other_link_masses)):
            wheel_to_link_ll = self.wheels_to_other_links[ll,:,wheel_idx]
            wheel_load = wheel_load + self.other_link_masses[ll] * (1 - (wheel_to_link_ll[0] + self.wheel_signes[wheel_idx,0]*wheel_to_link_ll[2]*np.tan(theta))/self.rover_wheelbase[0]) * (1 - (wheel_to_link_ll[1] + self.wheel_signes[wheel_idx,1]*wheel_to_link_ll[2]*np.tan(alpha))/self.rover_wheelbase[1])

        wheel_load = wheel_load * self.gravity

        return wheel_load

    def run(self):
        rospy.loginfo("ML Velocity Predictor running...")
        rospy.spin()

def main():
    try:
        predictor = MLVelocityPredictor()
        predictor.initialize()
        predictor.run()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down ML Velocity Predictor')
    except Exception as e:
        rospy.logerr(f'ML Velocity Predictor failed: {e}')

if __name__ == '__main__':
    main()
