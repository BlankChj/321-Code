#!/usr/bin/env python3

import numpy as np
import rospy
import copy
from udp_pkg.msg import PositionVelocityAccel
from geometry_msgs.msg import PoseStamped
import joblib
import pandas as pd

class RKF():
    def __init__(self):
        self.rkf_pub = rospy.Publisher('/rkf_py/pose', PoseStamped, queue_size=10)
        self.pose = PoseStamped()
        self.not_first = False
        self.gravity_coefficent = 9.8
        self.time_gap = 0
        self.lamb = 1e-6
        self.eye_lamb = np.matrix([
            [self.lamb, 0, 0, 0],
            [0, self.lamb, 0, 0],
            [0, 0, self.lamb, 0],
            [0, 0, 0, self.lamb]
        ], dtype=np.float64)
        self.zero_mat = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.eye_mat = np.matrix([[1, 0], [0, 1]], dtype=np.float64)
        self.sensor_mean = np.array([0, 0], dtype=np.float64)
        self.sensor_std = np.array([0.8, 0.5], dtype=np.float64)
        self.process_mean = np.array([0, 0], dtype=np.float64)
        self.process_std = np.array([1, 0.8], dtype=np.float64)

        self.x_hat = np.matrix([[0], [0]], dtype=np.float64)
        self.y_hat = np.matrix([[0], [0]], dtype=np.float64)
        self.z_hat = np.matrix([[0], [0]], dtype=np.float64)

        self.P_x = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.R_x = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.Q_x = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.R_x[0, 0] = self.sensor_std[0] * self.sensor_std[0]
        self.R_x[1, 1] = self.sensor_std[1] * self.sensor_std[1]
        self.Q_x[0, 0] = self.process_std[0] * self.process_std[0]
        self.Q_x[1, 1] = self.process_std[1] * self.process_std[1]

        self.P_y = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.R_y = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.Q_y = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.R_y[0, 0] = self.sensor_std[0] * self.sensor_std[0]
        self.R_y[1, 1] = self.sensor_std[1] * self.sensor_std[1]
        self.Q_y[0, 0] = self.process_std[0] * self.process_std[0]
        self.Q_y[1, 1] = self.process_std[1] * self.process_std[1]

        self.P_z = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.R_z = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.Q_z = np.matrix([[0, 0], [0, 0]], dtype=np.float64)
        self.R_z[0, 0] = self.sensor_std[0] * self.sensor_std[0]
        self.R_z[1, 1] = self.sensor_std[1] * self.sensor_std[1]
        self.Q_z[0, 0] = self.process_std[0] * self.process_std[0]
        self.Q_z[1, 1] = self.process_std[1] * self.process_std[1]

    def quaternion_to_rotation(self, x, y, z, w):
        norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
        if norm < 1e-10:
            raise ValueError("四元数模长为零，无法转换")
        w /= norm
        x /= norm
        y /= norm
        z /= norm
        r00 = 1 - 2 * y**2 - 2 * z**2
        r01 = 2 * x * y - 2 * w * z
        r02 = 2 * x * z + 2 * w * y
        r10 = 2 * x * y + 2 * w * z
        r11 = 1 - 2 * x**2 - 2 * z**2
        r12 = 2 * y * z - 2 * w * x
        r20 = 2 * x * z - 2 * w * y
        r21 = 2 * y * z + 2 * w * x
        r22 = 1 - 2 * x**2 - 2 * y**2
        rotation_matrix = np.matrix([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22]
        ], dtype=np.float64)
        return rotation_matrix

    def Psi(self, x, k):
        if abs(x) <= k:
            return x
        else:
            return np.sign(x) * k

    def RKF_filter(self, axis=0, k=0.5):
        if axis == 0:
            self.x_pre = self.A * self.x_hat + self.B * self.x_acc
            self.P_x_pre = self.A * self.P_x * self.A.T + self.time_gap * self.Q_x * self.time_gap
            temp1 = np.hstack((self.P_x_pre, self.zero_mat))
            temp2 = np.hstack((self.zero_mat, self.R_x))
            S_square = np.vstack((temp1, temp2))
            S_square += self.eye_lamb
            S_inv = np.linalg.inv(np.matrix(np.linalg.cholesky(S_square)).T)
            X_temp = np.vstack((self.eye_mat, self.C))
            X = S_inv * X_temp
            Y_temp = np.vstack((self.x_pre, self.out_x))
            Y = S_inv * Y_temp
            Omega = np.matrix([
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0]
            ], dtype=np.float64)
            for i in range(4):
                y_temp = Y[i, 0]
                x_temp = X[i, :] * self.x_hat
                x_temp_double = x_temp[0, 0]
                if y_temp == x_temp_double:
                    Omega[i, i] = 1
                else:
                    numerator = self.Psi(y_temp - x_temp_double, k)
                    denominator = y_temp - x_temp_double
                    Omega[i, i] = numerator / denominator
            self.x_hat = np.linalg.inv(X.T * Omega * X) * (X.T * Omega * Y)
            self.P_x = np.linalg.inv(X.T * Omega * X)
        elif axis == 1:
            self.y_pre = self.A * self.y_hat + self.B * self.y_acc
            self.P_y_pre = self.A * self.P_y * self.A.T + self.time_gap * self.Q_y * self.time_gap
            temp1 = np.hstack((self.P_y_pre, self.zero_mat))
            temp2 = np.hstack((self.zero_mat, self.R_y))
            S_square = np.vstack((temp1, temp2))
            S_square += self.eye_lamb
            S_inv = np.linalg.inv(np.matrix(np.linalg.cholesky(S_square)).T)
            X_temp = np.vstack((self.eye_mat, self.C))
            X = S_inv * X_temp
            Y_temp = np.vstack((self.y_pre, self.out_y))
            Y = S_inv * Y_temp
            Omega = np.matrix([
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0]
            ], dtype=np.float64)
            for i in range(4):
                y_temp = Y[i, 0]
                x_temp = X[i, :] * self.y_hat
                x_temp_double = x_temp[0, 0]
                if y_temp == x_temp_double:
                    Omega[i, i] = 1
                else:
                    numerator = self.Psi(y_temp - x_temp_double, k)
                    denominator = y_temp - x_temp_double
                    Omega[i, i] = numerator / denominator
            self.y_hat = np.linalg.inv(X.T * Omega * X) * (X.T * Omega * Y)
            self.P_y = np.linalg.inv(X.T * Omega * X)
        elif axis == 2:
            self.z_pre = self.A * self.z_hat + self.B * self.z_acc
            self.P_z_pre = self.A * self.P_z * self.A.T + self.time_gap * self.Q_z * self.time_gap
            temp1 = np.hstack((self.P_z_pre, self.zero_mat))
            temp2 = np.hstack((self.zero_mat, self.R_z))
            S_square = np.vstack((temp1, temp2))
            S_square += self.eye_lamb
            S_inv = np.linalg.inv(np.matrix(np.linalg.cholesky(S_square)).T)
            X_temp = np.vstack((self.eye_mat, self.C))
            X = S_inv * X_temp
            Y_temp = np.vstack((self.z_pre, self.out_z))
            Y = S_inv * Y_temp
            Omega = np.matrix([
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 0]
            ], dtype=np.float64)
            for i in range(4):
                y_temp = Y[i, 0]
                x_temp = X[i, :] * self.z_hat
                x_temp_double = x_temp[0, 0]
                if y_temp == x_temp_double:
                    Omega[i, i] = 1
                else:
                    numerator = self.Psi(y_temp - x_temp_double, k)
                    denominator = y_temp - x_temp_double
                    Omega[i, i] = numerator / denominator
            self.z_hat = np.linalg.inv(X.T * Omega * X) * (X.T * Omega * Y)
            self.P_z = np.linalg.inv(X.T * Omega * X)

    def predict(self, stamp, px_now, py_now, pz_now, ox_now, oy_now, oz_now, ow_now, vx_now, vy_now, vz_now, ax_now, ay_now, az_now):
        if self.not_first:
            now_time = rospy.Time.now().to_sec()
            self.time_gap = now_time - self.last_time
            # self.Rot = self.quaternion_to_rotation(ox_now, oy_now, oz_now, ow_now)
            self.Rot = self.quaternion_to_rotation(1, 0, 0, 0)
            self.x_acc = self.Rot[0, 0] * ax_now + self.Rot[0, 1] * ay_now + self.Rot[0, 2] * az_now
            self.y_acc = self.Rot[1, 0] * ax_now + self.Rot[1, 1] * ay_now + self.Rot[1, 2] * az_now
            self.z_acc = self.Rot[2, 0] * ax_now + self.Rot[2, 1] * ay_now + self.Rot[2, 2] * az_now - self.gravity_coefficent
            self.A = np.matrix([[1, self.time_gap], [0, 1]], dtype=np.float64)
            self.B = np.matrix([[0], [self.time_gap]], dtype=np.float64)
            self.C = np.matrix([[1, 0], [0, 1]], dtype=np.float64)
            self.out_x = np.matrix([[px_now], [vx_now]], dtype=np.float64)
            self.out_y = np.matrix([[py_now], [vy_now]], dtype=np.float64)
            self.out_z = np.matrix([[pz_now], [vz_now]], dtype=np.float64)
            self.RKF_filter(axis=0, k=0.5)
            self.RKF_filter(axis=1, k=0.5)
            self.RKF_filter(axis=2, k=0.5)
            self.last_time = rospy.Time.now().to_sec()
            return self.x_hat[0, 0], self.y_hat[0, 0], self.z_hat[0, 0]
        else:
            self.x_hat[0, 0] = px_now
            self.x_hat[1, 0] = vx_now
            self.y_hat[0, 0] = py_now
            self.y_hat[1, 0] = vy_now
            self.z_hat[0, 0] = pz_now
            self.z_hat[1, 0] = vz_now
            self.x_pre = copy.deepcopy(self.x_hat)
            self.P_x_pre = copy.deepcopy(self.P_x)
            self.y_pre = copy.deepcopy(self.y_hat)
            self.P_y_pre = copy.deepcopy(self.P_y)
            self.z_pre = copy.deepcopy(self.z_hat)
            self.P_z_pre = copy.deepcopy(self.P_z)
            self.last_time = rospy.Time.now().to_sec()
            self.not_first = True
            return px_now, py_now, pz_now
    
    def predict_info(self, info):
        stamp = info.stamp
        px_now = info.x_pos
        py_now = info.y_pos
        pz_now = info.z_pos
        ox_now = info.x_ori
        oy_now = info.y_ori
        oz_now = info.z_ori
        ow_now = info.w_ori
        vx_now = info.x_vel
        vy_now = info.y_vel
        vz_now = info.z_vel
        ax_now = info.x_acc
        ay_now = info.y_acc
        az_now = info.z_acc
        xx, yy, zz = self.predict(stamp, px_now, py_now, pz_now, ox_now, oy_now, oz_now, ow_now, vx_now, vy_now, vz_now, ax_now, ay_now, az_now)
        self.pose.header.frame_id = info.frame_id
        self.pose.header.stamp = rospy.Time.from_sec(stamp)
        self.pose.pose.position.x = xx
        self.pose.pose.position.y = yy
        self.pose.pose.position.z = zz
        self.rkf_pub.publish(self.pose)
        return xx, yy, zz

class Detection():
    def __init__(self, model_path=None, use_model=False):
        rospy.init_node("detection")
        self.model_path = model_path
        self.feature_cols = ["stamp","dt","x_pos","y_pos","z_pos","dx","dy","dz","x_ori","y_ori","z_ori","w_ori"]
        if model_path is not None:
            self.model = joblib.load(self.model_path)
        self.use_model = use_model
        self.real_cnt = [0, 0, 0, 0]
        self.pre_cnt = [0, 0, 0, 0]
        self.filter = RKF()
        self.fake = False
        self.leader = PositionVelocityAccel()
        self.old_old = PositionVelocityAccel()
        self.old = PositionVelocityAccel()
        self.old_old_pos = [0, 0, 0]
        self.old_pos = [0, 0, 0]
        self.old_old_var = 0
        self.old_var = 0
        self.info_flag = False
        self.replay_flag = False
        self.first = True
        self.second = True
        self.label = 0
        self.leader_sub = rospy.Subscriber('/leader/information', PositionVelocityAccel, self.callback)
    
    def callback(self, msg):
        self.leader = msg
        self.info_flag = True
        if self.first:
            self.old_old = copy.deepcopy(msg)
            x, y, z = self.filter.predict_info(self.old_old)
            self.old_old_pos = [x, y, z]
            self.old_old_var = ((x - self.old_old.x_pos)**2 + (y - self.old_old.y_pos)**2 + (z - self.old_old.z_pos)**2)
            self.first = False
        elif self.second:
            self.old = copy.deepcopy(msg)
            x, y, z = self.filter.predict_info(self.old)
            self.old_pos = [x, y, z]
            self.old_var = ((x - self.old.x_pos)**2 + (y - self.old.y_pos)**2 + (z - self.old.z_pos)**2)
            self.second = False
    
    def predict(self, info):
        xx, yy, zz = self.filter.predict_info(info)
        if self.label == 3:
            if info.stamp - self.old.stamp >= 1:
                self.label = 0
        elif info.stamp - self.old.stamp < 0:
            self.fake = True
            # self.label = 3
        elif ((info.x_pos == self.old.x_pos or self.old.x_pos == self.old_old.x_pos)
            and (info.y_pos == self.old.y_pos or self.old.y_pos == self.old_old.y_pos)
            and (info.z_pos == self.old.z_pos or self.old.z_pos == self.old_old.z_pos)):
            if info.stamp != self.old.stamp:
                self.label = 1
        elif (((xx - info.x_pos)**2 + (yy - info.y_pos)**2 + (zz - info.z_pos)**2) + self.old_old_var + self.old_var) / 3 > 0.1:
            self.label = 2
        else:
            self.label = 0
        var = 0
        if self.label == 0 or self.label == 2:
            var = ((xx - info.x_pos)**2 + (yy - info.y_pos)**2 + (zz - info.z_pos)**2)
        self.old_old = copy.deepcopy(self.old)
        self.old = copy.deepcopy(info)
        self.old_old_pos = copy.deepcopy(self.old_pos)
        self.old_pos = [xx, yy, zz]
        self.old_old_var = self.old_var
        self.old_var = var
    
    def xgboost(self, info):
        stamp = info.stamp
        dt = stamp - self.old.stamp
        x_pos = info.x_pos
        y_pos = info.y_pos
        z_pos = info.z_pos
        dx = x_pos - self.old.x_pos
        dy = y_pos - self.old.y_pos
        dz = z_pos - self.old.z_pos
        x_ori = info.x_ori
        y_ori = info.y_ori
        z_ori = info.z_ori
        w_ori = info.w_ori
        row = [[stamp, dt, x_pos, y_pos, z_pos, dx, dy, dz, x_ori, y_ori, z_ori, w_ori]]
        data = pd.DataFrame(row, columns=self.feature_cols)
        pred_label = self.model.predict(data)[0]
        return pred_label == 2
    
    def predict_model(self, info):
        if self.label == 3:
            if info.stamp - self.old.stamp >= 1:
                self.label = 0
        elif info.stamp - self.old.stamp < 0:
            self.fake = True
            # self.label = 3
        elif ((info.x_pos == self.old.x_pos or self.old.x_pos == self.old_old.x_pos)
            and (info.y_pos == self.old.y_pos or self.old.y_pos == self.old_old.y_pos)
            and (info.z_pos == self.old.z_pos or self.old.z_pos == self.old_old.z_pos)):
            if info.stamp != self.old.stamp:
                self.label = 1
        elif self.xgboost(info):
            self.label = 2
        else:
            self.label = 0
        self.old_old = copy.deepcopy(self.old)
        self.old = copy.deepcopy(info)
    
    def main(self):
        while True:
            if not self.first:
                break
        while True:
            if not self.second:
                break
        while not rospy.is_shutdown():
            try:
                if self.info_flag:
                    ll = int(self.leader.frame_id[-1])
                    if self.use_model:
                        self.predict_model(copy.deepcopy(self.leader))
                    else:
                        self.predict(copy.deepcopy(self.leader))
                    self.real_cnt[ll] += 1
                    if ll == self.label:
                        self.pre_cnt[self.label] += 1
                    self.info_flag = False
                    if self.fake:
                        self.fake = False
                        self.label = 3
                    print(f"-----------------------------\n 无攻击检测准确率为:{(self.pre_cnt[0] / self.real_cnt[0]) if self.real_cnt[0] != 0 else 1.0}\n Dos attack 检测准确率为:{(self.pre_cnt[1] / self.real_cnt[1]) if self.real_cnt[1] != 0 else 1.0}\n FDI attack检测准确率为:{(self.pre_cnt[2] / self.real_cnt[2]) if self.real_cnt[2] != 0 else 1.0}\n replay attack检测准确率为:{(self.pre_cnt[3] / self.real_cnt[3]) if self.real_cnt[3] != 0 else 1.0}")
            except KeyboardInterrupt:
                break

if __name__ == "__main__":
    model_path = "xgboost_model.pkl"
    detection = Detection()  # detection = Detection(model_path, True)
    detection.main()
