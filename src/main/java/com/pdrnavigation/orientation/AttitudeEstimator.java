package com.pdrnavigation.orientation;

import com.pdrnavigation.filters.KalmanFilter;
import com.pdrnavigation.math.EulerAngles;
import com.pdrnavigation.math.Quaternion;
import com.pdrnavigation.utils.Vector3D;

/**
 * 设备姿态估计器
 * 该类使用扩展卡尔曼滤波器从加速度计和陀螺仪数据中进行姿态估计
 */
public class AttitudeEstimator {
    private KalmanFilter attitudeFilter;
    private Quaternion quaternion;
    private double[] gyroData;
    private double timestep;
    private double gravity;

    /**
     * 创建一个新的姿态估计器
     * @param timestep 传感器读数之间的时间步长（秒）
     */
    public AttitudeEstimator(double timestep) {
        this.timestep = timestep;
        this.gravity = 9.81; // 默认重力

        // 初始化四元数为单位四元数（无旋转）
        this.quaternion = new Quaternion();

        // 初始化用于四元数估计的卡尔曼滤波器
        // 状态向量为 [q0, q1, q2, q3]（四元数）
        attitudeFilter = new KalmanFilter(4, 3);

        // 设置初始状态
        attitudeFilter.setState(new double[] {1.0, 0.0, 0.0, 0.0});

        // 根据陀螺仪噪声设置过程噪声
        double[][] Q = {
                {0.001, 0,     0,     0},
                {0,     0.001, 0,     0},
                {0,     0,     0.001, 0},
                {0,     0,     0,     0.001}
        };
        attitudeFilter.setProcessNoiseCovariance(Q);

        gyroData = new double[3];
    }

    /**
     * 使用新的陀螺仪和加速度计数据更新姿态估计
     * @param gyro 陀螺仪数据 [x, y, z]（单位：rad/s）
     * @param acc 加速度计数据 [x, y, z]（单位：m/s²）
     * @return 更新后的表示姿态的四元数
     */
    public Quaternion update(Vector3D gyro, Vector3D acc) {
        // 保存陀螺仪数据以供下一步预测使用
        gyroData[0] = gyro.x;
        gyroData[1] = gyro.y;
        gyroData[2] = gyro.z;

        // 归一化加速度计以获取重力方向
        double accMag = acc.magnitude();

        // 仅在加速度计接近重力大小时使用加速度计
        // （即，设备没有显著加速）
        boolean useAccelerometer = Math.abs(accMag - gravity) < 1.0;

        // 使用陀螺仪数据预测下一个状态
        predictAttitude();

        // 如果加速度计数据有效，则使用加速度计数据更新
        if (useAccelerometer) {
            updateWithAccelerometer(acc);
        }

        // 获取估计的四元数
        double[] state = attitudeFilter.getState();
        quaternion = new Quaternion(state[0], state[1], state[2], state[3]);
        quaternion.normalize();

        return quaternion;
    }

    /**
     * 使用陀螺仪数据预测姿态
     */
    private void predictAttitude() {
        // 获取当前四元数状态
        double[] state = attitudeFilter.getState();
        Quaternion q = new Quaternion(state[0], state[1], state[2], state[3]);

        // 根据陀螺仪数据计算旋转角度
        double angle = Math.sqrt(gyroData[0] * gyroData[0] +
                gyroData[1] * gyroData[1] +
                gyroData[2] * gyroData[2]) * timestep;

        if (angle > 0) {
            double scale = 1.0 / angle;
            Vector3D axis = new Vector3D(
                    gyroData[0] * scale,
                    gyroData[1] * scale,
                    gyroData[2] * scale
            );

            // 创建旋转四元数
            Quaternion deltaQ = Quaternion.fromAxisAngle(axis, angle);

            // 应用旋转
            q.multiply(deltaQ);
            q.normalize();
        }

        // 使用新四元数更新状态
        attitudeFilter.setState(new double[] {q.w, q.x, q.y, q.z});

        // 预测下一个状态
        attitudeFilter.predict();
    }

    /**
     * 使用加速度计数据更新姿态
     * @param acc 加速度计数据
     */
    private void updateWithAccelerometer(Vector3D acc) {
        // 获取当前四元数状态
        double[] state = attitudeFilter.getState();
        Quaternion q = new Quaternion(state[0], state[1], state[2], state[3]);

        // 导航坐标系中的参考重力向量
        Vector3D gravityRef = new Vector3D(0, 0, gravity);

        // 归一化加速度计
        Vector3D accNorm = new Vector3D(acc);
        accNorm.normalize();
        accNorm.multiply(gravity);

        // 使用当前四元数旋转重力向量
        Vector3D gravityBody = q.rotate(gravityRef);

        // 计算测量残差（误差）
        double[] z = new double[] {
                accNorm.x - gravityBody.x,
                accNorm.y - gravityBody.y,
                accNorm.z - gravityBody.z
        };

        // 计算测量雅可比矩阵
        double[][] H = calculateMeasurementJacobian(q, gravityRef);

        // 计算预期测量
        double[] h = new double[] {
                gravityBody.x,
                gravityBody.y,
                gravityBody.z
        };

        // 根据加速度计噪声设置测量噪声
        double[][] R = {
                {0.1, 0,   0},
                {0,   0.1, 0},
                {0,   0,   0.1}
        };
        attitudeFilter.setMeasurementNoiseCovariance(R);

        // 更新滤波器
        attitudeFilter.updateNonLinear(accNorm.toArray(), h, H);
    }

    /**
     * 计算测量雅可比矩阵
     * @param q 当前四元数
     * @param gravity 参考重力向量
     * @return 雅可比矩阵
     */
    private double[][] calculateMeasurementJacobian(Quaternion q, Vector3D gravity) {
        double g = gravity.z; // 重力大小

        // 简化的雅可比矩阵计算（可以从四元数旋转方程中推导）
        double[][] H = new double[3][4];

        // 四元数分量对旋转重力的偏导数
        H[0][0] = 2 * g * q.x;
        H[0][1] = 2 * g * q.w;
        H[0][2] = 2 * g * q.z;
        H[0][3] = 2 * g * q.y;

        H[1][0] = 2 * g * q.y;
        H[1][1] = 2 * g * q.z;
        H[1][2] = 2 * g * q.w;
        H[1][3] = -2 * g * q.x;

        H[2][0] = 2 * g * q.z;
        H[2][1] = -2 * g * q.y;
        H[2][2] = 2 * g * q.x;
        H[2][3] = 2 * g * q.w;

        return H;
    }

    /**
     * 获取当前姿态的四元数
     * @return 当前四元数
     */
    public Quaternion getQuaternion() {
        return new Quaternion(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    }

    /**
     * 获取当前姿态的欧拉角
     * @return 当前欧拉角（横滚角、俯仰角、偏航角）
     */
    public EulerAngles getEulerAngles() {
        double[] angles = quaternion.toEulerAngles();
        return new EulerAngles(angles[0], angles[1], angles[2]);
    }

    /**
     * 设置重力常数
     * @param gravity 新的重力值（m/s²）
     */
    public void setGravity(double gravity) {
        this.gravity = gravity;
    }

    /**
     * 重置滤波器到初始状态
     */
    public void reset() {
        quaternion = new Quaternion();
        attitudeFilter.setState(new double[] {1.0, 0.0, 0.0, 0.0});
    }
}
