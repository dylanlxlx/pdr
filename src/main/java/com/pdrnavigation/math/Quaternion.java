package com.pdrnavigation.math;

import com.pdrnavigation.utils.Vector3D;

/**
 * 四元数类，用于表示和计算方向
 * 实现了 dcm2qua.m 和 qua2dcm.m 中的四元数操作
 */
public class Quaternion {
    // 组件 (w, x, y, z)，其中 w 是标量部分
    public double w; // 标量部分（在 MATLAB 代码中以前是 a）
    public double x; // 向量部分（以前是 b）
    public double y; // 向量部分（以前是 c）
    public double z; // 向量部分（以前是 d）

    /**
     * 创建一个新的单位四元数（无旋转）
     */
    public Quaternion() {
        this.w = 1.0;
        this.x = 0.0;
        this.y = 0.0;
        this.z = 0.0;
    }

    /**
     * 使用给定的组件创建一个新的四元数
     * @param w 标量部分
     * @param x 向量部分的 X 分量
     * @param y 向量部分的 Y 分量
     * @param z 向量部分的 Z 分量
     */
    public Quaternion(double w, double x, double y, double z) {
        this.w = w;
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * 复制构造函数
     * @param其他 Quaternion 进行复制
     */
    public Quaternion(Quaternion other) {
        this.w = other.w;
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
    }

    /**
     * 从旋转轴和角度创建四元数
     * @param axis 旋转轴（应为归一化向量）
     * @param angle 旋转角度（弧度）
     * @return 表示旋转的新四元数
     */
    public static Quaternion fromAxisAngle(Vector3D axis, double angle) {
        double halfAngle = angle / 2.0;
        double sinHalfAngle = Math.sin(halfAngle);

        return new Quaternion(
                Math.cos(halfAngle),
                axis.x * sinHalfAngle,
                axis.y * sinHalfAngle,
                axis.z * sinHalfAngle
        );
    }

    /**
     * 从欧拉角（横滚角、俯仰角、偏航角）创建四元数
     * 实现了 eulr2dcm.m 和 dcm2qua.m 的组合功能
     * @param roll 横滚角（弧度），绕 X 轴旋转
     * @param pitch 俯仰角（弧度），绕 Y 轴旋转
     * @param yaw 偏航角（弧度），绕 Z 轴旋转
     * @return 表示旋转的新四元数
     */
    public static Quaternion fromEulerAngles(double roll, double pitch, double yaw) {
        // 计算半角
        double halfRoll = roll / 2.0;
        double halfPitch = pitch / 2.0;
        double halfYaw = yaw / 2.0;

        // 计算正弦和余弦
        double cosRoll = Math.cos(halfRoll);
        double sinRoll = Math.sin(halfRoll);
        double cosPitch = Math.cos(halfPitch);
        double sinPitch = Math.sin(halfPitch);
        double cosYaw = Math.cos(halfYaw);
        double sinYaw = Math.sin(halfYaw);

        // 计算四元数组件
        return new Quaternion(
                cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw,
                sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
                cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
                cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw
        );
    }

    /**
     * 将方向余弦矩阵 (DCM) 转换为四元数
     * 实现了 dcm2qua.m 函数
     * @param dcm 3x3 旋转矩阵（机体到导航坐标系）
     * @return 表示旋转的新四元数
     */
    public static Quaternion fromDCM(double[][] dcm) {
        double[] s = new double[5];

        s[4] = dcm[0][0] + dcm[1][1] + dcm[2][2];
        s[0] = 1.0 + s[4];
        s[1] = 1.0 + 2.0 * dcm[0][0] - s[4];
        s[2] = 1.0 + 2.0 * dcm[1][1] - s[4];
        s[3] = 1.0 + 2.0 * dcm[2][2] - s[4];

        // 找到最大值索引
        int imax = 0;
        double cmax = s[0];
        for (int i = 1; i < 5; i++) {
            if (s[i] > cmax) {
                cmax = s[i];
                imax = i;
            }
        }

        Quaternion q = new Quaternion();

        switch (imax) {
            case 0:
                q.w = 0.5 * Math.sqrt(s[0]);
                q.x = 0.25 * (dcm[2][1] - dcm[1][2]) / q.w;
                q.y = 0.25 * (dcm[0][2] - dcm[2][0]) / q.w;
                q.z = 0.25 * (dcm[1][0] - dcm[0][1]) / q.w;
                break;
            case 1:
                q.x = 0.5 * Math.sqrt(s[1]);
                q.y = 0.25 * (dcm[1][0] + dcm[0][1]) / q.x;
                q.z = 0.25 * (dcm[0][2] + dcm[2][0]) / q.x;
                q.w = 0.25 * (dcm[2][1] - dcm[1][2]) / q.x;
                break;
            case 2:
                q.y = 0.5 * Math.sqrt(s[2]);
                q.z = 0.25 * (dcm[2][1] + dcm[1][2]) / q.y;
                q.w = 0.25 * (dcm[0][2] - dcm[2][0]) / q.y;
                q.x = 0.25 * (dcm[1][0] + dcm[0][1]) / q.y;
                break;
            case 3:
                q.z = 0.5 * Math.sqrt(s[3]);
                q.w = 0.25 * (dcm[1][0] - dcm[0][1]) / q.z;
                q.x = 0.25 * (dcm[0][2] + dcm[2][0]) / q.z;
                q.y = 0.25 * (dcm[2][1] + dcm[1][2]) / q.z;
                break;
        }

        return q;
    }

    /**
     * 将此四元数转换为方向余弦矩阵 (DCM)
     * 实现了 qua2dcm.m 函数
     * @return 3x3 旋转矩阵（机体到导航坐标系）
     */
    public double[][] toDCM() {
        double[][] dcm = new double[3][3];

        // 使用 w, x, y, z 符号（而不是 MATLAB 代码中的 a, b, c, d）
        dcm[0][0] = w*w + x*x - y*y - z*z;
        dcm[0][1] = 2*(x*y - w*z);
        dcm[0][2] = 2*(x*z + w*y);

        dcm[1][0] = 2*(x*y + w*z);
        dcm[1][1] = w*w - x*x + y*y - z*z;
        dcm[1][2] = 2*(y*z - w*x);

        dcm[2][0] = 2*(x*z - w*y);
        dcm[2][1] = 2*(y*z + w*x);
        dcm[2][2] = w*w - x*x - y*y + z*z;

        return dcm;
    }

    /**
     * 将此四元数转换为欧拉角（横滚角、俯仰角、偏航角）
     * @return 包含 [横滚角, 俯仰角, 偏航角] 的数组（弧度）
     */
    public double[] toEulerAngles() {
        double[][] dcm = toDCM();

        // 从 DCM 提取欧拉角
        double roll = Math.atan2(dcm[2][1], dcm[2][2]);
        double pitch = Math.asin(-dcm[2][0]);
        double yaw = Math.atan2(dcm[1][0], dcm[0][0]);

        return new double[] { roll, pitch, yaw };
    }

    /**
     * 计算此四元数的模（长度）
     * @return 四元数的模
     */
    public double magnitude() {
        return Math.sqrt(w*w + x*x + y*y + z*z);
    }

    /**
     * 归一化此四元数（使其成为单位长度）
     * @return 此四元数，用于链式调用
     */
    public Quaternion normalize() {
        double mag = magnitude();
        if (mag > 0) {
            w /= mag;
            x /= mag;
            y /= mag;
            z /= mag;
        }
        return this;
    }

    /**
     * 将此四元数乘以另一个四元数（连接旋转）
     * @param q 要相乘的四元数
     * @return 此四元数，用于链式调用
     */
    public Quaternion multiply(Quaternion q) {
        double newW = w*q.w - x*q.x - y*q.y - z*q.z;
        double newX = w*q.x + x*q.w + y*q.z - z*q.y;
        double newY = w*q.y - x*q.z + y*q.w + z*q.x;
        double newZ = w*q.z + x*q.y - y*q.x + z*q.w;

        w = newW;
        x = newX;
        y = newY;
        z = newZ;

        return this;
    }

    /**
     * 获取此四元数的共轭
     * @return 新的共轭四元数
     */
    public Quaternion conjugate() {
        return new Quaternion(w, -x, -y, -z);
    }

    /**
     * 使用此四元数旋转一个向量
     * @param v 要旋转的向量
     * @return 新的旋转向量
     */
    public Vector3D rotate(Vector3D v) {
        // 将向量转换为四元数（w=0）
        Quaternion qv = new Quaternion(0, v.x, v.y, v.z);

        // q * v * q^-1
        Quaternion result = new Quaternion(this);
        result.normalize();
        result.multiply(qv);
        result.multiply(result.conjugate());

        return new Vector3D(result.x, result.y, result.z);
    }

    @Override
    public String toString() {
        return String.format("[%.6f, %.6f, %.6f, %.6f]", w, x, y, z);
    }
}
