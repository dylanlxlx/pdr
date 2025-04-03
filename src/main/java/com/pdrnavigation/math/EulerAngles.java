package com.pdrnavigation.math;

/**
 * 欧拉角操作的工具类
 */
public class EulerAngles {
    public double roll;  // 绕 X 轴的旋转
    public double pitch; // 绕 Y 轴的旋转
    public double yaw;   // 绕 Z 轴的旋转

    /**
     * 使用给定的角度创建一个新的欧拉角对象
     * @param roll 横滚角（弧度）
     * @param pitch 俯仰角（弧度）
     * @param yaw 偏航角（弧度）
     */
    public EulerAngles(double roll, double pitch, double yaw) {
        this.roll = roll;
        this.pitch = pitch;
        this.yaw = yaw;
    }

    /**
     * 创建一个新的零欧拉角对象
     */
    public EulerAngles() {
        this(0, 0, 0);
    }

    /**
     * 转换为方向余弦矩阵
     * @return 方向余弦矩阵
     */
    public DCM toDCM() {
        return DCM.fromEulerAngles(roll, pitch, yaw);
    }

    /**
     * 转换为四元数
     * @return 四元数
     */
    public Quaternion toQuaternion() {
        return Quaternion.fromEulerAngles(roll, pitch, yaw);
    }

    /**
     * 将角度归一化到范围 [-π, π]
     * @return 当前欧拉角对象，用于链式调用
     */
    public EulerAngles normalize() {
        roll = MathUtils.wrapToPI(roll);
        pitch = MathUtils.wrapToPI(pitch);
        yaw = MathUtils.wrapToPI(yaw);
        return this;
    }

    /**
     * 将角度从度转换为弧度
     * @param rollDeg 横滚角（度）
     * @param pitchDeg 俯仰角（度）
     * @param yawDeg 偏航角（度）
     * @return 新的欧拉角对象，角度为弧度
     */
    public static EulerAngles fromDegrees(double rollDeg, double pitchDeg, double yawDeg) {
        double deg2rad = Math.PI / 180.0;
        return new EulerAngles(rollDeg * deg2rad, pitchDeg * deg2rad, yawDeg * deg2rad);
    }

    /**
     * 将角度转换为度
     * @return 包含 [横滚角, 俯仰角, 偏航角] 的数组（度）
     */
    public double[] toDegrees() {
        double rad2deg = 180.0 / Math.PI;
        return new double[] {
                roll * rad2deg,
                pitch * rad2deg,
                yaw * rad2deg
        };
    }

    @Override
    public String toString() {
        return String.format("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
                toDegrees()[0], toDegrees()[1], toDegrees()[2]);
    }
}
