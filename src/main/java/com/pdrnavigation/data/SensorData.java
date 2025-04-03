package com.pdrnavigation.data;

/**
 * 用于存储 IMU 传感器数据的类
 */
public class SensorData {
    private double[] accelerometerX;
    private double[] accelerometerY;
    private double[] accelerometerZ;

    private double[] gyroscopeX;
    private double[] gyroscopeY;
    private double[] gyroscopeZ;

    private double[] roll;  // 绕 X 轴的旋转
    private double[] pitch; // 绕 Y 轴的旋转
    private double[] yaw;   // 绕 Z 轴的旋转

    private double samplingRate;
    private int dataLength;

    /**
     * 使用给定的数据点数量创建一个新的传感器数据对象
     * @param dataLength 数据点的数量
     */
    public SensorData(int dataLength) {
        this.dataLength = dataLength;

        accelerometerX = new double[dataLength];
        accelerometerY = new double[dataLength];
        accelerometerZ = new double[dataLength];

        gyroscopeX = new double[dataLength];
        gyroscopeY = new double[dataLength];
        gyroscopeZ = new double[dataLength];

        roll = new double[dataLength];
        pitch = new double[dataLength];
        yaw = new double[dataLength];

        samplingRate = 50.0; // 默认采样率 (Hz)
    }

    /**
     * 使用给定的数据创建一个新的传感器数据对象
     * @param accX 加速度计 X 轴数据
     * @param accY 加速度计 Y 轴数据
     * @param accZ 加速度计 Z 轴数据
     * @param gyroX 陀螺仪 X 轴数据
     * @param gyroY 陀螺仪 Y 轴数据
     * @param gyroZ 陀螺仪 Z 轴数据
     * @param roll 横滚角
     * @param pitch 俯仰角
     * @param yaw 偏航角
     * @param samplingRate 采样率 (Hz)
     */
    public SensorData(double[] accX, double[] accY, double[] accZ,
                      double[] gyroX, double[] gyroY, double[] gyroZ,
                      double[] roll, double[] pitch, double[] yaw,
                      double samplingRate) {

        // 验证所有数组长度是否相同
        int length = accX.length;
        if (accY.length != length || accZ.length != length ||
                gyroX.length != length || gyroY.length != length || gyroZ.length != length ||
                roll.length != length || pitch.length != length || yaw.length != length) {
            throw new IllegalArgumentException("所有数据数组的长度必须相同");
        }

        this.dataLength = length;
        this.samplingRate = samplingRate;

        this.accelerometerX = accX.clone();
        this.accelerometerY = accY.clone();
        this.accelerometerZ = accZ.clone();

        this.gyroscopeX = gyroX.clone();
        this.gyroscopeY = gyroY.clone();
        this.gyroscopeZ = gyroZ.clone();

        this.roll = roll.clone();
        this.pitch = pitch.clone();
        this.yaw = yaw.clone();
    }

    /**
     * 获取加速度计 X 轴数据
     * @return 加速度计 X 轴数据
     */
    public double[] getAccelerometerX() {
        return accelerometerX;
    }

    /**
     * 设置加速度计 X 轴数据
     * @param accelerometerX 新的加速度计 X 轴数据
     */
    public void setAccelerometerX(double[] accelerometerX) {
        validateArrayLength(accelerometerX);
        this.accelerometerX = accelerometerX.clone();
    }

    /**
     * 获取加速度计 Y 轴数据
     * @return 加速度计 Y 轴数据
     */
    public double[] getAccelerometerY() {
        return accelerometerY;
    }

    /**
     * 设置加速度计 Y 轴数据
     * @param accelerometerY 新的加速度计 Y 轴数据
     */
    public void setAccelerometerY(double[] accelerometerY) {
        validateArrayLength(accelerometerY);
        this.accelerometerY = accelerometerY.clone();
    }

    /**
     * 获取加速度计 Z 轴数据
     * @return 加速度计 Z 轴数据
     */
    public double[] getAccelerometerZ() {
        return accelerometerZ;
    }

    /**
     * 设置加速度计 Z 轴数据
     * @param accelerometerZ 新的加速度计 Z 轴数据
     */
    public void setAccelerometerZ(double[] accelerometerZ) {
        validateArrayLength(accelerometerZ);
        this.accelerometerZ = accelerometerZ.clone();
    }

    /**
     * 获取陀螺仪 X 轴数据
     * @return 陀螺仪 X 轴数据
     */
    public double[] getGyroscopeX() {
        return gyroscopeX;
    }

    /**
     * 设置陀螺仪 X 轴数据
     * @param gyroscopeX 新的陀螺仪 X 轴数据
     */
    public void setGyroscopeX(double[] gyroscopeX) {
        validateArrayLength(gyroscopeX);
        this.gyroscopeX = gyroscopeX.clone();
    }

    /**
     * 获取陀螺仪 Y 轴数据
     * @return 陀螺仪 Y 轴数据
     */
    public double[] getGyroscopeY() {
        return gyroscopeY;
    }

    /**
     * 设置陀螺仪 Y 轴数据
     * @param gyroscopeY 新的陀螺仪 Y 轴数据
     */
    public void setGyroscopeY(double[] gyroscopeY) {
        validateArrayLength(gyroscopeY);
        this.gyroscopeY = gyroscopeY.clone();
    }

    /**
     * 获取陀螺仪 Z 轴数据
     * @return 陀螺仪 Z 轴数据
     */
    public double[] getGyroscopeZ() {
        return gyroscopeZ;
    }

    /**
     * 设置陀螺仪 Z 轴数据
     * @param gyroscopeZ 新的陀螺仪 Z 轴数据
     */
    public void setGyroscopeZ(double[] gyroscopeZ) {
        validateArrayLength(gyroscopeZ);
        this.gyroscopeZ = gyroscopeZ.clone();
    }

    /**
     * 获取横滚角
     * @return 横滚角数组
     */
    public double[] getRoll() {
        return roll;
    }

    /**
     * 设置横滚角
     * @param roll 新的横滚角数组
     */
    public void setRoll(double[] roll) {
        validateArrayLength(roll);
        this.roll = roll.clone();
    }

    /**
     * 获取俯仰角
     * @return 俯仰角数组
     */
    public double[] getPitch() {
        return pitch;
    }

    /**
     * 设置俯仰角
     * @param pitch 新的俯仰角数组
     */
    public void setPitch(double[] pitch) {
        validateArrayLength(pitch);
        this.pitch = pitch.clone();
    }

    /**
     * 获取偏航角
     * @return 偏航角数组
     */
    public double[] getYaw() {
        return yaw;
    }

    /**
     * 设置偏航角
     * @param yaw 新的偏航角数组
     */
    public void setYaw(double[] yaw) {
        validateArrayLength(yaw);
        this.yaw = yaw.clone();
    }

    /**
     * 获取采样率
     * @return 采样率 (Hz)
     */
    public double getSamplingRate() {
        return samplingRate;
    }

    /**
     * 设置采样率
     * @param samplingRate 新的采样率 (Hz)
     */
    public void setSamplingRate(double samplingRate) {
        this.samplingRate = samplingRate;
    }

    /**
     * 获取数据长度
     * @return 数据点的数量
     */
    public int getDataLength() {
        return dataLength;
    }

    /**
     * 获取此数据的时间数组
     * @return 时间数组（以秒为单位）
     */
    public double[] getTimeArray() {
        double[] time = new double[dataLength];
        double dt = 1.0 / samplingRate;

        for (int i = 0; i < dataLength; i++) {
            time[i] = i * dt;
        }

        return time;
    }

    /**
     * 验证数组是否具有预期长度
     * @param array 要验证的数组
     */
    private void validateArrayLength(double[] array) {
        if (array.length != dataLength) {
            throw new IllegalArgumentException("数组长度必须为 " + dataLength);
        }
    }
}
