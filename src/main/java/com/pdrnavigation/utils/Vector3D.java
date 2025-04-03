package com.pdrnavigation.utils;

/**
 * 表示传感器数据和操作的 3D 向量类
 */
public class Vector3D {
    public double x;
    public double y;
    public double z;

    /**
     * 使用给定的分量创建一个新的 3D 向量
     * @param x X 分量
     * @param y Y 分量
     * @param z Z 分量
     */
    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
     * 创建一个新的零向量
     */
    public Vector3D() {
        this(0, 0, 0);
    }

    /**
     * 复制构造函数
     * @param other 要复制的向量
     */
    public Vector3D(Vector3D other) {
        this(other.x, other.y, other.z);
    }

    /**
     * 计算此向量的模（长度）
     * @return 向量的模
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    /**
     * 归一化此向量（使其成为单位向量）
     * @return 此向量，用于链式调用
     */
    public Vector3D normalize() {
        double mag = magnitude();
        if (mag > 0) {
            x /= mag;
            y /= mag;
            z /= mag;
        }
        return this;
    }

    /**
     * 获取此向量的归一化副本
     * @return 新的归一化向量
     */
    public Vector3D normalized() {
        Vector3D result = new Vector3D(this);
        result.normalize();
        return result;
    }

    /**
     * 将另一个向量加到此向量上
     * @param v 要加的向量
     * @return 此向量，用于链式调用
     */
    public Vector3D add(Vector3D v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return this;
    }

    /**
     * 从此向量中减去另一个向量
     * @param v 要减去的向量
     * @return 此向量，用于链式调用
     */
    public Vector3D subtract(Vector3D v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return this;
    }

    /**
     * 将此向量乘以一个标量
     * @param scalar 要乘的标量
     * @return 此向量，用于链式调用
     */
    public Vector3D multiply(double scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return this;
    }

    /**
     * 计算与此向量的点积
     * @param v 另一个向量
     * @return 点积
     */
    public double dot(Vector3D v) {
        return x * v.x + y * v.y + z * v.z;
    }

    /**
     * 计算与此向量的叉积
     * @param v 另一个向量
     * @return 新的叉积向量
     */
    public Vector3D cross(Vector3D v) {
        return new Vector3D(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
        );
    }

    /**
     * 检查此向量是否近似等于另一个向量
     * @param v 另一个向量
     * @param epsilon 等价的容差
     * @return 如果向量近似相等，返回 true
     */
    public boolean isEqual(Vector3D v, double epsilon) {
        return Math.abs(x - v.x) < epsilon &&
                Math.abs(y - v.y) < epsilon &&
                Math.abs(z - v.z) < epsilon;
    }

    /**
     * 将此向量转换为数组
     * @return数组 [x， y， z]
     */
    public double[] toArray() {
        return new double[] {x, y, z};
    }

    @Override
    public String toString() {
        return String.format("[%.6f, %.6f, %.6f]", x, y, z);
    }
}
