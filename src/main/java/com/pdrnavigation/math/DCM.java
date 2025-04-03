package com.pdrnavigation.math;

import com.pdrnavigation.utils.Vector3D;

/**
 * 方向余弦矩阵 (DCM) 类，用于表示方向
 * 实现了 eulr2dcm.m 中的功能
 */
public class DCM {
    private double[][] matrix;

    /**
     * 创建一个新的单位 DCM（无旋转）
     */
    public DCM() {
        matrix = new double[3][3];
        matrix[0][0] = 1.0;
        matrix[1][1] = 1.0;
        matrix[2][2] = 1.0;
    }

    /**
     * 从 3x3 矩阵创建 DCM
     *
     * @param matrix 3x3 旋转矩阵
     */
    public DCM(double[][] matrix) {
        if (matrix.length != 3 || matrix[0].length != 3) {
            throw new IllegalArgumentException("DCM 必须是 3x3 矩阵");
        }
        this.matrix = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                this.matrix[i][j] = matrix[i][j];
            }
        }
    }

    /**
     * 从欧拉角（横滚角、俯仰角、偏航角）创建 DCM
     * 这实现了 eulr2dcm.m 函数
     *
     * @param roll  横滚角（弧度），绕 X 轴旋转
     * @param pitch 俯仰角（弧度），绕 Y 轴旋转
     * @param yaw   偏航角（弧度），绕 Z 轴旋转
     * @return 表示旋转的新 DCM
     */
    public static DCM fromEulerAngles(double roll, double pitch, double yaw) {
        double cpsi = Math.cos(yaw);
        double spsi = Math.sin(yaw);
        double cthe = Math.cos(pitch);
        double sthe = Math.sin(pitch);
        double cphi = Math.cos(roll);
        double sphi = Math.sin(roll);

        // 初始化每个轴的旋转矩阵
        // Z 轴旋转（偏航角）
        double[][] C1 = {
                {cpsi, spsi, 0},
                {-spsi, cpsi, 0},
                {0, 0, 1}
        };

        // Y 轴旋转（俯仰角）
        double[][] C2 = {
                {cthe, 0, -sthe},
                {0, 1, 0},
                {sthe, 0, cthe}
        };

        // X 轴旋转（横滚角）
        double[][] C3 = {
                {1, 0, 0},
                {0, cphi, sphi},
                {0, -sphi, cphi}
        };

        // 组合旋转：DCM = C3 * C2 * C1
        double[][] temp = multiplyMatrices(C3, C2);
        double[][] result = multiplyMatrices(temp, C1);

        return new DCM(result);
    }

    /**
     * 辅助方法，用于乘以两个 3x3 矩阵
     *
     * @param A 第一个矩阵
     * @param B 第二个矩阵
     * @return A * B 的结果
     */
    private static double[][] multiplyMatrices(double[][] A, double[][] B) {
        double[][] C = new double[3][3];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                C[i][j] = 0;
                for (int k = 0; k < 3; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return C;
    }

    /**
     * 获取此 DCM 的矩阵表示
     *
     * @return 3x3 旋转矩阵
     */
    public double[][] getMatrix() {
        return matrix.clone();
    }

    /**
     * 获取 DCM 的特定元素
     *
     * @param row 行索引（0-2）
     * @param col 列索引（0-2）
     * @return 元素值
     */
    public double get(int row, int col) {
        return matrix[row][col];
    }

    /**
     * 设置 DCM 的特定元素
     *
     * @param row   行索引（0-2）
     * @param col   列索引（0-2）
     * @param value 新值
     */
    public void set(int row, int col, double value) {
        matrix[row][col] = value;
    }

    /**
     * 将此 DCM 转换为欧拉角（横滚角、俯仰角、偏航角）
     *
     * @return 包含 [横滚角, 俯仰角, 偏航角] 的数组（弧度）
     */
    public double[] toEulerAngles() {
        double roll = Math.atan2(matrix[2][1], matrix[2][2]);
        double pitch = Math.asin(-matrix[2][0]);
        double yaw = Math.atan2(matrix[1][0], matrix[0][0]);

        return new double[]{roll, pitch, yaw};
    }

    /**
     * 将向量从机体坐标系转换到导航坐标系
     *
     * @param v 机体坐标系中的向量
     * @return 导航坐标系中的向量
     */
    public Vector3D transformToNavFrame(Vector3D v) {
        double x = matrix[0][0] * v.x + matrix[0][1] * v.y + matrix[0][2] * v.z;
        double y = matrix[1][0] * v.x + matrix[1][1] * v.y + matrix[1][2] * v.z;
        double z = matrix[2][0] * v.x + matrix[2][1] * v.y + matrix[2][2] * v.z;

        return new Vector3D(x, y, z);
    }

    /**
     * 将向量从导航坐标系转换到机体坐标系
     *
     * @param v 导航坐标系中的向量
     * @return 机体坐标系中的向量
     */
    public Vector3D transformToBodyFrame(Vector3D v) {
        double x = matrix[0][0] * v.x + matrix[1][0] * v.y + matrix[2][0] * v.z;
        double y = matrix[0][1] * v.x + matrix[1][1] * v.y + matrix[2][1] * v.z;
        double z = matrix[0][2] * v.x + matrix[1][2] * v.y + matrix[2][2] * v.z;

        return new Vector3D(x, y, z);
    }

    /**
     * 转置此 DCM
     *
     * @return 此 DCM 的转置
     */
    public DCM transpose() {
        double[][] result = new double[3][3];

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = matrix[j][i];
            }
        }

        return new DCM(result);
    }

    /**
     * 将此 DCM 乘以另一个 DCM
     *
     * @param other 要乘以的 DCM
     * @return 乘法结果的新 DCM
     */
    public DCM multiply(DCM other) {
        return new DCM(multiplyMatrices(matrix, other.matrix));
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 3; i++) {
            sb.append("[");
            for (int j = 0; j < 3; j++) {
                sb.append(String.format("%.6f", matrix[i][j]));
                if (j < 2) sb.append(", ");
            }
            sb.append("]\n");
        }
        return sb.toString();
    }
}
