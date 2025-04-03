package com.pdrnavigation.filters;

/**
 * 扩展卡尔曼滤波器实现
 * 基于 MATLAB 代码中的 EKF.m 和 map_aid_ori_kal.m 文件
 */
public class KalmanFilter {
    // 状态矩阵
    private double[][] A; // 状态转移矩阵
    private double[][] P; // 误差协方差矩阵
    private double[][] Q; // 过程噪声协方差矩阵
    private double[][] R; // 测量噪声协方差矩阵
    private double[][] H; // 测量矩阵
    private double[] x;   // 状态向量
    private int dim;      // 状态向量的维度

    /**
     * 创建一个新的卡尔曼滤波器
     * @param stateDim 状态向量的维度
     * @param measureDim 测量向量的维度
     */
    public KalmanFilter(int stateDim, int measureDim) {
        this.dim = stateDim;

        // 初始化矩阵为默认值
        A = createIdentityMatrix(stateDim);
        P = createIdentityMatrix(stateDim);
        Q = createZeroMatrix(stateDim, stateDim);
        R = createIdentityMatrix(measureDim);
        H = createZeroMatrix(measureDim, stateDim);
        x = new double[stateDim];
    }

    /**
     * 设置状态转移矩阵
     * @param A 新的状态转移矩阵
     */
    public void setStateTransitionMatrix(double[][] A) {
        validateMatrix(A, dim, dim, "状态转移矩阵");
        this.A = deepCopy(A);
    }

    /**
     * 设置测量矩阵
     * @param H 新的测量矩阵
     */
    public void setMeasurementMatrix(double[][] H) {
        validateMatrix(H, H.length, dim, "测量矩阵");
        this.H = deepCopy(H);
    }

    /**
     * 设置过程噪声协方差矩阵
     * @param Q 新的过程噪声协方差矩阵
     */
    public void setProcessNoiseCovariance(double[][] Q) {
        validateMatrix(Q, dim, dim, "过程噪声协方差矩阵");
        this.Q = deepCopy(Q);
    }

    /**
     * 设置测量噪声协方差矩阵
     * @param R 新的测量噪声协方差矩阵
     */
    public void setMeasurementNoiseCovariance(double[][] R) {
        validateMatrix(R, R.length, R.length, "测量噪声协方差矩阵");
        this.R = deepCopy(R);
    }

    /**
     * 设置误差协方差矩阵
     * @param P 新的误差协方差矩阵
     */
    public void setErrorCovariance(double[][] P) {
        validateMatrix(P, dim, dim, "误差协方差矩阵");
        this.P = deepCopy(P);
    }

    /**
     * 设置状态向量
     * @param x 新的状态向量
     */
    public void setState(double[] x) {
        if (x.length != dim) {
            throw new IllegalArgumentException("状态向量必须具有维度 " + dim);
        }
        this.x = x.clone();
    }

    /**
     * 预测下一个状态
     */
    public void predict() {
        // x = A * x
        double[] xNew = new double[dim];
        for (int i = 0; i < dim; i++) {
            xNew[i] = 0;
            for (int j = 0; j < dim; j++) {
                xNew[i] += A[i][j] * x[j];
            }
        }
        x = xNew;

        // P = A * P * A' + Q
        double[][] AP = multiplyMatrices(A, P);
        double[][] AT = transpose(A);
        double[][] APAT = multiplyMatrices(AP, AT);
        P = addMatrices(APAT, Q);
    }

    /**
     * 使用非线性状态转移函数预测下一个状态
     * @param nonLinearStateTransition 计算下一个状态的函数
     */
    public void predictNonLinear(NonLinearStateTransition nonLinearStateTransition) {
        // 获取状态转移的雅可比矩阵
        double[][] jacobian = nonLinearStateTransition.getJacobian(x);

        // 使用非线性函数更新状态
        x = nonLinearStateTransition.predict(x);

        // 使用雅可比矩阵更新 P
        double[][] JP = multiplyMatrices(jacobian, P);
        double[][] JT = transpose(jacobian);
        double[][] JPJT = multiplyMatrices(JP, JT);
        P = addMatrices(JPJT, Q);
    }

    /**
     * 使用测量更新状态
     * @param z 测量向量
     */
    public void update(double[] z) {
        // y = z - H * x (测量残差)
        double[] Hx = new double[z.length];
        for (int i = 0; i < z.length; i++) {
            Hx[i] = 0;
            for (int j = 0; j < dim; j++) {
                Hx[i] += H[i][j] * x[j];
            }
        }

        double[] y = new double[z.length];
        for (int i = 0; i < z.length; i++) {
            y[i] = z[i] - Hx[i];
        }

        // S = H * P * H' + R (创新协方差)
        double[][] HP = multiplyMatrices(H, P);
        double[][] HT = transpose(H);
        double[][] HPH = multiplyMatrices(HP, HT);
        double[][] S = addMatrices(HPH, R);

        // K = P * H' * S^-1 (卡尔曼增益)
        double[][] PHT = multiplyMatrices(P, HT);
        double[][] Sinv = invertMatrix(S);
        double[][] K = multiplyMatrices(PHT, Sinv);

        // x = x + K * y (更新状态)
        double[] Ky = new double[dim];
        for (int i = 0; i < dim; i++) {
            Ky[i] = 0;
            for (int j = 0; j < y.length; j++) {
                Ky[i] += K[i][j] * y[j];
            }
        }

        for (int i = 0; i < dim; i++) {
            x[i] += Ky[i];
        }

        // P = (I - K * H) * P (更新误差协方差)
        double[][] I = createIdentityMatrix(dim);
        double[][] KH = multiplyMatrices(K, H);
        double[][] IKH = subtractMatrices(I, KH);
        P = multiplyMatrices(IKH, P);
    }

    /**
     * 使用非线性测量函数更新状态
     * @param z 测量向量
     * @param h 计算预期测量的函数
     * @param H 测量函数的雅可比矩阵
     */
    public void updateNonLinear(double[] z, double[] h, double[][] H) {
        // y = z - h(x) (测量残差)
        double[] y = new double[z.length];
        for (int i = 0; i < z.length; i++) {
            y[i] = z[i] - h[i];
        }

        // S = H * P * H' + R (创新协方差)
        double[][] HP = multiplyMatrices(H, P);
        double[][] HT = transpose(H);
        double[][] HPH = multiplyMatrices(HP, HT);
        double[][] S = addMatrices(HPH, R);

        // K = P * H' * S^-1 (卡尔曼增益)
        double[][] PHT = multiplyMatrices(P, HT);
        double[][] Sinv = invertMatrix(S);
        double[][] K = multiplyMatrices(PHT, Sinv);

        // x = x + K * y (更新状态)
        double[] Ky = new double[dim];
        for (int i = 0; i < dim; i++) {
            Ky[i] = 0;
            for (int j = 0; j < y.length; j++) {
                Ky[i] += K[i][j] * y[j];
            }
        }

        for (int i = 0; i < dim; i++) {
            x[i] += Ky[i];
        }

        // P = (I - K * H) * P (更新误差协方差)
        double[][] I = createIdentityMatrix(dim);
        double[][] KH = multiplyMatrices(K, H);
        double[][] IKH = subtractMatrices(I, KH);
        P = multiplyMatrices(IKH, P);
    }

    /**
     * 获取当前状态估计
     * @return 当前状态向量
     */
    public double[] getState() {
        return x.clone();
    }

    /**
     * 获取当前误差协方差
     * @return 当前误差协方差矩阵
     */
    public double[][] getErrorCovariance() {
        return deepCopy(P);
    }

    /**
     * 非线性状态转移函数的接口
     */
    public interface NonLinearStateTransition {
        /**
         * 使用非线性函数预测下一个状态
         * @param x 当前状态
         * @return 下一个状态
         */
        double[] predict(double[] x);

        /**
         * 获取状态转移函数的雅可比矩阵
         * @param x 当前状态
         * @return 雅可比矩阵
         */
        double[][] getJacobian(double[] x);
    }

    /**
     * 创建单位矩阵
     * @param n 矩阵维度
     * @return n x n 单位矩阵
     */
    private static double[][] createIdentityMatrix(int n) {
        double[][] I = new double[n][n];
        for (int i = 0; i < n; i++) {
            I[i][i] = 1.0;
        }
        return I;
    }

    /**
     * 创建零矩阵
     * @param rows 行数
     * @param cols 列数
     * @return rows x cols 零矩阵
     */
    private static double[][] createZeroMatrix(int rows, int cols) {
        return new double[rows][cols];
    }

    /**
     * 乘以两个矩阵
     * @param A 第一个矩阵
     * @param B 第二个矩阵
     * @return A * B
     */
    private static double[][] multiplyMatrices(double[][] A, double[][] B) {
        int rowsA = A.length;
        int colsA = A[0].length;
        int colsB = B[0].length;

        double[][] C = new double[rowsA][colsB];

        for (int i = 0; i < rowsA; i++) {
            for (int j = 0; j < colsB; j++) {
                for (int k = 0; k < colsA; k++) {
                    C[i][j] += A[i][k] * B[k][j];
                }
            }
        }

        return C;
    }

    /**
     * 加两个矩阵
     * @param A 第一个矩阵
     * @param B 第二个矩阵
     * @return A + B
     */
    private static double[][] addMatrices(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;

        double[][] C = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                C[i][j] = A[i][j] + B[i][j];
            }
        }

        return C;
    }

    /**
     * 减两个矩阵
     * @param A 第一个矩阵
     * @param B 第二个矩阵
     * @return A - B
     */
    private static double[][] subtractMatrices(double[][] A, double[][] B) {
        int rows = A.length;
        int cols = A[0].length;

        double[][] C = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                C[i][j] = A[i][j] - B[i][j];
            }
        }

        return C;
    }

    /**
     * 转置矩阵
     * @param A 要转置的矩阵
     * @return A 的转置
     */
    private static double[][] transpose(double[][] A) {
        int rows = A.length;
        int cols = A[0].length;

        double[][] AT = new double[cols][rows];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                AT[j][i] = A[i][j];
            }
        }

        return AT;
    }

    /**
     * 逆矩阵（简单实现适用于小矩阵）
     * @param A 要逆的矩阵
     * @return A 的逆矩阵
     */
    private static double[][] invertMatrix(double[][] A) {
        int n = A.length;

        // 特殊情况：1x1 矩阵
        if (n == 1) {
            return new double[][] {{1.0 / A[0][0]}};
        }

        // 特殊情况：2x2 矩阵
        if (n == 2) {
            double det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
            double[][] inv = new double[2][2];
            inv[0][0] = A[1][1] / det;
            inv[0][1] = -A[0][1] / det;
            inv[1][0] = -A[1][0] / det;
            inv[1][1] = A[0][0] / det;
            return inv;
        }

        // 对于更大的矩阵，使用更通用的算法（高斯消元法）
        double[][] augmented = new double[n][2 * n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented[i][j] = A[i][j];
                augmented[i][j + n] = (i == j) ? 1.0 : 0.0;
            }
        }

        // 高斯消元法
        for (int i = 0; i < n; i++) {
            // 找到主元
            int max = i;
            for (int j = i + 1; j < n; j++) {
                if (Math.abs(augmented[j][i]) > Math.abs(augmented[max][i])) {
                    max = j;
                }
            }

            // 交换行
            if (max != i) {
                for (int j = 0; j < 2 * n; j++) {
                    double temp = augmented[i][j];
                    augmented[i][j] = augmented[max][j];
                    augmented[max][j] = temp;
                }
            }

            // 将行缩放为对角线上有 1
            double scale = augmented[i][i];
            if (Math.abs(scale) < 1e-10) {
                throw new ArithmeticException("矩阵是奇异的");
            }

            for (int j = 0; j < 2 * n; j++) {
                augmented[i][j] /= scale;
            }

            // 消去其他行
            for (int j = 0; j < n; j++) {
                if (j != i) {
                    double factor = augmented[j][i];
                    for (int k = 0; k < 2 * n; k++) {
                        augmented[j][k] -= factor * augmented[i][k];
                    }
                }
            }
        }

        // 提取逆矩阵
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                inv[i][j] = augmented[i][j + n];
            }
        }

        return inv;
    }

    /**
     * 创建矩阵的深拷贝
     * @param A 要复制的矩阵
     * @return A 的深拷贝
     */
    private static double[][] deepCopy(double[][] A) {
        int rows = A.length;
        int cols = A[0].length;

        double[][] copy = new double[rows][cols];

        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                copy[i][j] = A[i][j];
            }
        }

        return copy;
    }

    /**
     * 验证矩阵具有预期的维度
     * @param matrix 要验证的矩阵
     * @param rows 预期的行数
     * @param cols 预期的列数
     * @param name 矩阵的名称，用于错误消息
     */
    private static void validateMatrix(double[][] matrix, int rows, int cols, String name) {
        if (matrix.length != rows) {
            throw new IllegalArgumentException(name + " 必须具有 " + rows + " 行");
        }

        for (int i = 0; i < rows; i++) {
            if (matrix[i].length != cols) {
                throw new IllegalArgumentException(name + " 必须具有 " + cols + " 列");
            }
        }
    }
}
