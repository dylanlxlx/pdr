package com.pdrnavigation.data;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * 用于从文件加载传感器数据的类
 */
public class SensorDataLoader {
    /**
     * 从文本文件加载传感器数据
     * @param accFile 加速度计数据文件路径
     * @param gyroFile 陀螺仪数据文件路径
     * @param orientationFile 方向数据文件路径
     * @param samplingRate 采样率 (Hz)
     * @return 加载的传感器数据
     * @throws IOException 如果发生 I/O 错误
     */
    public static SensorData loadFromFiles(String accFile, String gyroFile, String orientationFile, double samplingRate) throws IOException {
        // 加载加速度计数据
        double[][] accData = loadTextFile(accFile);

        // 加载陀螺仪数据
        double[][] gyroData = loadTextFile(gyroFile);

        // 加载方向数据
        double[][] orientationData = loadTextFile(orientationFile);

        // 验证所有文件的行数是否相同
        int numRows = accData.length;
        if (gyroData.length != numRows || orientationData.length != numRows) {
            throw new IllegalArgumentException("所有数据文件的行数必须相同");
        }

        // 提取数据列
        double[] accX = extractColumn(accData, 0);
        double[] accY = extractColumn(accData, 1);
        double[] accZ = extractColumn(accData, 2);

        double[] gyroX = extractColumn(gyroData, 0);
        double[] gyroY = extractColumn(gyroData, 1);
        double[] gyroZ = extractColumn(gyroData, 2);

        double[] roll = extractColumn(orientationData, 1);  // 横滚角是 ori.txt 文件中的第 1 列
        double[] pitch = extractColumn(orientationData, 2); // 俯仰角是 ori.txt 文件中的第 2 列
        double[] yaw = extractColumn(orientationData, 0);   // 偏航角是 ori.txt 文件中的第 0 列

        // 创建并返回传感器数据
        return new SensorData(
                accX, accY, accZ,
                gyroX, gyroY, gyroZ,
                roll, pitch, yaw,
                samplingRate
        );
    }

    /**
     * 将文本文件加载到二维数组中
     * @param filePath 文本文件路径
     * @return 文件中的数据二维数组
     * @throws IOException 如果发生 I/O 错误
     */
    private static double[][] loadTextFile(String filePath) throws IOException {
        List<double[]> dataList = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
            String line;
            while ((line = reader.readLine()) != null) {
                // 跳过空行和注释
                if (line.trim().isEmpty() || line.startsWith("%")) {
                    continue;
                }

                // 解析数据行
                String[] values = line.trim().split("\\s+");
                double[] dataRow = new double[values.length];

                for (int i = 0; i < values.length; i++) {
                    dataRow[i] = Double.parseDouble(values[i]);
                }

                dataList.add(dataRow);
            }
        }

        // 将列表转换为数组
        double[][] data = new double[dataList.size()][];
        for (int i = 0; i < dataList.size(); i++) {
            data[i] = dataList.get(i);
        }

        return data;
    }

    /**
     * 从二维数组中提取一列
     * @param data 二维数据数组
     * @param columnIndex 要提取的列的索引
     * @return 提取的列作为一维数组
     */
    private static double[] extractColumn(double[][] data, int columnIndex) {
        double[] column = new double[data.length];

        for (int i = 0; i < data.length; i++) {
            if (columnIndex < data[i].length) {
                column[i] = data[i][columnIndex];
            } else {
                column[i] = 0.0; // 如果列索引超出范围，默认值为 0.0
            }
        }

        return column;
    }
}
