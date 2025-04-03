package com.pdrnavigation.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * 文件操作的工具类
 */
public class FileUtils {
    /**
     * 读取包含数值数据的文本文件
     * @param filename 文件路径
     * @return 数值数据的二维数组
     * @throws IOException 如果发生 I/O 错误
     */
    public static double[][] readNumericFile(String filename) throws IOException {
        List<double[]> dataList = new ArrayList<>();

        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
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
     * 将数值数据的二维数组写入文本文件
     * @param filename 文件路径
     * @param data 数据的二维数组
     * @param header 头部行（可以为空）
     * @throws IOException 如果发生 I/O 错误
     */
    public static void writeNumericFile(String filename, double[][] data, String header) throws IOException {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
            // 如果提供了头部行，则写入头部
            if (header != null && !header.isEmpty()) {
                writer.println(header);
            }

            // 写入数据
            for (double[] row : data) {
                StringBuilder sb = new StringBuilder();
                for (int i = 0; i < row.length; i++) {
                    sb.append(String.format("%.6f", row[i]));
                    if (i < row.length - 1) {
                        sb.append(" ");
                    }
                }
                writer.println(sb.toString());
            }
        }
    }

    /**
     * 将轨迹写入 CSV 文件
     * @param filename 文件路径
     * @param north 北坐标位置
     * @param east 东坐标位置
     * @param stepLengths 步长（可以为 null）
     * @param orientations 步进方向（弧度）（可以为 null）
     * @throws IOException 如果发生 I/O 错误
     */
    public static void writeTrajectory(String filename, double[] north, double[] east,
                                       double[] stepLengths, double[] orientations) throws IOException {
        try (PrintWriter writer = new PrintWriter(new FileWriter(filename))) {
            // 写入头部
            StringBuilder header = new StringBuilder("Step,North,East");
            if (stepLengths != null) {
                header.append(",StepLength");
            }
            if (orientations != null) {
                header.append(",Orientation");
            }
            writer.println(header.toString());

            // 写入数据
            for (int i = 0; i < north.length; i++) {
                StringBuilder sb = new StringBuilder();
                sb.append(i).append(",").append(north[i]).append(",").append(east[i]);

                if (stepLengths != null && i < stepLengths.length) {
                    sb.append(",").append(stepLengths[i]);
                }

                if (orientations != null && i < orientations.length) {
                    sb.append(",").append(orientations[i]);
                }

                writer.println(sb.toString());
            }
        }
    }

    /**
     * 检查文件是否存在
     * @param filename 文件路径
     * @return 如果文件存在，返回 true；否则返回 false
     */
    public static boolean fileExists(String filename) {
        File file = new File(filename);
        return file.exists() && file.isFile();
    }

    /**
     * 如果目录不存在，则创建目录
     * @param directoryPath 目录路径
     * @return 如果目录存在或已创建，返回 true；否则返回 false
     */
    public static boolean createDirectory(String directoryPath) {
        File directory = new File(directoryPath);
        if (!directory.exists()) {
            return directory.mkdirs();
        }
        return directory.isDirectory();
    }

    /**
     * 获取文件扩展名
     * @param filename 文件路径
     * @return 文件扩展名（不带点）
     */
    public static String getFileExtension(String filename) {
        int dotIndex = filename.lastIndexOf('.');
        if (dotIndex > 0 && dotIndex < filename.length() - 1) {
            return filename.substring(dotIndex + 1).toLowerCase();
        }
        return "";
    }

    /**
     * 获取不带扩展名的文件名
     * @param filename 文件路径
     * @return 不带扩展名的文件名
     */
    public static String getFileNameWithoutExtension(String filename) {
        File file = new File(filename);
        String name = file.getName();
        int dotIndex = name.lastIndexOf('.');
        if (dotIndex > 0 && dotIndex < name.length() - 1) {
            return name.substring(0, dotIndex);
        }
        return name;
    }

    // 私有构造函数，防止实例化
    private FileUtils() {
        throw new UnsupportedOperationException("Utility classes should not be instantiated");
    }
}
