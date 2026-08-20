package org.firstinspires.ftc.teamcode.lib.controller;

public final class LeastSquaresSolver {
    private LeastSquaresSolver() {}

    public static double[] solve(double[][] augmentedMatrix) {
        int n = augmentedMatrix.length;
        if (n == 0) {
            return new double[0];
        }
        int cols = augmentedMatrix[0].length;
        for (int pivot = 0; pivot < n; pivot++) {
            int maxRow = pivot;
            for (int row = pivot + 1; row < n; row++) {
                if (Math.abs(augmentedMatrix[row][pivot]) > Math.abs(augmentedMatrix[maxRow][pivot])) {
                    maxRow = row;
                }
            }
            if (Math.abs(augmentedMatrix[maxRow][pivot]) < 1e-6) {
                continue;
            }
            if (maxRow != pivot) {
                double[] tmp = augmentedMatrix[pivot];
                augmentedMatrix[pivot] = augmentedMatrix[maxRow];
                augmentedMatrix[maxRow] = tmp;
            }
            double pivotVal = augmentedMatrix[pivot][pivot];
            for (int col = pivot; col < cols; col++) {
                augmentedMatrix[pivot][col] /= pivotVal;
            }
            for (int row = 0; row < n; row++) {
                if (row == pivot) {
                    continue;
                }
                double factor = augmentedMatrix[row][pivot];
                for (int col = pivot; col < cols; col++) {
                    augmentedMatrix[row][col] -= factor * augmentedMatrix[pivot][col];
                }
            }
        }
        double[] result = new double[n];
        for (int i = 0; i < n; i++) {
            result[i] = augmentedMatrix[i][cols - 1];
        }
        return result;
    }
}

