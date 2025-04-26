package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;

public class GaussianProcess {
    private List<double[]> points;
    private List<Double> values;
    private double[][] kernelMatrix;
    private double[] kernelVector;
    private double[][] inverseKernel;
    private double noise = 1e-6;

    public GaussianProcess() {
        points = new ArrayList<>();
        values = new ArrayList<>();
        kernelMatrix = new double[0][0];
        inverseKernel = new double[0][0];
        kernelVector = new double[0];
    }

    public void addPoint(double[] x, double y) {
        points.add(x.clone());
        values.add(y);
        updateKernelMatrix();
    }

    private void updateKernelMatrix() {
        int n = points.size();
        if (n == 1) {
            kernelMatrix = new double[1][1];
            kernelMatrix[0][0] = rbfKernel(points.get(0), points.get(0)) + noise;
            inverseKernel = new double[1][1];
            inverseKernel[0][0] = 1.0 / kernelMatrix[0][0];
        } else {
            /// Incremental update using Sherman-Morrison
            double[][] oldKernelMatrix = kernelMatrix;
            double[][] oldInverse = inverseKernel;
            kernelMatrix = new double[n][n];
            inverseKernel = new double[n][n];
            for (int i = 0; i < n - 1; i++) {
                for (int j = 0; j < n - 1; j++) {
                    kernelMatrix[i][j] = oldKernelMatrix[i][j];
                }
                kernelMatrix[i][n - 1] = rbfKernel(points.get(i), points.get(n - 1));
                kernelMatrix[n - 1][i] = kernelMatrix[i][n - 1];
            }
            kernelMatrix[n - 1][n - 1] = rbfKernel(points.get(n - 1), points.get(n - 1)) + noise;
            /// Sherman-Morrison update
            double[] u = new double[n];
            double[] v = new double[n];
            for (int i = 0; i < n - 1; i++) {
                u[i] = kernelMatrix[i][n - 1];
                v[i] = i == n - 1 ? 1.0 : 0.0;
            }
            u[n - 1] = 0.0;
            v[n - 1] = 1.0;
            double a = kernelMatrix[n - 1][n - 1];
            /// Compute oldInverse * u
            double[] temp = new double[n - 1];
            for (int i = 0; i < n - 1; i++) {
                double sum = 0.0;
                for (int j = 0; j < n - 1; j++) {
                    sum += oldInverse[i][j] * u[j];
                }
                temp[i] = sum;
            }
            /// Compute denominator: v^T * oldInverse * u + a
            double denom = a;
            for (int i = 0; i < n - 1; i++) {
                denom += v[i] * temp[i];
            }
            denom = denom == 0 ? 1e-10 : denom;
            for (int i = 0; i < n - 1; i++) {
                for (int j = 0; j < n - 1; j++) {
                    inverseKernel[i][j] = oldInverse[i][j] - temp[i] * temp[j] / denom;
                }
                inverseKernel[i][n - 1] = -temp[i] / denom;
                inverseKernel[n - 1][i] = -temp[i] / denom;
            }
            inverseKernel[n - 1][n - 1] = 1.0 / denom;
        }
    }

    public double predict(double[] x) {
        int n = points.size();
        kernelVector = new double[n];
        for (int i = 0; i < n; i++) {
            kernelVector[i] = rbfKernel(x, points.get(i));
        }
        double[] weights = matrixVectorMultiply(inverseKernel, kernelVector);
        double prediction = 0.0;
        for (int i = 0; i < n; i++) {
            prediction += weights[i] * values.get(i);
        }
        return prediction;
    }

    public double predictVariance(double[] x) {
        int n = points.size();
        kernelVector = new double[n];
        for (int i = 0; i < n; i++) {
            kernelVector[i] = rbfKernel(x, points.get(i));
        }
        double variance = rbfKernel(x, x) + noise;
        double[] temp = matrixVectorMultiply(inverseKernel, kernelVector);
        for (int i = 0; i < n; i++) {
            variance -= temp[i] * kernelVector[i];
        }
        return Math.max(variance, 1e-10);
    }

    private double rbfKernel(double[] x1, double[] x2) {
        double sigma = 1.0;
        double sum = 0.0;
        for (int i = 0; i < x1.length; i++) {
            double diff = x1[i] - x2[i];
            sum += diff * diff;
        }
        return Math.exp(-sum / (2 * sigma * sigma));
    }

    private double[] matrixVectorMultiply(double[][] matrix, double[] vector) {
        int n = matrix.length;
        double[] result = new double[n];
        for (int i = 0; i < n; i++) {
            double sum = 0.0;
            for (int j = 0; j < n; j++) {
                sum += matrix[i][j] * vector[j];
            }
            result[i] = sum;
        }
        return result;
    }

    public List<double[]> getPoints() {
        return new ArrayList<>(points);
    }

    public List<Double> getValues() {
        return new ArrayList<>(values);
    }
}