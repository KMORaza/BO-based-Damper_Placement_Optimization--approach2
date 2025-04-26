package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class GaussianProcess {
    private final List<double[]> points;
    private final List<Double> values;
    private double lengthScale;
    private double sigmaF;
    private final double sigmaN;
    private double[][] K_inv; // Cached inverse kernel matrix
    private final Random random;
    private final Object lock = new Object();

    public GaussianProcess() {
        this.points = new ArrayList<>();
        this.values = new ArrayList<>();
        this.lengthScale = 1.0;
        this.sigmaF = 1.0;
        this.sigmaN = 0.01;
        this.random = new Random();
        this.K_inv = null;
    }

    public void addPoint(double[] x, double y) {
        synchronized (lock) {
            points.add(x);
            values.add(y);
            K_inv = null; // Invalidate cache
            if (points.size() % 10 == 0) {
                optimizeHyperparameters();
            }
        }
    }

    public double predict(double[] x) {
        synchronized (lock) {
            if (points.isEmpty()) return 0.0;
            double[] k = computeKernelVector(x);
            double[] weights = multiplyMatrixVector(K_inv(), k);
            double mean = 0.0;
            for (int i = 0; i < weights.length; i++) {
                mean += weights[i] * values.get(i);
            }
            return mean;
        }
    }

    public double predictVariance(double[] x) {
        synchronized (lock) {
            if (points.isEmpty()) return sigmaF * sigmaF;
            double[] k = computeKernelVector(x);
            double kStarStar = kernel(x, x);
            double[] weights = multiplyMatrixVector(K_inv(), k);
            double variance = kStarStar;
            for (int i = 0; i < weights.length; i++) {
                variance -= weights[i] * k[i];
            }
            return Math.max(variance, 0.0);
        }
    }

    public List<Double> getValues() {
        synchronized (lock) {
            return new ArrayList<>(values);
        }
    }

    private double[] computeKernelVector(double[] x) {
        double[] k = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            k[i] = kernel(x, points.get(i));
        }
        return k;
    }

    private double kernel(double[] x1, double[] x2) {
        double sum = 0.0;
        for (int i = 0; i < x1.length; i++) {
            sum += Math.pow((x1[i] - x2[i]) / lengthScale, 2);
        }
        return sigmaF * sigmaF * Math.exp(-0.5 * sum);
    }

    private double[][] computeKernelMatrix() {
        int n = points.size();
        double[][] K = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                K[i][j] = kernel(points.get(i), points.get(j));
                if (i == j) K[i][j] += sigmaN * sigmaN;
            }
        }
        return K;
    }

    private double[][] K_inv() {
        if (K_inv == null) {
            double[][] K = computeKernelMatrix();
            K_inv = choleskyInverse(K);
        }
        return K_inv;
    }

    private double[][] choleskyInverse(double[][] A) {
        int n = A.length;
        double[][] L = new double[n][n];
        // Cholesky decomposition: A = L * L^T
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = 0.0;
                for (int k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k];
                }
                if (i == j) {
                    L[i][j] = Math.sqrt(Math.max(A[i][j] - sum, 1e-10));
                } else {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }
        // Solve L * y = I and L^T * x = y for inverse
        double[][] inv = new double[n][n];
        for (int col = 0; col < n; col++) {
            double[] y = new double[n];
            double[] x = new double[n];
            // Forward: L * y = e_col
            for (int i = 0; i < n; i++) {
                double sum = 0.0;
                for (int k = 0; k < i; k++) {
                    sum += L[i][k] * y[k];
                }
                y[i] = (col == i ? 1.0 : 0.0) - sum;
                y[i] /= L[i][i];
            }
            // Backward: L^T * x = y
            for (int i = n - 1; i >= 0; i--) {
                double sum = 0.0;
                for (int k = i + 1; k < n; k++) {
                    sum += L[k][i] * x[k];
                }
                x[i] = (y[i] - sum) / L[i][i];
            }
            for (int i = 0; i < n; i++) {
                inv[i][col] = x[i];
            }
        }
        return inv;
    }

    private double[] multiplyMatrixVector(double[][] A, double[] b) {
        int n = A.length;
        double[] result = new double[n];
        for (int i = 0; i < n; i++) {
            double sum = 0.0;
            for (int j = 0; j < n; j++) {
                sum += A[i][j] * b[j];
            }
            result[i] = sum;
        }
        return result;
    }

    private void optimizeHyperparameters() {
        double bestLogLikelihood = Double.NEGATIVE_INFINITY;
        double bestLengthScale = lengthScale;
        double bestSigmaF = sigmaF;
        double[] lengthScales = {0.5, 1.0, 2.0};
        double[] sigmaFs = {0.5, 1.0, 2.0};
        for (double ls : lengthScales) {
            for (double sf : sigmaFs) {
                this.lengthScale = ls;
                this.sigmaF = sf;
                K_inv = null;
                double ll = computeLogMarginalLikelihood();
                if (ll > bestLogLikelihood) {
                    bestLogLikelihood = ll;
                    bestLengthScale = ls;
                    bestSigmaF = sf;
                }
            }
        }
        this.lengthScale = bestLengthScale;
        this.sigmaF = bestSigmaF;
        K_inv = null;
    }

    private double computeLogMarginalLikelihood() {
        double[][] K = computeKernelMatrix();
        double[] y = values.stream().mapToDouble(Double::doubleValue).toArray();
        double[] alpha = multiplyMatrixVector(K_inv(), y);
        double dataFit = 0.0;
        for (int i = 0; i < y.length; i++) {
            dataFit += y[i] * alpha[i];
        }
        double logDet = 0.0;
        double[][] L = choleskyDecompose(K);
        for (int i = 0; i < L.length; i++) {
            logDet += Math.log(L[i][i]);
        }
        return -0.5 * dataFit - logDet - 0.5 * y.length * Math.log(2 * Math.PI);
    }

    private double[][] choleskyDecompose(double[][] A) {
        int n = A.length;
        double[][] L = new double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j <= i; j++) {
                double sum = 0.0;
                for (int k = 0; k < j; k++) {
                    sum += L[i][k] * L[j][k];
                }
                if (i == j) {
                    L[i][j] = Math.sqrt(Math.max(A[i][j] - sum, 1e-10));
                } else {
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                }
            }
        }
        return L;
    }
}