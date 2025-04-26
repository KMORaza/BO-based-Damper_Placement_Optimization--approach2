package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;

public class GaussianProcess {
    private List<double[]> points;
    private List<Double> values;
    private double lengthScale;
    private double sigmaF;
    private double sigmaN;

    public GaussianProcess() {
        this.points = new ArrayList<>();
        this.values = new ArrayList<>();
        this.lengthScale = 1.0;
        this.sigmaF = 1.0;
        this.sigmaN = 0.01;
    }

    public void addPoint(double[] x, double y) {
        points.add(x);
        values.add(y);
    }

    public double predict(double[] x) {
        if (points.isEmpty()) return 0.0;

        double[] k = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            k[i] = kernel(x, points.get(i));
        }

        double[][] K = computeKernelMatrix();
        double[] weights = solveLinearSystem(K, k);

        double mean = 0.0;
        for (int i = 0; i < weights.length; i++) {
            mean += weights[i] * values.get(i);
        }
        return mean;
    }

    public double predictVariance(double[] x) {
        if (points.isEmpty()) return sigmaF * sigmaF;

        double[] k = new double[points.size()];
        for (int i = 0; i < points.size(); i++) {
            k[i] = kernel(x, points.get(i));
        }

        double[][] K = computeKernelMatrix();
        double kStarStar = kernel(x, x);
        double[] weights = solveLinearSystem(K, k);

        double variance = kStarStar;
        for (int i = 0; i < weights.length; i++) {
            variance -= weights[i] * k[i];
        }
        return Math.max(variance, 0.0);
    }

    // Added getter for values
    public List<Double> getValues() {
        return new ArrayList<>(values); // Return a copy to prevent external modification
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

    private double[] solveLinearSystem(double[][] A, double[] b) {
        // Simple Gaussian elimination for small matrices
        int n = A.length;
        double[][] augmented = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented[i][j] = A[i][j];
            }
            augmented[i][n] = b[i];
        }

        for (int i = 0; i < n; i++) {
            double pivot = augmented[i][i];
            for (int j = i; j <= n; j++) {
                augmented[i][j] /= pivot;
            }
            for (int k = 0; k < n; k++) {
                if (k != i) {
                    double factor = augmented[k][i];
                    for (int j = i; j <= n; j++) {
                        augmented[k][j] -= factor * augmented[i][j];
                    }
                }
            }
        }

        double[] x = new double[n];
        for (int i = 0; i < n; i++) {
            x[i] = augmented[i][n];
        }
        return x;
    }
}