package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class NeuralNetwork {
    private final int inputSize;
    private final int hiddenSize = 32; 
    private final int outputSize = 1;
    private double[][] w1, w2, w3; 
    private double[] b1, b2, b3; 
    private final Random random;
    private final double learningRate = 0.01;
    private List<double[]> X;
    private List<Double> y;

    public NeuralNetwork(int inputSize) {
        this.inputSize = inputSize;
        this.random = new Random();
        this.X = new ArrayList<>();
        this.y = new ArrayList<>();
        initializeWeights();
    }

    private void initializeWeights() {
        w1 = new double[inputSize][hiddenSize];
        w2 = new double[hiddenSize][hiddenSize];
        w3 = new double[hiddenSize][outputSize];
        b1 = new double[hiddenSize];
        b2 = new double[hiddenSize];
        b3 = new double[outputSize];
        for (int i = 0; i < inputSize; i++)
            for (int j = 0; j < hiddenSize; j++)
                w1[i][j] = random.nextGaussian() * 0.01;
        for (int i = 0; i < hiddenSize; i++)
            for (int j = 0; j < hiddenSize; j++)
                w2[i][j] = random.nextGaussian() * 0.01;
        for (int i = 0; i < hiddenSize; i++)
            for (int j = 0; j < outputSize; j++)
                w3[i][j] = random.nextGaussian() * 0.01;
        for (int i = 0; i < hiddenSize; i++) {
            b1[i] = b2[i] = 0.0;
        }
        b3[0] = 0.0;
    }

    public void addPoint(double[] x, double y) {
        X.add(x.clone());
        this.y.add(y);
        train();
    }

    private void train() {
        if (X.isEmpty()) return;
        for (int epoch = 0; epoch < 50; epoch++) {
            for (int i = 0; i < X.size(); i++) {
                double[] x = X.get(i);
                double target = y.get(i);
                /// Forward pass
                double[] h1 = matrixMultiply(x, w1, b1, true);
                double[] h2 = matrixMultiply(h1, w2, b2, true);
                double[] out = matrixMultiply(h2, w3, b3, false);
                double error = target - out[0];
                /// Backward pass
                double[] deltaOut = new double[]{error};
                double[] deltaH2 = backpropagate(deltaOut, w3, h2, true);
                double[] deltaH1 = backpropagate(deltaH2, w2, h1, true);
                /// Update weights and biases
                updateWeights(w3, h2, deltaOut);
                updateWeights(w2, h1, deltaH2);
                updateWeights(w1, x, deltaH1);
                for (int j = 0; j < b3.length; j++) b3[j] += learningRate * deltaOut[j];
                for (int j = 0; j < b2.length; j++) b2[j] += learningRate * deltaH2[j];
                for (int j = 0; j < b1.length; j++) b1[j] += learningRate * deltaH1[j];
            }
        }
    }

    public double predict(double[] x) {
        double[] h1 = matrixMultiply(x, w1, b1, true);
        double[] h2 = matrixMultiply(h1, w2, b2, true);
        double[] out = matrixMultiply(h2, w3, b3, false);
        return out[0];
    }

    public double predictVariance(double[] x) {
        /// Monte Carlo dropout-like sampling
        int samples = 5;
        double[] predictions = new double[samples];
        for (int s = 0; s < samples; s++) {
            double[] h1 = matrixMultiply(x, w1, b1, true, 0.2);
            double[] h2 = matrixMultiply(h1, w2, b2, true, 0.2);
            double[] out = matrixMultiply(h2, w3, b3, false, 0.2);
            predictions[s] = out[0];
        }
        double mean = 0.0;
        for (double p : predictions) mean += p;
        mean /= samples;
        double variance = 0.0;
        for (double p : predictions) variance += (p - mean) * (p - mean);
        return variance / (samples - 1);
    }

    private double[] matrixMultiply(double[] input, double[][] weights, double[] bias, boolean relu) {
        return matrixMultiply(input, weights, bias, relu, 0.0);
    }

    private double[] matrixMultiply(double[] input, double[][] weights, double[] bias, boolean relu, double dropout) {
        double[] output = new double[weights[0].length];
        for (int j = 0; j < output.length; j++) {
            if (dropout > 0 && random.nextDouble() < dropout) continue;
            double sum = bias[j];
            for (int i = 0; i < input.length; i++) {
                sum += input[i] * weights[i][j];
            }
            output[j] = relu ? Math.max(0, sum) : sum;
        }
        return output;
    }

    private double[] backpropagate(double[] delta, double[][] weights, double[] input, boolean relu) {
        double[] newDelta = new double[weights.length];
        for (int i = 0; i < newDelta.length; i++) {
            double sum = 0.0;
            for (int j = 0; j < delta.length; j++) {
                sum += delta[j] * weights[i][j];
            }
            newDelta[i] = relu && input[i] <= 0 ? 0.0 : sum;
        }
        return newDelta;
    }

    private void updateWeights(double[][] weights, double[] input, double[] delta) {
        for (int i = 0; i < input.length; i++) {
            for (int j = 0; j < delta.length; j++) {
                weights[i][j] += learningRate * delta[j] * input[i];
            }
        }
    }
}