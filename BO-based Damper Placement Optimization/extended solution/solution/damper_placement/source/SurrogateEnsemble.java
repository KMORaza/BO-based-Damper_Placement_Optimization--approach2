package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class SurrogateEnsemble {
    private final GaussianProcess gp;
    private final RandomForest rf;
    private final NeuralNetwork nn;
    private final List<double[]> X;
    private final List<Double> y;
    private double[] weights; // [gp, rf, nn]
    private final Random random;
    private int pointCount; // Track points for weight update frequency
    public SurrogateEnsemble(int inputSize) {
        this.gp = new GaussianProcess();
        this.rf = new RandomForest(inputSize);
        this.nn = new NeuralNetwork(inputSize);
        this.X = new ArrayList<>();
        this.y = new ArrayList<>();
        this.weights = new double[]{1.0 / 3, 1.0 / 3, 1.0 / 3};
        this.random = new Random();
        this.pointCount = 0;
    }
    public void addPoint(double[] x, double y) {
        X.add(x.clone());
        this.y.add(y);
        gp.addPoint(x, y);
        rf.addPoint(x, y);
        nn.addPoint(x, y);
        pointCount++;
        if (pointCount % 5 == 0) { 
            updateWeights();
        }
    }
    public double predict(double[] x) {
        double gpPred = gp.predict(x);
        double rfPred = rf.predict(x);
        double nnPred = nn.predict(x);
        return weights[0] * gpPred + weights[1] * rfPred + weights[2] * nnPred;
    }
    public double predictVariance(double[] x) {
        double gpVar = gp.predictVariance(x);
        double rfVar = rf.predictVariance(x);
        double nnVar = nn.predictVariance(x);
        return Math.max(gpVar, Math.max(rfVar, nnVar)); 
    }
    private void updateWeights() {
        if (X.size() < 5) return; 
        double[] errors = new double[3];
        for (int fold = 0; fold < 3; fold++) { 
            List<double[]> trainX = new ArrayList<>();
            List<Double> trainY = new ArrayList<>();
            List<double[]> testX = new ArrayList<>();
            List<Double> testY = new ArrayList<>();
            for (int i = 0; i < X.size(); i++) {
                if (i % 3 == fold) {
                    testX.add(X.get(i));
                    testY.add(y.get(i));
                } else {
                    trainX.add(X.get(i));
                    trainY.add(y.get(i));
                }
            }
            GaussianProcess tempGp = new GaussianProcess();
            RandomForest tempRf = new RandomForest(X.get(0).length);
            NeuralNetwork tempNn = new NeuralNetwork(X.get(0).length);
            for (int i = 0; i < trainX.size(); i++) {
                tempGp.addPoint(trainX.get(i), trainY.get(i));
                tempRf.addPoint(trainX.get(i), trainY.get(i));
                tempNn.addPoint(trainX.get(i), trainY.get(i));
            }
            for (int i = 0; i < testX.size(); i++) {
                double trueY = testY.get(i);
                errors[0] += Math.pow(tempGp.predict(testX.get(i)) - trueY, 2);
                errors[1] += Math.pow(tempRf.predict(testX.get(i)) - trueY, 2);
                errors[2] += Math.pow(tempNn.predict(testX.get(i)) - trueY, 2);
            }
        }
        double sumErrors = errors[0] + errors[1] + errors[2] + 1e-10;
        for (int i = 0; i < 3; i++) {
            weights[i] = (sumErrors - errors[i]) / (2 * sumErrors); 
        }
        double sumWeights = weights[0] + weights[1] + weights[2];
        for (int i = 0; i < 3; i++) weights[i] /= sumWeights;
    }
}