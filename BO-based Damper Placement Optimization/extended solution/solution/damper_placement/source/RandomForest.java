package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RandomForest {
    private final List<DecisionTree> trees;
    private final Random random;
    private final int numTrees = 5; 
    private final int maxDepth = 8; 
    private final int minNodeSize = 5;
    private final int numFeatures;
    public RandomForest(int numFeatures) {
        this.trees = new ArrayList<>();
        this.random = new Random();
        this.numFeatures = numFeatures;
        for (int i = 0; i < numTrees; i++) {
            trees.add(new DecisionTree());
        }
    }
    public void addPoint(double[] x, double y) {
        for (DecisionTree tree : trees) {
            List<double[]> sampleX = new ArrayList<>();
            List<Double> sampleY = new ArrayList<>();
            for (int i = 0; i < x.length; i++) {
                int idx = random.nextInt(x.length);
                sampleX.add(x);
                sampleY.add(y);
            }
            tree.train(sampleX, sampleY);
        }
    }
    public double predict(double[] x) {
        double sum = 0.0;
        for (DecisionTree tree : trees) {
            sum += tree.predict(x);
        }
        return sum / numTrees;
    }
    public double predictVariance(double[] x) {
        double[] predictions = new double[numTrees];
        double mean = 0.0;
        for (int i = 0; i < numTrees; i++) {
            predictions[i] = trees.get(i).predict(x); 
            mean += predictions[i];
        }
        mean /= numTrees;
        double variance = 0.0;
        for (double pred : predictions) {
            variance += (pred - mean) * (pred - mean);
        }
        return variance / (numTrees - 1);
    }
    private class DecisionTree {
        private Node root;

        public void train(List<double[]> X, List<Double> y) {
            root = buildTree(X, y, 0);
        }
        private Node buildTree(List<double[]> X, List<Double> y, int depth) {
            if (X.size() < minNodeSize || depth >= maxDepth) {
                return new Node(average(y));
            }
            int[] features = randomFeatures();
            double bestSplit = 0.0;
            int bestFeature = -1;
            double bestVariance = variance(y);
            List<double[]> leftX = new ArrayList<>();
            List<Double> leftY = new ArrayList<>();
            List<double[]> rightX = new ArrayList<>();
            List<Double> rightY = new ArrayList<>();
            for (int feature : features) {
                List<Double> values = new ArrayList<>();
                for (double[] x : X) values.add(x[feature]);
                values.sort(Double::compareTo);
                double split = values.get(values.size() / 2);
                leftX.clear();
                leftY.clear();
                rightX.clear();
                rightY.clear();
                for (int i = 0; i < X.size(); i++) {
                    if (X.get(i)[feature] <= split) {
                        leftX.add(X.get(i));
                        leftY.add(y.get(i));
                    } else {
                        rightX.add(X.get(i));
                        rightY.add(y.get(i));
                    }
                }
                if (leftY.isEmpty() || rightY.isEmpty()) continue;
                double varLeft = variance(leftY) * leftY.size();
                double varRight = variance(rightY) * rightY.size();
                double varReduction = bestVariance - (varLeft + varRight) / X.size();
                if (varReduction > bestVariance) {
                    bestVariance = varReduction;
                    bestFeature = feature;
                    bestSplit = split;
                }
            }
            if (bestFeature == -1) {
                return new Node(average(y));
            }
            Node node = new Node(bestFeature, bestSplit);
            node.left = buildTree(leftX, leftY, depth + 1);
            node.right = buildTree(rightX, rightY, depth + 1);
            return node;
        }
        public double predict(double[] x) {
            return predictNode(root, x);
        }
        private double predictNode(Node node, double[] x) {
            if (node.isLeaf) return node.value;
            if (x[node.feature] <= node.split) {
                return predictNode(node.left, x);
            } else {
                return predictNode(node.right, x);
            }
        }
        private int[] randomFeatures() {
            int sqrtFeatures = (int) Math.sqrt(numFeatures);
            int[] features = new int[numFeatures];
            for (int i = 0; i < numFeatures; i++) features[i] = i;
            for (int i = 0; i < sqrtFeatures; i++) {
                int j = i + random.nextInt(numFeatures - i);
                int temp = features[i];
                features[i] = features[j];
                features[j] = temp;
            }
            int[] selected = new int[sqrtFeatures];
            System.arraycopy(features, 0, selected, 0, sqrtFeatures);
            return selected;
        }
        private double variance(List<Double> y) {
            if (y.isEmpty()) return 0.0;
            double mean = average(y);
            double sum = 0.0;
            for (double v : y) sum += (v - mean) * (v - mean);
            return sum / y.size();
        }
        private double average(List<Double> y) {
            double sum = 0.0;
            for (double v : y) sum += v;
            return y.isEmpty() ? 0.0 : sum / y.size();
        }
    }
    private static class Node {
        int feature;
        double split;
        double value;
        Node left, right;
        boolean isLeaf;
        Node(double value) {
            this.value = value;
            this.isLeaf = true;
        }
        Node(int feature, double split) {
            this.feature = feature;
            this.split = split;
            this.isLeaf = false;
        }
    }
}