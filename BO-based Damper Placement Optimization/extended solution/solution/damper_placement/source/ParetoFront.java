package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

public class ParetoFront {
    private final int maxSize;
    private final List<Solution> solutions;
    private final Random random;
    public static class Solution {
        public final double[] params;
        public final String profile;
        public final double[] objectives;
        public Solution(double[] params, String profile, double[] objectives) {
            this.params = params.clone();
            this.profile = profile;
            this.objectives = objectives.clone();
        }
    }
    public ParetoFront(int maxSize) {
        this.maxSize = maxSize;
        this.solutions = new ArrayList<>();
        this.random = new Random();
    }
    public synchronized void addSolution(double[] params, String profile, double[] objectives) {
        Solution newSolution = new Solution(params, profile, objectives);
        List<Solution> newFront = new ArrayList<>();
        boolean isDominated = false;
        for (Solution existing : solutions) {
            if (dominates(existing.objectives, newSolution.objectives)) {
                isDominated = true;
            } else if (!dominates(newSolution.objectives, existing.objectives)) {
                newFront.add(existing);
            }
        }
        if (!isDominated) {
            newFront.add(newSolution);
        }
        if (newFront.size() > maxSize) {
            List<Solution> frontCopy = new ArrayList<>(newFront);
            frontCopy.sort(Comparator.comparingDouble(s -> -computeCrowdingDistance(s, frontCopy))); // Descending
            while (newFront.size() > maxSize) {
                newFront.remove(newFront.size() - 1);
            }
        }
        solutions.clear();
        solutions.addAll(newFront);
    }
    private boolean dominates(double[] obj1, double[] obj2) {
        boolean atLeastOneBetter = false;
        for (int i = 0; i < obj1.length; i++) {
            if (obj1[i] > obj2[i]) {
                return false;
            }
            if (obj1[i] < obj2[i]) {
                atLeastOneBetter = true;
            }
        }
        return atLeastOneBetter;
    }
    private double computeCrowdingDistance(Solution solution, List<Solution> front) {
        List<Solution> frontCopy = new ArrayList<>(front);
        double distance = 0.0;
        double[] ranges = {100.0, 1e6, 100.0}; // Objective ranges for normalization
        for (int i = 0; i < 3; i++) {
            final int objIndex = i;
            frontCopy.sort(Comparator.comparingDouble(s -> s.objectives[objIndex]));
            int index = frontCopy.indexOf(solution);
            if (index == 0 || index == frontCopy.size() - 1) {
                distance += Double.MAX_VALUE; // Infinite distance for boundary points
            } else {
                double prev = frontCopy.get(index - 1).objectives[objIndex];
                double next = frontCopy.get(index + 1).objectives[objIndex];
                distance += (next - prev) / ranges[i]; // Normalize by objective range
            }
        }
        return distance;
    }
    public double computeHypervolume(double[] referencePoint) {
        if (solutions.isEmpty()) return 0.0;
        List<double[]> normalized = new ArrayList<>();
        synchronized (solutions) {
            List<Solution> sampled = solutions.size() > 50 ? subsample(solutions, 50) : solutions;
            for (Solution s : sampled) {
                double[] norm = new double[3];
                for (int i = 0; i < 3; i++) {
                    norm[i] = Math.min(s.objectives[i] / referencePoint[i] + 1e-6, 1.0);
                }
                normalized.add(norm);
            }
        }
        normalized.sort(Comparator.comparingDouble(a -> a[0]));
        double volume = 0.0;
        double prevX = 1.0;
        for (double[] point : normalized) {
            double x = point[0];
            double yMax = 1.0;
            double zMax = 1.0;
            for (double[] other : normalized) {
                if (other[0] >= x) {
                    yMax = Math.min(yMax, other[1]);
                    zMax = Math.min(zMax, other[2]);
                }
            }
            volume += (prevX - x) * (1.0 - yMax) * (1.0 - zMax);
            prevX = x;
        }
        return volume * referencePoint[0] * referencePoint[1] * referencePoint[2];
    }
    public double computeHypervolumeWithPoint(double[] point, double[] referencePoint) {
        List<double[]> tempFront = new ArrayList<>();
        synchronized (solutions) {
            List<Solution> sampled = solutions.size() > 50 ? subsample(solutions, 50) : solutions;
            for (Solution s : sampled) {
                double[] norm = new double[3];
                for (int i = 0; i < 3; i++) {
                    norm[i] = Math.min(s.objectives[i] / referencePoint[i] + 1e-6, 1.0);
                }
                tempFront.add(norm);
            }
        }
        double[] normPoint = new double[3];
        for (int i = 0; i < 3; i++) {
            normPoint[i] = Math.min(point[i] + 1e-6, 1.0); 
        }
        tempFront.add(normPoint);
        tempFront.sort(Comparator.comparingDouble(a -> a[0]));
        double volume = 0.0;
        double prevX = 1.0;
        for (double[] p : tempFront) {
            double x = p[0];
            double yMax = 1.0;
            double zMax = 1.0;
            for (double[] other : tempFront) {
                if (other[0] >= x) {
                    yMax = Math.min(yMax, other[1]);
                    zMax = Math.min(zMax, other[2]);
                }
            }
            volume += (prevX - x) * (1.0 - yMax) * (1.0 - zMax);
            prevX = x;
        }
        return volume * referencePoint[0] * referencePoint[1] * referencePoint[2];
    }
    private List<Solution> subsample(List<Solution> solutions, int sampleSize) {
        List<Solution> sampled = new ArrayList<>();
        if (solutions.size() <= sampleSize) return new ArrayList<>(solutions);
        for (int i = 0; i < sampleSize; i++) {
            sampled.add(solutions.get(random.nextInt(solutions.size())));
        }
        return sampled;
    }
    public double computeCrowdingEntropy() {
        synchronized (solutions) {
            if (solutions.size() < 2) return 0.0;
            List<Solution> solutionsCopy = new ArrayList<>(solutions);
            List<Double> distances = new ArrayList<>();
            double sumDist = 0.0;
            for (Solution s : solutionsCopy) {
                double dist = computeCrowdingDistance(s, solutionsCopy);
                if (!Double.isInfinite(dist)) {
                    distances.add(dist);
                    sumDist += dist;
                }
            }
            if (distances.isEmpty()) return 0.0;
            double entropy = 0.0;
            for (double dist : distances) {
                double prob = dist / sumDist;
                if (prob > 0) {
                    entropy -= prob * Math.log(prob) / Math.log(2.0);
                }
            }
            double maxEntropy = Math.log(distances.size()) / Math.log(2.0);
            return maxEntropy > 0 ? entropy / maxEntropy : 0.0;
        }
    }
    public synchronized List<Solution> getSolutions() {
        return new ArrayList<>(solutions);
    }
}