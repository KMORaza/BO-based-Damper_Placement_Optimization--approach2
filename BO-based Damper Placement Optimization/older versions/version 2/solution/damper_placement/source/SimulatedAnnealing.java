package solution.damper_placement.source;

import java.util.Random;
import java.util.function.BiFunction;

public class SimulatedAnnealing {
    private final double[][] bounds;
    private final BiFunction<double[], double[], Double> objective;
    private final Random random;
    private final double initialTemp = 100.0;
    private final double coolingRate = 0.95;
    private final int maxIterations = 200;
    private final double perturbationScale = 0.1; // Fraction of range for neighbor perturbation

    public SimulatedAnnealing(double[][] bounds, BiFunction<double[], double[], Double> objective, Random random) {
        this.bounds = bounds;
        this.objective = objective;
        this.random = random;
    }

    public double[] optimize(double[] weights) {
        // Initialize with a random point
        double[] current = randomPoint();
        double currentValue = objective.apply(current, weights);
        double[] best = current.clone();
        double bestValue = currentValue;

        double temp = initialTemp;
        for (int i = 0; i < maxIterations; i++) {
            // Generate neighbor
            double[] neighbor = generateNeighbor(current);
            double neighborValue = objective.apply(neighbor, weights);

            // Handle invalid values
            if (Double.isNaN(neighborValue) || Double.isInfinite(neighborValue)) {
                neighborValue = Double.NEGATIVE_INFINITY;
            }

            // Accept neighbor based on Metropolis criterion
            if (neighborValue >= currentValue) {
                current = neighbor.clone();
                currentValue = neighborValue;
                if (currentValue > bestValue) {
                    best = current.clone();
                    bestValue = currentValue;
                }
            } else {
                double acceptanceProb = Math.exp((neighborValue - currentValue) / temp);
                if (random.nextDouble() < acceptanceProb) {
                    current = neighbor.clone();
                    currentValue = neighborValue;
                }
            }

            // Cool down
            temp *= coolingRate;
        }

        return best;
    }

    private double[] randomPoint() {
        double[] point = new double[bounds.length];
        for (int i = 0; i < bounds.length; i++) {
            point[i] = bounds[i][0] + (bounds[i][1] - bounds[i][0]) * random.nextDouble();
        }
        return point;
    }

    private double[] generateNeighbor(double[] current) {
        double[] neighbor = current.clone();
        for (int i = 0; i < neighbor.length; i++) {
            double range = bounds[i][1] - bounds[i][0];
            double perturbation = random.nextGaussian() * perturbationScale * range;
            neighbor[i] += perturbation;
            // Clamp to bounds
            neighbor[i] = Math.max(bounds[i][0], Math.min(bounds[i][1], neighbor[i]));
        }
        return neighbor;
    }
}