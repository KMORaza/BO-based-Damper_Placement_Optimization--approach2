package solution.damper_placement.source;

import java.util.Random;

public class BayesianOptimizer {
    private final ObjectiveFunction objective;
    private final GaussianProcess gp;
    private final double[][] bounds;
    private final String[] dampingProfiles;
    private final Random random;
    private final int maxIterations;

    public BayesianOptimizer(ObjectiveFunction objective, double[][] bounds, String[] dampingProfiles, int maxIterations) {
        this.objective = objective;
        this.gp = new GaussianProcess();
        this.bounds = bounds;
        this.dampingProfiles = dampingProfiles;
        this.random = new Random();
        this.maxIterations = maxIterations;
    }

    public double[] optimize() {
        double[] bestParams = null;
        String bestProfile = null;
        double bestValue = Double.POSITIVE_INFINITY;

        // Initial random points (increased to 20)
        for (int i = 0; i < 20; i++) {
            double[] params = randomPoint();
            String profile = randomProfile();
            if (params == null) {
                System.err.println("Error: randomPoint() returned null at initial point " + i);
                continue;
            }
            double value = objective.evaluate(params, profile);
            if (Double.isNaN(value) || Double.isInfinite(value)) {
                System.err.println("Error: Invalid objective value (NaN or Infinity) at initial point " + i);
                continue;
            }
            System.out.printf("Initial point %d: Objective = %.4f, Profile = %s%n", i, value, profile);
            gp.addPoint(params, value);
            if (value < bestValue) {
                bestValue = value;
                bestParams = params.clone();
                bestProfile = profile;
            }
        }

        if (bestParams == null) {
            throw new RuntimeException("Failed to initialize with valid parameters");
        }

        // Bayesian Optimization loop
        for (int iter = 0; iter < maxIterations; iter++) {
            double[] nextParams = findNextPoint();
            String nextProfile = randomProfile();
            if (nextParams == null) {
                System.err.println("Error: findNextPoint() returned null at iteration " + iter);
                continue;
            }
            double value = objective.evaluate(nextParams, nextProfile);
            if (Double.isNaN(value) || Double.isInfinite(value)) {
                System.err.println("Error: Invalid objective value (NaN or Infinity) at iteration " + iter);
                continue;
            }
            gp.addPoint(nextParams, value);
            if (value < bestValue) {
                bestValue = value;
                bestParams = nextParams.clone();
                bestProfile = nextProfile;
                System.out.printf("Iteration %d: Best Objective = %.4f, Profile = %s%n", iter + 1, bestValue, bestProfile);
            }
        }

        // Store profile in the last element of bestParams
        double[] result = new double[bestParams.length + 1];
        System.arraycopy(bestParams, 0, result, 0, bestParams.length);
        result[bestParams.length] = indexOfProfile(bestProfile);
        return result;
    }

    private double[] randomPoint() {
        double[] point = new double[bounds.length];
        for (int i = 0; i < bounds.length; i++) {
            point[i] = bounds[i][0] + (bounds[i][1] - bounds[i][0]) * random.nextDouble();
        }
        return point;
    }

    private String randomProfile() {
        return dampingProfiles[random.nextInt(dampingProfiles.length)];
    }

    private double[] findNextPoint() {
        double[] bestPoint = null;
        double bestEI = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < 200; i++) {
            double[] point = randomPoint();
            if (point == null) {
                System.err.println("Error: randomPoint() returned null in findNextPoint");
                continue;
            }
            double ei = expectedImprovement(point, bestObjective());
            if (ei > bestEI) {
                bestEI = ei;
                bestPoint = point.clone();
            }
        }

        return bestPoint;
    }

    private double expectedImprovement(double[] x, double best) {
        double mean = gp.predict(x);
        double variance = gp.predictVariance(x);
        double std = Math.sqrt(variance);

        if (std < 1e-6) return 0.0;

        double z = (best - mean) / std;
        return (best - mean) * cdf(z) + std * pdf(z);
    }

    private double bestObjective() {
        double best = Double.POSITIVE_INFINITY;
        for (Double value : gp.getValues()) {
            if (value < best) best = value;
        }
        return best;
    }

    private double cdf(double z) {
        return 1.0 / (1.0 + Math.exp(-z * 1.702));
    }

    private double pdf(double z) {
        return Math.exp(-0.5 * z * z) / Math.sqrt(2 * Math.PI);
    }

    private int indexOfProfile(String profile) {
        for (int i = 0; i < dampingProfiles.length; i++) {
            if (dampingProfiles[i].equals(profile)) return i;
        }
        return 0;
    }

    public String getProfileByIndex(int index) {
        return dampingProfiles[index];
    }
}