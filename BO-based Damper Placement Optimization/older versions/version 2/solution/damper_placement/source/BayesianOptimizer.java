package solution.damper_placement.source;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.stream.Collectors;

public class BayesianOptimizer {
    private final ObjectiveFunction objective;
    private final GaussianProcess[] gps; // One GP per objective
    private final double[][] bounds;
    private final String[] dampingProfiles;
    private final Random random;
    private final int maxIterations;
    private final ParetoFront paretoFront;
    private final ExecutorService executor;

    public BayesianOptimizer(ObjectiveFunction objective, double[][] bounds, String[] dampingProfiles, int maxIterations) {
        this.objective = objective;
        this.gps = new GaussianProcess[3]; // Comfort, Vibration, Handling
        for (int i = 0; i < 3; i++) {
            gps[i] = new GaussianProcess();
        }
        this.bounds = bounds;
        this.dampingProfiles = dampingProfiles;
        this.random = new Random();
        this.maxIterations = maxIterations;
        this.paretoFront = new ParetoFront(50); // Max 50 solutions
        this.executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    }

    public List<double[]> optimize() {
        // Initial random points (20, parallelized)
        List<double[]> initialParams = new ArrayList<>();
        List<String> initialProfiles = new ArrayList<>();
        for (int i = 0; i < 20; i++) {
            initialParams.add(randomPoint());
            initialProfiles.add(randomProfile());
        }

        List<double[]> initialObjectives = evaluateInParallel(initialParams, initialProfiles);
        for (int i = 0; i < initialParams.size(); i++) {
            double[] objectives = initialObjectives.get(i);
            if (isValidObjectives(objectives)) {
                for (int j = 0; j < 3; j++) {
                    gps[j].addPoint(initialParams.get(i), objectives[j]);
                }
                paretoFront.addSolution(initialParams.get(i), initialProfiles.get(i), objectives);
                System.out.printf("Initial point %d: Comfort=%.4f, Vibration=%.4f, Handling=%.4f, Profile=%s%n",
                        i, objectives[0], objectives[1], objectives[2], initialProfiles.get(i));
            } else {
                System.err.printf("Error: Invalid objectives at initial point %d: %s%n", i, paramsToString(initialParams.get(i)));
            }
        }

        // Bayesian Optimization loop
        SimulatedAnnealing sa = new SimulatedAnnealing(bounds, this::computeMultiObjectiveEI, random);
        for (int iter = 0; iter < maxIterations; iter++) {
            double[] weights = getCycledWeights(iter); // Cycle weights for diversity
            double[] nextParams = sa.optimize(weights);
            String nextProfile = randomProfile();
            double[] objectives = objective.evaluate(nextParams, nextProfile);
            if (isValidObjectives(objectives)) {
                for (int j = 0; j < 3; j++) {
                    gps[j].addPoint(nextParams, objectives[j]);
                }
                paretoFront.addSolution(nextParams, nextProfile, objectives);
                System.out.printf("Iteration %d: Comfort=%.4f, Vibration=%.4f, Handling=%.4f, Profile=%s%n",
                        iter + 1, objectives[0], objectives[1], objectives[2], nextProfile);
            } else {
                System.err.printf("Error: Invalid objectives at iteration %d: %s%n", iter, paramsToString(nextParams));
            }
        }

        executor.shutdown();
        return paretoFront.getSolutions().stream()
                .map(s -> {
                    double[] result = new double[s.params.length + 1];
                    System.arraycopy(s.params, 0, result, 0, s.params.length);
                    result[s.params.length] = indexOfProfile(s.profile);
                    return result;
                })
                .collect(Collectors.toList());
    }

    private List<double[]> evaluateInParallel(List<double[]> paramsList, List<String> profiles) {
        List<Future<double[]>> futures = new ArrayList<>();
        for (int i = 0; i < paramsList.size(); i++) {
            final double[] params = paramsList.get(i);
            final String profile = profiles.get(i);
            futures.add(executor.submit(() -> objective.evaluate(params, profile)));
        }
        List<double[]> results = new ArrayList<>();
        for (Future<double[]> future : futures) {
            try {
                results.add(future.get());
            } catch (Exception e) {
                System.err.println("Error in parallel evaluation: " + e.getMessage());
                results.add(new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE});
            }
        }
        return results;
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

    private double computeMultiObjectiveEI(double[] x, double[] weights) {
        double eiSum = 0.0;
        for (int i = 0; i < 3; i++) {
            double mean = gps[i].predict(x);
            double variance = gps[i].predictVariance(x);
            double std = Math.sqrt(Math.max(variance, 1e-10));
            if (std < 1e-6) continue;
            double best = gps[i].getValues().stream().mapToDouble(Double::doubleValue).min().orElse(0.0);
            double z = (best - mean) / std;
            double ei = (best - mean) * cdf(z) + std * pdf(z);
            eiSum += weights[i] * ei;
        }
        return eiSum;
    }

    private double[] getCycledWeights(int iter) {
        double[] weights = new double[3];
        int cycle = iter % 3;
        if (cycle == 0) {
            weights[0] = 0.6;
            weights[1] = 0.2;
            weights[2] = 0.2;
        } else if (cycle == 1) {
            weights[0] = 0.2;
            weights[1] = 0.6;
            weights[2] = 0.2;
        } else {
            weights[0] = 0.2;
            weights[1] = 0.2;
            weights[2] = 0.6;
        }
        return weights;
    }

    private boolean isValidObjectives(double[] objectives) {
        for (double obj : objectives) {
            if (Double.isNaN(obj) || Double.isInfinite(obj) || obj >= 1e12) { // Increased threshold
                return false;
            }
        }
        return true;
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

    private String paramsToString(double[] params) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < params.length; i++) {
            sb.append(String.format("%.2f", params[i]));
            if (i < params.length - 1) sb.append(", ");
        }
        sb.append("]");
        return sb.toString();
    }
}