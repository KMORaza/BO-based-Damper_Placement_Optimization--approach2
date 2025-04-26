package solution.damper_placement.source;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.stream.Collectors;

public class BayesianOptimizer {
    private final ObjectiveFunction objective;
    private final SurrogateEnsemble[] ensembles; 
    private final double[][] bounds;
    private final String[] dampingProfiles;
    private final Random random;
    private final int maxIterations;
    private final ParetoFront paretoFront;
    private final ExecutorService executor;
    private final double[] referencePoint = {100.0, 1e6, 100.0}; 
    private int invalidObjectiveCount = 0; 
    private final int numCandidates = 100; 

    public BayesianOptimizer(ObjectiveFunction objective, double[][] bounds, String[] dampingProfiles, int maxIterations) {
        this.objective = objective;
        this.ensembles = new SurrogateEnsemble[3]; // Comfort, Vibration, Handling
        for (int i = 0; i < 3; i++) {
            ensembles[i] = new SurrogateEnsemble(bounds.length);
        }
        this.bounds = bounds;
        this.dampingProfiles = dampingProfiles;
        this.random = new Random();
        this.maxIterations = maxIterations;
        this.paretoFront = new ParetoFront(100); 
        this.executor = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    }

    public List<double[]> optimize() {
        List<double[]> initialParams = new ArrayList<>();
        List<String> initialProfiles = new ArrayList<>();
        for (int i = 0; i < 30; i++) {
            initialParams.add(randomPoint());
            initialProfiles.add(randomProfile());
        }
        List<double[]> initialObjectives = evaluateInParallel(initialParams, initialProfiles);
        for (int i = 0; i < initialParams.size(); i++) {
            double[] objectives = initialObjectives.get(i);
            if (isValidObjectives(objectives)) {
                for (int j = 0; j < 3; j++) {
                    ensembles[j].addPoint(initialParams.get(i), objectives[j]);
                }
                paretoFront.addSolution(initialParams.get(i), initialProfiles.get(i), objectives);
                System.out.printf("Initial point %d: Comfort=%.4f, Vibration=%.4f, Handling=%.4f, Profile=%s%n",
                        i, objectives[0], objectives[1], objectives[2], initialProfiles.get(i));
            } else {
                invalidObjectiveCount++;
                System.err.printf("Error: Invalid objectives at initial point %d: %s%n", i, paramsToString(initialParams.get(i)));
            }
        }

        // Bayesian Optimization loop
        SimulatedAnnealing sa = new SimulatedAnnealing(bounds, this::computeActiveLearningScore, random);
        for (int iter = 0; iter < maxIterations; iter++) {
            double[] weights = getAdaptiveWeights(iter); // Adaptive weights based on diversity
            double alpha = 0.5 * (1.0 - (double) iter / maxIterations); // Active learning weight
            double[] nextParams = sa.optimize(new double[]{weights[0], weights[1], weights[2], alpha});
            String nextProfile = randomProfile();
            double[] objectives = objective.evaluate(nextParams, nextProfile);
            if (isValidObjectives(objectives)) {
                for (int j = 0; j < 3; j++) {
                    ensembles[j].addPoint(nextParams, objectives[j]);
                }
                paretoFront.addSolution(nextParams, nextProfile, objectives);
                System.out.printf("Iteration %d: Comfort=%.4f, Vibration=%.4f, Handling=%.4f, Profile=%s%n",
                        iter + 1, objectives[0], objectives[1], objectives[2], nextProfile);
            } else {
                invalidObjectiveCount++;
                System.err.printf("Error: Invalid objectives at iteration %d: %s%n", iter, paramsToString(nextParams));
            }
        }

        System.out.printf("Total invalid objectives: %d%n", invalidObjectiveCount);
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
        int batchSize = Math.max(1, paramsList.size() / Runtime.getRuntime().availableProcessors()); // Optimize batch size
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

    private double computeActiveLearningScore(double[] x, double[] args) {
        double[] weights = new double[]{args[0], args[1], args[2]};
        double alpha = args[3]; // Active learning weight
        /// Compute EHVI
        double ehvi = computeMultiObjectiveEI(x, weights);
        /// Compute uncertainty (average variance across objectives)
        double variance = 0.0;
        for (int i = 0; i < 3; i++) {
            variance += ensembles[i].predictVariance(x);
        }
        variance /= 3.0;
        /// Compute diversity (minimum distance to Pareto front points)
        double diversity = Double.MAX_VALUE;
        List<ParetoFront.Solution> front = paretoFront.getSolutions();
        for (ParetoFront.Solution sol : front) {
            double dist = 0.0;
            for (int i = 0; i < x.length; i++) {
                double diff = x[i] - sol.params[i];
                dist += diff * diff;
            }
            diversity = Math.min(diversity, Math.sqrt(dist));
        }
        diversity = diversity == Double.MAX_VALUE ? 1.0 : diversity;
        /// Normalize variance and diversity
        double maxVariance = 1.0; // Approximate max variance based on ensemble
        double maxDiversity = Math.sqrt(bounds.length * 100.0); // Approximate max distance
        double normVariance = Math.min(variance / maxVariance, 1.0);
        double normDiversity = Math.min(diversity / maxDiversity, 1.0);
        /// Combine scores
        double score = alpha * (normVariance + normDiversity) + (1.0 - alpha) * ehvi;
        return score > 0 ? score : 1e-10; 
    }

    private double computeMultiObjectiveEI(double[] x, double[] weights) {
        /// Compute Expected Hypervolume Improvement (EHVI)
        double[] mean = new double[3];
        double[] std = new double[3];
        double[] best = new double[3];
        double exploration = 0.0;
        /// Cache ensemble predictions
        for (int i = 0; i < 3; i++) {
            mean[i] = ensembles[i].predict(x);
            double variance = ensembles[i].predictVariance(x);
            std[i] = Math.sqrt(Math.max(variance, 1e-10));
            exploration += std[i]; 
            List<Double> values = new ArrayList<>();
            for (ParetoFront.Solution point : paretoFront.getSolutions()) {
                values.add(point.objectives[i]);
            }
            best[i] = values.stream().mapToDouble(Double::doubleValue).min().orElse(0.0);
        }
        /// Normalize objectives for hypervolume calculation
        double[] normalizedMean = normalizeObjectives(mean);
        double currentHypervolume = paretoFront.computeHypervolume(referencePoint);
        /// Approximate EHVI using Monte Carlo sampling
        int samples = 100; // Optimized from previous response
        double ehvi = 0.0;
        for (int s = 0; s < samples; s++) {
            double[] sample = new double[3];
            for (int i = 0; i < 3; i++) {
                sample[i] = mean[i] + std[i] * random.nextGaussian();
                sample[i] = Math.max(best[i], sample[i]); 
            }
            /// Compute hypervolume contribution of the sample
            double[] normalizedSample = normalizeObjectives(sample);
            double sampleHypervolume = paretoFront.computeHypervolumeWithPoint(normalizedSample, referencePoint);
            ehvi += (sampleHypervolume - currentHypervolume) / samples;
        }
        /// Add exploration term 
        ehvi += 0.1 * exploration; 
        double weightedEhvi = ehvi;
        for (int i = 0; i < 3; i++) {
            weightedEhvi *= Math.pow(1.0 + weights[i], normalizedMean[i] / referencePoint[i]);
        }
        return weightedEhvi > 0 ? weightedEhvi : 1e-10; 
    }

    private double[] normalizeObjectives(double[] objectives) {
        double[] normalized = new double[3];
        for (int i = 0; i < 3; i++) {
            normalized[i] = objectives[i] / referencePoint[i];
        }
        return normalized;
    }

    private double[] getAdaptiveWeights(int iter) {
        /// Compute crowding distance entropy to assess Pareto front diversity
        double entropy = paretoFront.computeCrowdingEntropy();
        double[] weights = new double[3];
        /// If entropy is low (sparse front), focus on exploration
        if (entropy < 0.3 || paretoFront.getSolutions().size() < 10) {
            weights[0] = weights[1] = weights[2] = 1.0 / 3.0;
        } else {
            // Focus on underrepresented objectives based on crowding
            List<ParetoFront.Solution> front = paretoFront.getSolutions();
            List<ParetoFront.Solution> frontCopy = new ArrayList<>(front);
            double[] maxGaps = new double[3];
            for (int i = 0; i < 3; i++) {
                final int objIndex = i;
                frontCopy.sort(Comparator.comparingDouble(s -> s.objectives[objIndex]));
                double maxGap = 0.0;
                for (int j = 1; j < frontCopy.size(); j++) {
                    double gap = frontCopy.get(j).objectives[objIndex] - frontCopy.get(j - 1).objectives[objIndex];
                    maxGap = Math.max(maxGap, gap);
                }
                maxGaps[i] = maxGap;
            }
            /// Apply softmax to smooth weights
            double sumExp = 0.0;
            double[] expGaps = new double[3];
            for (int i = 0; i < 3; i++) {
                expGaps[i] = Math.exp(maxGaps[i] / (sumGaps(maxGaps) + 1e-10));
                sumExp += expGaps[i];
            }
            for (int i = 0; i < 3; i++) {
                weights[i] = sumExp > 0 ? expGaps[i] / sumExp : 1.0 / 3.0;
            }
        }
        return weights;
    }

    private double sumGaps(double[] gaps) {
        double sum = 0.0;
        for (double gap : gaps) sum += gap;
        return sum;
    }

    private boolean isValidObjectives(double[] objectives) {
        for (double obj : objectives) {
            if (Double.isNaN(obj) || Double.isInfinite(obj) || obj >= 1e12) {
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