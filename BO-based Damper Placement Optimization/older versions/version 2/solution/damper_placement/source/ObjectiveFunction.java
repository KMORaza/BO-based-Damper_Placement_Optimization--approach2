package solution.damper_placement.source;

import java.util.Random;

public class ObjectiveFunction {
    private final VehicleParameters vehicle;
    private final Random random = new Random();

    public ObjectiveFunction(VehicleParameters vehicle) {
        this.vehicle = vehicle;
    }

    public double[] evaluate(double[] params, String dampingProfile) {
        // Retry up to 3 times with slight parameter perturbation
        for (int attempt = 0; attempt < 3; attempt++) {
            double[] currentParams = attempt == 0 ? params : perturbParams(params);
            try {
                ExtendedDamperParameters damperParams = new ExtendedDamperParameters(currentParams, dampingProfile);
                FullCarSuspensionSystem system = new FullCarSuspensionSystem(damperParams, vehicle);
                double[] metrics = system.simulate();

                if (metrics[0] == Double.MAX_VALUE || metrics[1] == Double.MAX_VALUE || metrics[2] == Double.MAX_VALUE) {
                    continue; // Retry on invalid simulation
                }

                double strokePenalty = computeStrokePenalty(damperParams);
                double[] objectives = new double[3];
                objectives[0] = metrics[0] / 3.0 + strokePenalty; // Comfort
                objectives[1] = metrics[1] / 1000.0 + strokePenalty; // Vibration (adjusted normalization)
                objectives[2] = metrics[2] / 1000.0 + strokePenalty; // Handling
                return objectives;
            } catch (Exception e) {
                System.err.println("Evaluation error: " + e.getMessage());
                continue;
            }
        }
        return new double[]{1e10, 1e10, 1e10}; // Return penalty if all retries fail
    }

    private double[] perturbParams(double[] params) {
        double[] perturbed = params.clone();
        for (int i = 0; i < perturbed.length; i++) {
            double range = (i < 4 ? 8000 : i == 4 ? 1.5 : i == 5 ? 8e5 : i < 12 ? 0.3 : i < 14 ? Math.PI / 6 : i < 16 ? 40000 : i == 16 ? 0.03 : 0.4)
                    - (i < 4 ? 2000 : i == 4 ? 0.5 : i == 5 ? 2e5 : i < 12 ? -0.3 : i < 14 ? 0 : i < 16 ? 25000 : i == 16 ? -0.03 : 0.1);
            perturbed[i] += random.nextGaussian() * 0.05 * range;
            // Clamp to bounds
            perturbed[i] = clamp(perturbed[i], i < 4 ? 2000 : i == 4 ? 0.5 : i == 5 ? 2e5 : i < 12 ? -0.3 : i < 14 ? 0 : i < 16 ? 25000 : i == 16 ? -0.03 : 0.1,
                    i < 4 ? 8000 : i == 4 ? 1.5 : i == 5 ? 8e5 : i < 12 ? 0.3 : i < 14 ? Math.PI / 6 : i < 16 ? 40000 : i == 16 ? 0.03 : 0.4);
        }
        return perturbed;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double computeStrokePenalty(ExtendedDamperParameters params) {
        double[] motionRatios = params.getMotionRatios();
        double penalty = 0.0;
        for (double ratio : motionRatios) {
            if (ratio > 2.0 || ratio < 0.5) {
                penalty += 0.1;
            }
        }
        return penalty;
    }
}