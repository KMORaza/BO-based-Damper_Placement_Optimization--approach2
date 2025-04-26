package solution.damper_placement.source;

public class ObjectiveFunction {
    private final double wComfort = 0.35;
    private final double wVibration = 0.35;
    private final double wHandling = 0.30;
    private final VehicleParameters vehicle;

    public ObjectiveFunction(VehicleParameters vehicle) {
        this.vehicle = vehicle;
    }

    public double evaluate(double[] params, String dampingProfile) {
        ExtendedDamperParameters damperParams = new ExtendedDamperParameters(params, dampingProfile);
        FullCarSuspensionSystem system = new FullCarSuspensionSystem(damperParams, vehicle);
        double[] metrics = system.simulate();

        // Check for penalty metrics
        if (metrics[0] == Double.MAX_VALUE || metrics[1] == Double.MAX_VALUE || metrics[2] == Double.MAX_VALUE) {
            return 1e10; // High penalty for invalid simulation
        }

        // Normalized metrics
        double comfort = metrics[0] / 3.0;
        double vibration = metrics[1] / 0.005;
        double handling = metrics[2] / 1000.0;

        // Add penalty for excessive damper stroke
        double strokePenalty = computeStrokePenalty(damperParams);
        double objective = wComfort * comfort + wVibration * vibration + wHandling * handling + strokePenalty;

        return objective;
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