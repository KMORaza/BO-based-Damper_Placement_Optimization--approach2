package solution.damper_placement.source;

import java.util.List;

public class Main {
    public static void main(String[] args) {
        double[][] bounds = {
            {2000, 8000}, {2000, 8000}, {2000, 8000}, {2000, 8000},
            {0.5, 1.5}, {2e5, 8e5}, {-0.3, 0.3}, {-0.3, 0.3}, {0.2, 0.8},
            {-0.3, 0.3}, {-0.3, 0.3}, {0.2, 0.8}, {0, Math.PI / 6}, {0, Math.PI / 6},
            {25000, 40000}, {25000, 40000}, {-0.03, 0.03}, {0.1, 0.4}
        };
        String[] dampingProfiles = {"linear", "digressive", "progressive"};
        VehicleParameters vehicle = new VehicleParameters();
        ObjectiveFunction objective = new ObjectiveFunction(vehicle);
        BayesianOptimizer optimizer = new BayesianOptimizer(objective, bounds, dampingProfiles, 100);

        List<double[]> paretoSolutions = optimizer.optimize();
        System.out.println("Pareto Front Solutions:");
        for (int i = 0; i < paretoSolutions.size(); i++) {
            double[] params = paretoSolutions.get(i);
            int profileIndex = (int) params[params.length - 1];
            String profile = optimizer.getProfileByIndex(profileIndex);
            double[] evalParams = new double[params.length - 1];
            System.arraycopy(params, 0, evalParams, 0, evalParams.length);
            double[] objectives = objective.evaluate(evalParams, profile);

            System.out.printf("Solution %d:%n", i + 1);
            System.out.printf("  Comfort: %.4f, Vibration: %.4f, Handling: %.4f%n", objectives[0], objectives[1], objectives[2]);
            System.out.printf("  Front Compression Damping: %.2f Ns/m%n", params[0]);
            System.out.printf("  Front Rebound Damping: %.2f Ns/m%n", params[1]);
            System.out.printf("  Rear Compression Damping: %.2f Ns/m%n", params[2]);
            System.out.printf("  Rear Rebound Damping: %.2f Ns/m%n", params[3]);
            System.out.printf("  Blow-off Threshold: %.2f m/s%n", params[4]);
            System.out.printf("  Gas Pressure: %.2f Pa%n", params[5]);
            System.out.printf("  Front Mounting X: %.2f m%n", params[6]);
            System.out.printf("  Front Mounting Y: %.2f m%n", params[7]);
            System.out.printf("  Front Mounting Z: %.2f m%n", params[8]);
            System.out.printf("  Rear Mounting X: %.2f m%n", params[9]);
            System.out.printf("  Rear Mounting Y: %.2f m%n", params[10]);
            System.out.printf("  Rear Mounting Z: %.2f m%n", params[11]);
            System.out.printf("  Front Inclination Angle: %.2f rad%n", params[12]);
            System.out.printf("  Rear Inclination Angle: %.2f rad%n", params[13]);
            System.out.printf("  Front Spring Stiffness: %.2f N/m%n", params[14]);
            System.out.printf("  Rear Spring Stiffness: %.2f N/m%n", params[15]);
            System.out.printf("  Camber Angle: %.4f rad%n", params[16]);
            System.out.printf("  Anti-Dive Ratio: %.2f%n", params[17]);
            System.out.printf("  Damping Profile: %s%n", profile);
        }
    }
}