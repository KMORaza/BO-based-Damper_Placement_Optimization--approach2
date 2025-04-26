package solution.damper_placement.source;

public class Main {
    public static void main(String[] args) {
        // Define parameter bounds (tightened for stability)
        double[][] bounds = {
            {2000, 8000},  // Front compression damping (Ns/m)
            {2000, 8000},  // Front rebound damping (Ns/m)
            {2000, 8000},  // Rear compression damping (Ns/m)
            {2000, 8000},  // Rear rebound damping (Ns/m)
            {0.5, 1.5},    // Blow-off threshold (m/s)
            {2e5, 8e5},    // Gas pressure (Pa)
            {-0.3, 0.3},   // Front mounting X (m)
            {-0.3, 0.3},   // Front mounting Y (m)
            {0.2, 0.8},    // Front mounting Z (m)
            {-0.3, 0.3},   // Rear mounting X (m)
            {-0.3, 0.3},   // Rear mounting Y (m)
            {0.2, 0.8},    // Rear mounting Z (m)
            {0, Math.PI / 6}, // Front inclination angle (rad)
            {0, Math.PI / 6}, // Rear inclination angle (rad)
            {25000, 40000}, // Front spring stiffness (N/m)
            {25000, 40000}, // Rear spring stiffness (N/m)
            {-0.03, 0.03}, // Camber angle (rad)
            {0.1, 0.4}     // Anti-dive ratio
        };

        String[] dampingProfiles = {"linear", "digressive", "progressive"};
        VehicleParameters vehicle = new VehicleParameters();

        // Initialize optimizer
        ObjectiveFunction objective = new ObjectiveFunction(vehicle);
        BayesianOptimizer optimizer = new BayesianOptimizer(objective, bounds, dampingProfiles, 100);

        // Run optimization
        double[] optimalParams = optimizer.optimize();

        // Extract profile
        int profileIndex = (int) optimalParams[optimalParams.length - 1];
        String optimalProfile = optimizer.getProfileByIndex(profileIndex);

        // Output results
        System.out.println("Optimal Damper Parameters:");
        System.out.printf("Front Compression Damping: %.2f Ns/m%n", optimalParams[0]);
        System.out.printf("Front Rebound Damping: %.2f Ns/m%n", optimalParams[1]);
        System.out.printf("Rear Compression Damping: %.2f Ns/m%n", optimalParams[2]);
        System.out.printf("Rear Rebound Damping: %.2f Ns/m%n", optimalParams[3]);
        System.out.printf("Blow-off Threshold: %.2f m/s%n", optimalParams[4]);
        System.out.printf("Gas Pressure: %.2f Pa%n", optimalParams[5]);
        System.out.printf("Front Mounting X: %.2f m%n", optimalParams[6]);
        System.out.printf("Front Mounting Y: %.2f m%n", optimalParams[7]);
        System.out.printf("Front Mounting Z: %.2f m%n", optimalParams[8]);
        System.out.printf("Rear Mounting X: %.2f m%n", optimalParams[9]);
        System.out.printf("Rear Mounting Y: %.2f m%n", optimalParams[10]);
        System.out.printf("Rear Mounting Z: %.2f m%n", optimalParams[11]);
        System.out.printf("Front Inclination Angle: %.2f rad%n", optimalParams[12]);
        System.out.printf("Rear Inclination Angle: %.2f rad%n", optimalParams[13]);
        System.out.printf("Front Spring Stiffness: %.2f N/m%n", optimalParams[14]);
        System.out.printf("Rear Spring Stiffness: %.2f N/m%n", optimalParams[15]);
        System.out.printf("Camber Angle: %.4f rad%n", optimalParams[16]);
        System.out.printf("Anti-Dive Ratio: %.2f%n", optimalParams[17]);
        System.out.printf("Damping Profile: %s%n", optimalProfile);

        // Evaluate final objective
        double[] evalParams = new double[optimalParams.length - 1];
        System.arraycopy(optimalParams, 0, evalParams, 0, evalParams.length);
        double finalValue = objective.evaluate(evalParams, optimalProfile);
        System.out.printf("Final Objective Value: %.4f%n", finalValue);
    }
}