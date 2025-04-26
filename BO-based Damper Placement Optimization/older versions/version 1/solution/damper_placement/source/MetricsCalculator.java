package solution.damper_placement.source;

public class MetricsCalculator {
    public double computeISO2631Comfort(double[] accelerations) {
        // Simplified ISO 2631-1: Frequency-weighted RMS acceleration
        double sum = 0.0;
        for (double a : accelerations) {
            // Apply frequency weighting (simplified as 1.0 for 0.5-80 Hz)
            sum += a * a;
        }
        return Math.sqrt(sum / accelerations.length);
    }

    public double computeFrequencyWeightedPSD(double[] displacements) {
        // Simplified frequency-weighted PSD
        double sum = 0.0;
        for (double d : displacements) {
            sum += d * d; // Assume weighting emphasizes 1-10 Hz
        }
        return sum / displacements.length;
    }

    public double computeHandlingMetric(double[] tireLoadVariations) {
        // RMS of tire load variations
        double sum = 0.0;
        for (double v : tireLoadVariations) {
            sum += v * v;
        }
        return Math.sqrt(sum / tireLoadVariations.length);
    }
}