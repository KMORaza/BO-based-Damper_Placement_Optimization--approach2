package solution.damper_placement.source;

public class MetricsCalculator {
    private static final double FREQ_MIN = 1.0; // Hz
    private static final double FREQ_MAX = 10.0; // Hz
    private static final double TIME_STEP = 0.01; // s

    public double computeISO2631Comfort(double[] accelerations) {
        /// Frequency-weighted RMS acceleration
        double sum = 0.0;
        for (double a : accelerations) {
            sum += a * a; 
        }
        return Math.sqrt(sum / accelerations.length);
    }

    public double computeFrequencyWeightedPSD(double[] displacements) {
        /// Approximate frequency-weighted PSD for 1-10 Hz
        double sum = 0.0;
        int n = displacements.length;
        /// Simulate frequency weighting by scaling displacements
        for (int i = 1; i < n; i++) {
            double vel = (displacements[i] - displacements[i - 1]) / TIME_STEP;
            double freq = estimateFrequency(i, n);
            double weight = (freq >= FREQ_MIN && freq <= FREQ_MAX) ? 1.0 : 0.1;
            sum += weight * vel * vel; 
        }
        return sum / (n - 1); 
    }

    public double computeHandlingMetric(double[] tireLoadVariations) {
        /// RMS of tire load variations
        double sum = 0.0;
        for (double v : tireLoadVariations) {
            sum += v * v;
        }
        return Math.sqrt(sum / tireLoadVariations.length);
    }

    private double estimateFrequency(int index, int n) {
        /// Rough estimate of frequency based on index and time step
        double period = index * TIME_STEP;
        return period > 0 ? 1.0 / period : FREQ_MIN;
    }
}