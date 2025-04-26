package solution.damper_placement.source;

import java.util.Random;

public class RoadProfile {
    private final String name;
    private final double minFreq; // Hz
    private final double maxFreq; // Hz
    private final double amplitude; // m
    private final double noiseStd; // m
    public RoadProfile(String name, double minFreq, double maxFreq, double amplitude, double noiseStd) {
        this.name = name;
        this.minFreq = minFreq;
        this.maxFreq = maxFreq;
        this.amplitude = amplitude;
        this.noiseStd = noiseStd;
    }
    public double[] generateInputs(double time, Random random) {
        double[] inputs = new double[4];
        double freq = minFreq + (maxFreq - minFreq) * random.nextDouble();
        double baseBump = amplitude * Math.sin(2 * Math.PI * freq * time);
        double noise = noiseStd * random.nextGaussian();
        inputs[0] = baseBump + noise; // Front-left
        inputs[1] = baseBump + noise; // Front-right
        inputs[2] = amplitude * Math.sin(2 * Math.PI * freq * (time - 0.1)) + noise; // Rear-left
        inputs[3] = amplitude * Math.sin(2 * Math.PI * freq * (time - 0.1)) + noise; // Rear-right
        return inputs;
    }
}