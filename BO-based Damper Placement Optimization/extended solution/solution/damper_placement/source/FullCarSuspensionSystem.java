package solution.damper_placement.source;

import java.util.Random;

public class FullCarSuspensionSystem {
    private static final double TIME_STEP = 0.01; // s
    private static final double SIMULATION_DURATION = 5.0; // s
    private static final double MAX_FORCE = 1e6; // N
    private static final double MAX_DISPLACEMENT = 0.5; // m
    private final ExtendedDamperParameters params;
    private final VehicleParameters vehicle;
    private final Random random;
    private final RoadProfile[] roadProfiles;

    public FullCarSuspensionSystem(ExtendedDamperParameters params, VehicleParameters vehicle) {
        this.params = params;
        this.vehicle = vehicle;
        this.random = new Random();
        this.roadProfiles = new RoadProfile[] {
            new RoadProfile("urban", 1.0, 10.0, 0.02, 0.01),
            new RoadProfile("highway", 0.1, 2.0, 0.05, 0.005),
            new RoadProfile("off-road", 0.5, 15.0, 0.1, 0.02)
        };
    }

    public double[] simulate() {
        double[] state = new double[14];
        double[] metrics = new double[3]; // Comfort, Vibration, Handling
        int steps = (int) (SIMULATION_DURATION / TIME_STEP);
        double[][] profileMetrics = new double[roadProfiles.length][3];
        for (int p = 0; p < roadProfiles.length; p++) {
            double[] chassisAccels = new double[steps];
            double[] chassisDisplacements = new double[steps];
            double[] tireLoadVariations = new double[steps];
            double time = 0.0;
            for (int i = 0; i < steps; i++) {
                double[] roadInputs = roadProfiles[p].generateInputs(time, random);
                double[] forces = computeForces(state, roadInputs);
                for (double f : forces) {
                    if (Double.isNaN(f) || Double.isInfinite(f)) {
                        return new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
                    }
                }
                state = updateState(state, forces);
                for (double s : state) {
                    if (Double.isNaN(s) || Double.isInfinite(s)) {
                        return new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
                    }
                }
                chassisAccels[i] = computeChassisAcceleration(state, forces);
                chassisDisplacements[i] = state[0];
                tireLoadVariations[i] = computeTireLoadVariation(state, forces);
                if (Double.isNaN(chassisAccels[i]) || Double.isInfinite(chassisAccels[i])) {
                    chassisAccels[i] = 1e3;
                }
                if (Double.isNaN(tireLoadVariations[i]) || Double.isInfinite(tireLoadVariations[i])) {
                    tireLoadVariations[i] = 1e4;
                }
                time += TIME_STEP;
            }
            MetricsCalculator calculator = new MetricsCalculator();
            profileMetrics[p][0] = calculator.computeISO2631Comfort(chassisAccels);
            profileMetrics[p][1] = calculator.computeFrequencyWeightedPSD(chassisDisplacements);
            profileMetrics[p][2] = calculator.computeHandlingMetric(tireLoadVariations);
        }
        for (int i = 0; i < 3; i++) {
            double sum = 0.0;
            for (int p = 0; p < roadProfiles.length; p++) {
                sum += profileMetrics[p][i];
                if (Double.isNaN(profileMetrics[p][i]) || Double.isInfinite(profileMetrics[p][i])) {
                    metrics[i] = Double.MAX_VALUE;
                }
            }
            metrics[i] = sum / roadProfiles.length;
        }
        return metrics;
    }

    private double[] computeForces(double[] state, double[] roadInputs) {
        double z = state[0], theta = state[1], phi = state[2];
        double z_fl = state[3], z_fr = state[4], z_rl = state[5], z_rr = state[6];
        double vz_fl = state[10], vz_fr = state[11], vz_rl = state[12], vz_rr = state[13];
        double[] suspensionForces = new double[4];
        double[] tireForces = new double[4];
        double[] motionRatios = params.getMotionRatios();
        for (int i = 0; i < 4; i++) {
            double z_susp = computeSuspensionDisplacement(z, theta, phi, i);
            double vz_susp = computeSuspensionVelocity(state, i);
            double relVel = vz_susp - state[10 + i];
            double springForce = (i < 2 ? params.getFrontSpringStiffness() : params.getRearSpringStiffness()) * (z_susp - state[3 + i]);
            double dampingForce = computeDampingForce(relVel * motionRatios[i], i) / motionRatios[i];
            suspensionForces[i] = clamp(springForce + dampingForce, -MAX_FORCE, MAX_FORCE);
            double tireDisp = state[3 + i] - roadInputs[i];
            double tireVel = state[10 + i];
            tireForces[i] = clamp(
                vehicle.getTireStiffness() * tireDisp + vehicle.getTireDamping() * tireVel,
                0, MAX_FORCE
            );
        }
        double fz = suspensionForces[0] + suspensionForces[1] + suspensionForces[2] + suspensionForces[3];
        double m_roll = vehicle.getTrackWidth() / 2 * (suspensionForces[0] - suspensionForces[1] + suspensionForces[2] - suspensionForces[3]);
        double m_pitch = vehicle.getWheelbase() / 2 * (suspensionForces[0] + suspensionForces[1] - suspensionForces[2] - suspensionForces[3]);
        double[] f_unsprung = new double[4];
        for (int i = 0; i < 4; i++) {
            f_unsprung[i] = clamp(-suspensionForces[i] + tireForces[i], -MAX_FORCE, MAX_FORCE);
        }
        return new double[]{fz, m_roll, m_pitch, f_unsprung[0], f_unsprung[1], f_unsprung[2], f_unsprung[3]};
    }

    private double computeDampingForce(double relVel, int corner) {
        double absVel = Math.abs(relVel);
        double c_c = corner < 2 ? params.getFrontCompressionDamping() : params.getRearCompressionDamping();
        double c_r = corner < 2 ? params.getFrontReboundDamping() : params.getRearReboundDamping();
        double blowOff = params.getBlowOffThreshold();
        String profile = params.getDampingProfile();
        double c_eff = relVel >= 0 ? c_r : c_c;
        if (profile.equals("digressive") && absVel > blowOff) {
            c_eff *= blowOff / absVel;
        } else if (profile.equals("progressive") && absVel > blowOff) {
            c_eff *= Math.sqrt(absVel / blowOff);
        }
        return clamp(c_eff * relVel, -MAX_FORCE, MAX_FORCE);
    }

    private double computeSuspensionDisplacement(double z, double theta, double phi, int corner) {
        double t = vehicle.getTrackWidth() / 2;
        double l = vehicle.getWheelbase() / 2;
        double disp;
        if (corner == 0) disp = z + t * theta + l * phi;
        else if (corner == 1) disp = z - t * theta + l * phi;
        else if (corner == 2) disp = z + t * theta - l * phi;
        else disp = z - t * theta - l * phi;
        return clamp(disp, -MAX_DISPLACEMENT, MAX_DISPLACEMENT);
    }

    private double computeSuspensionVelocity(double[] state, int corner) {
        double vz = state[7], vtheta = state[8], vphi = state[9];
        double t = vehicle.getTrackWidth() / 2;
        double l = vehicle.getWheelbase() / 2;
        if (corner == 0) return vz + t * vtheta + l * vphi;
        if (corner == 1) return vz - t * vtheta + l * vphi;
        if (corner == 2) return vz + t * vtheta - l * vphi;
        return vz - t * vtheta - l * vphi;
    }

    private double[] updateState(double[] state, double[] forces) {
        double[] newState = state.clone();
        double fz = forces[0], m_roll = forces[1], m_pitch = forces[2];
        newState[7] += (fz / vehicle.getSprungMass()) * TIME_STEP;
        newState[8] += (m_roll / vehicle.getRollInertia()) * TIME_STEP;
        newState[9] += (m_pitch / vehicle.getPitchInertia()) * TIME_STEP;
        for (int i = 0; i < 4; i++) {
            newState[10 + i] += (forces[3 + i] / vehicle.getUnsprungMass()) * TIME_STEP;
        }
        newState[0] += newState[7] * TIME_STEP;
        newState[1] += newState[8] * TIME_STEP;
        newState[2] += newState[9] * TIME_STEP;
        for (int i = 0; i < 4; i++) {
            newState[3 + i] += newState[10 + i] * TIME_STEP;
            newState[3 + i] = clamp(newState[3 + i], -MAX_DISPLACEMENT, MAX_DISPLACEMENT);
        }
        return newState;
    }

    private double computeChassisAcceleration(double[] state, double[] forces) {
        return clamp(forces[0] / vehicle.getSprungMass(), -1e3, 1e3);
    }

    private double computeTireLoadVariation(double[] state, double[] forces) {
        double[] tireLoads = new double[4];
        for (int i = 0; i < 4; i++) {
            tireLoads[i] = clamp(
                vehicle.getTireStiffness() * state[3 + i] + vehicle.getTireDamping() * state[10 + i],
                0, MAX_FORCE
            );
        }
        double mean = (tireLoads[0] + tireLoads[1] + tireLoads[2] + tireLoads[3]) / 4;
        double variance = 0;
        for (double load : tireLoads) {
            variance += Math.pow(load - mean, 2);
        }
        return clamp(Math.sqrt(variance / 4), 0, 1e4);
    }

    private double clamp(double value, double min, double max) {
        if (Double.isNaN(value) || Double.isInfinite(value)) return max;
        return Math.max(min, Math.min(max, value));
    }
}