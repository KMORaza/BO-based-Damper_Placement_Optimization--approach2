package solution.damper_placement.source;

public class ExtendedDamperParameters {
    private double frontCompressionDamping; // Ns/m
    private double frontReboundDamping; // Ns/m
    private double rearCompressionDamping; // Ns/m
    private double rearReboundDamping; // Ns/m
    private double blowOffThreshold; // m/s
    private double gasPressure; // Pa
    private double frontMountingX; // m
    private double frontMountingY; // m
    private double frontMountingZ; // m
    private double rearMountingX; // m
    private double rearMountingY; // m
    private double rearMountingZ; // m
    private double frontInclinationAngle; // rad
    private double rearInclinationAngle; // rad
    private double frontSpringStiffness; // N/m
    private double rearSpringStiffness; // N/m
    private double camberAngle; // rad
    private double antiDiveRatio; // dimensionless
    private String dampingProfile; // linear, digressive, progressive
    private double[] motionRatios;

    public ExtendedDamperParameters(double[] params, String profile) {
        if (params == null) {
            throw new IllegalArgumentException("Parameter array cannot be null");
        }
        if (params.length != 18) {
            throw new IllegalArgumentException("Parameter array must have 18 elements, got " + params.length);
        }
        this.frontCompressionDamping = clamp(params[0], 1000, 10000);
        this.frontReboundDamping = clamp(params[1], 1000, 10000);
        this.rearCompressionDamping = clamp(params[2], 1000, 10000);
        this.rearReboundDamping = clamp(params[3], 1000, 10000);
        this.blowOffThreshold = clamp(params[4], 0.1, 2.0);
        this.gasPressure = clamp(params[5], 1e5, 10e5);
        this.frontMountingX = clamp(params[6], -0.5, 0.5);
        this.frontMountingY = clamp(params[7], -0.5, 0.5);
        this.frontMountingZ = clamp(params[8], 0.1, 1.0);
        this.rearMountingX = clamp(params[9], -0.5, 0.5);
        this.rearMountingY = clamp(params[10], -0.5, 0.5);
        this.rearMountingZ = clamp(params[11], 0.1, 1.0);
        this.frontInclinationAngle = clamp(params[12], 0, Math.PI / 4);
        this.rearInclinationAngle = clamp(params[13], 0, Math.PI / 4);
        this.frontSpringStiffness = clamp(params[14], 20000, 50000);
        this.rearSpringStiffness = clamp(params[15], 20000, 50000);
        this.camberAngle = clamp(params[16], -0.05, 0.05);
        this.antiDiveRatio = clamp(params[17], 0, 0.5);
        this.dampingProfile = profile != null ? profile : "linear";
        this.motionRatios = computeMotionRatios();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double[] computeMotionRatios() {
        double[] ratios = new double[4];
        for (int i = 0; i < 4; i++) {
            double x = i < 2 ? frontMountingX : rearMountingX;
            double y = i < 2 ? frontMountingY : rearMountingY;
            double z = i < 2 ? frontMountingZ : rearMountingZ;
            double angle = i < 2 ? frontInclinationAngle : rearInclinationAngle;
            double effectiveLength = Math.sqrt(x * x + y * y + z * z);
            double angleFactor = Math.cos(angle);
            ratios[i] = 1.0 / (effectiveLength * angleFactor + 0.1);
        }
        return ratios;
    }

    // Getters
    public double getFrontCompressionDamping() {
        return frontCompressionDamping;
    }

    public double getFrontReboundDamping() {
        return frontReboundDamping;
    }

    public double getRearCompressionDamping() {
        return rearCompressionDamping;
    }

    public double getRearReboundDamping() {
        return rearReboundDamping;
    }

    public double getBlowOffThreshold() {
        return blowOffThreshold;
    }

    public double getGasPressure() {
        return gasPressure;
    }

    public double getFrontMountingX() {
        return frontMountingX;
    }

    public double getFrontMountingY() {
        return frontMountingY;
    }

    public double getFrontMountingZ() {
        return frontMountingZ;
    }

    public double getRearMountingX() {
        return rearMountingX;
    }

    public double getRearMountingY() {
        return rearMountingY;
    }

    public double getRearMountingZ() {
        return rearMountingZ;
    }

    public double getFrontInclinationAngle() {
        return frontInclinationAngle;
    }

    public double getRearInclinationAngle() {
        return rearInclinationAngle;
    }

    public double getFrontSpringStiffness() {
        return frontSpringStiffness;
    }

    public double getRearSpringStiffness() {
        return rearSpringStiffness;
    }

    public double getCamberAngle() {
        return camberAngle;
    }

    public double getAntiDiveRatio() {
        return antiDiveRatio;
    }

    public String getDampingProfile() {
        return dampingProfile;
    }

    public double[] getMotionRatios() {
        return motionRatios.clone();
    }
}