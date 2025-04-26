package solution.damper_placement.source;

public class VehicleParameters {
    private final double sprungMass = 1500.0; // kg
    private final double unsprungMass = 50.0; // kg per wheel
    private final double trackWidth = 1.5; // m
    private final double wheelbase = 2.7; // m
    private final double rollInertia = 500.0; // kg·m^2
    private final double pitchInertia = 800.0; // kg·m^2
    private final double tireStiffness = 200000.0; // N/m
    private final double tireDamping = 500.0; // Ns/m

    public double getSprungMass() {
        return sprungMass;
    }

    public double getUnsprungMass() {
        return unsprungMass;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getWheelbase() {
        return wheelbase;
    }

    public double getRollInertia() {
        return rollInertia;
    }

    public double getPitchInertia() {
        return pitchInertia;
    }

    public double getTireStiffness() {
        return tireStiffness;
    }

    public double getTireDamping() {
        return tireDamping;
    }
}