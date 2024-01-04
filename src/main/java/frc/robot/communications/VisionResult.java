package frc.robot.communications;

public class VisionResult {
    
    private boolean found;
    private double degrees;

    public VisionResult(
            boolean found,
            double degrees) {
        this.found = found;
        this.degrees = degrees;
    }

    public boolean isTargetFound() {
        return this.found;
    }

    public double getDegrees() {
        return this.degrees;
    }
}
