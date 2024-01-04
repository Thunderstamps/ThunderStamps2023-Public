package frc.robot.subsystems.swerve;

public class SwerveUtil {
    /*
     * Swerve Manual says to use Modulo, discovered that this works while trying to
     * implement that.
     * Confused but happily suprised.
     */
    public static double limitAngleFromNegativePItoPI_rad(double angle_rad) {
        angle_rad = (Math.toDegrees(angle_rad) - (360 * (1 + (Math.floor(Math.toDegrees(angle_rad) / 360)))));
        if (angle_rad < -180) {
            angle_rad = (angle_rad + 360);
        }

        return Math.toRadians(angle_rad);
    }

    public static double shortestDiffToTargetAngle_rad(
            double currentAngle_rad,
            double targetAngle_rad) {
        double diff = limitAngleFromNegativePItoPI_rad(targetAngle_rad)
                - limitAngleFromNegativePItoPI_rad(currentAngle_rad);

        double diffMagnitude = Math.abs(diff);
        double diffDirection = diff >= 0 ? 1.0 : -1.0;

        if (diffMagnitude > Math.PI) {
            diffMagnitude = 2 * Math.PI - diffMagnitude;
            diffDirection *= -1.0;
        }
        return diffMagnitude * diffDirection;
    }

}
