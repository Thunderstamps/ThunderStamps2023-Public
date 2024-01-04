package frc.robot.subsystems.swerve;

public class FieldOrientedSwerve implements IFieldOrientedSwerve {

    private final IRobotOrientedSwerve robotOrientedSwerve;
    private final IGyro gyro;
    private final double maxSpeed_in_s;
    private double maxAcceleration_in_s2;
    private double maxRotationRate_rad_s;
    private final double scanTime_s;
    private double rotationP;
    private double maximumTranslationChange_in_s;
    private Vector2D lastFieldTranslation = new Vector2D();

    public FieldOrientedSwerve(
            IRobotOrientedSwerve robotOrientedSwerve,
            IGyro gyro,
            double maxSpeed_in_s,
            double maxAcceleration_in_s2,
            double maxRotationRate_rad_s,
            double scanTime_s,
            double rotationP) {
        this.robotOrientedSwerve = robotOrientedSwerve;
        this.gyro = gyro;
        this.maxSpeed_in_s = maxSpeed_in_s;
        this.maxAcceleration_in_s2 = maxAcceleration_in_s2;
        this.maxRotationRate_rad_s = maxRotationRate_rad_s;
        this.scanTime_s = scanTime_s;
        this.rotationP = rotationP; 
        this.maximumTranslationChange_in_s = this.maxAcceleration_in_s2 * scanTime_s;

    }

    @Override
    public void execute(
            IVector2D fieldOrientedTranslationCommand_in_s_rad,
            double fieldOrientedHeadingCommand_rad,
            IVector2D robotOrientedTranslationCommand_in_s_rad,
            double targetRotationRate_rad_s) {
        double fieldOrientation_rad = this.gyro.getFieldOrientation_rad();

        // convert the robot-oriented translation into a field-oriented translation
        double robotTranslationMagnitude_in_s = robotOrientedTranslationCommand_in_s_rad.getMagnitude();
        double robotTranslationAngle_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(
                robotOrientedTranslationCommand_in_s_rad.getAngleRadians() + fieldOrientation_rad);
        Vector2D robotTranslationInField = Vector2D.FromPolar(robotTranslationMagnitude_in_s,
                robotTranslationAngle_rad);

        // combine the robot- and field-oriented translation commands
        Vector2D fieldTranslationTotal = new Vector2D();
        fieldTranslationTotal.add(
                fieldOrientedTranslationCommand_in_s_rad,
                robotTranslationInField);

        // limit to maximum speed
        if (fieldTranslationTotal.getMagnitude() > this.maxSpeed_in_s) {
            fieldTranslationTotal.setMagnitude(this.maxSpeed_in_s);
        }

        // limit acceleration
        Vector2D translationChange = new Vector2D();
        translationChange.subtract(fieldTranslationTotal, lastFieldTranslation);

        if (translationChange.getMagnitude() > this.maximumTranslationChange_in_s) {
            translationChange.setMagnitude(this.maximumTranslationChange_in_s);
        }

        fieldTranslationTotal.add(this.lastFieldTranslation, translationChange);

        this.lastFieldTranslation.copy(fieldTranslationTotal);

        // convert back to robot-oriented
        double totalMagnitude_in_s = fieldTranslationTotal.getMagnitude();
        double totalAngle_rad = SwerveUtil.limitAngleFromNegativePItoPI_rad(
                fieldTranslationTotal.getAngleRadians() - fieldOrientation_rad);
        Vector2D robotTranslationTotal = Vector2D.FromPolar(totalMagnitude_in_s, totalAngle_rad);

        // rotation PID
        double rotationError_rad = fieldOrientedHeadingCommand_rad - fieldOrientation_rad;
        rotationError_rad = SwerveUtil
                .limitAngleFromNegativePItoPI_rad(rotationError_rad);

        double rotateToTarget_rad_s = rotationError_rad * this.rotationP;

        // total rotation rate
        double totalRotationRate_rad_s = rotateToTarget_rad_s + targetRotationRate_rad_s;

        // limit to maximum rotation rate
        if (totalRotationRate_rad_s > this.maxRotationRate_rad_s) {
            totalRotationRate_rad_s = this.maxRotationRate_rad_s;
        } else if (totalRotationRate_rad_s < -this.maxRotationRate_rad_s) {
            totalRotationRate_rad_s = -this.maxRotationRate_rad_s;
        }

        // send the result to the robot-oriented swerve class
        this.robotOrientedSwerve.execute(
                robotTranslationTotal,
                totalRotationRate_rad_s);

    }

    public void setMaxAcceleration(double newMaxAcceleration_in_s2) {
        this.maxAcceleration_in_s2 = newMaxAcceleration_in_s2;
        this.maximumTranslationChange_in_s = this.maxAcceleration_in_s2 * scanTime_s;
    }

    public void setRotationP(double rotationP) {
        this.rotationP = rotationP;
    }

    public void setMaxRotationRate(double newMaxRotationRate_rad_s) {
        this.maxRotationRate_rad_s = newMaxRotationRate_rad_s;
    }

}