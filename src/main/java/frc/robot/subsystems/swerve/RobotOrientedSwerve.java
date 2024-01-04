package frc.robot.subsystems.swerve;

import java.util.*;

public class RobotOrientedSwerve implements IRobotOrientedSwerve {

    private final ArrayList<SwerveModuleWrapper> swerveModules;

    public RobotOrientedSwerve(
            ArrayList<ISwerveModule> swerveModules) {
        this.swerveModules = new ArrayList<>(swerveModules.size());
        for (ISwerveModule swerveModule : swerveModules) {
            this.swerveModules.add(new SwerveModuleWrapper(swerveModule));
        }
    }

    @Override
    public void execute(IVector2D translationCommand_in_s_rad, double targetRotationRate_rad_s) {

        if (!allHomed()) {
            return;
        }

        for (SwerveModuleWrapper swerveModule : this.swerveModules) {
            swerveModule.calculateUnlimitedVelocityCommand(
                    translationCommand_in_s_rad,
                    targetRotationRate_rad_s);
        }

        double maxSpeedFactor = 1.0;
        for (SwerveModuleWrapper swerveModule : this.swerveModules) {
            double speedFactor = swerveModule.getSpeedFactor();
            maxSpeedFactor = Math.max(speedFactor, maxSpeedFactor);
        }

        for (SwerveModuleWrapper swerveModule : this.swerveModules) {
            swerveModule.applyAndExecute(maxSpeedFactor);
        }

    }

    private boolean allHomed() {
        boolean result = true;
        for (SwerveModuleWrapper swerveModule : this.swerveModules) {
            if (!swerveModule.isHomed()) {
                result = false;
                break;
            }
        }
        return result;
    }

    class SwerveModuleWrapper {
        private final ISwerveModule swerveModule;
        private final double maxSpeed_in_s;
        private final IVector2D positionVector;
        private final Vector2D unlimitedVelocity = new Vector2D();

        public SwerveModuleWrapper(
                ISwerveModule swerveModule) {
            this.swerveModule = swerveModule;
            this.maxSpeed_in_s = swerveModule.getMaxSpeed_in_s();
            this.positionVector = swerveModule.getModulePos_in();
        }

        public boolean isHomed() {
            return this.swerveModule.isHomed();
        }

        public void calculateUnlimitedVelocityCommand(
                IVector2D translationCommand_in_s_rad,
                double targetRotationRate_rad_s) {

            double rotation_speed_in_s = this.positionVector.getMagnitude() * targetRotationRate_rad_s;
            double rotation_angle_rad = this.positionVector.getAngleRadians() + Math.PI / 2.0;
            IVector2D rotation_in_s_rad = Vector2D.FromPolar(
                    rotation_speed_in_s, rotation_angle_rad);
            unlimitedVelocity.add(translationCommand_in_s_rad, rotation_in_s_rad);
        }

        public double getSpeedFactor() {
            double speed = this.unlimitedVelocity.getMagnitude();
            if (speed > this.maxSpeed_in_s) {
                return speed / this.maxSpeed_in_s;
            }
            return 1.0;
        }

        public void applyAndExecute(double maxSpeedFactor) {
            double oldSpeed = this.unlimitedVelocity.getMagnitude();
            double newSpeed = oldSpeed / maxSpeedFactor;
            Vector2D limitedVelocity = new Vector2D();
            limitedVelocity.copy(this.unlimitedVelocity);
            limitedVelocity.setMagnitude(newSpeed);
            this.swerveModule.executeVelocityMode(limitedVelocity);
        }

    }
}