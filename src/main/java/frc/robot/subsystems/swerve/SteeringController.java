
package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.*;

public class SteeringController implements ISteeringController {

    private final static int COUNTS_PER_MOTOR_REV = 2048;
    private final static int CANCODER_COUNTS = 4096;

    private final TalonFX talon;
    private boolean homed;
    private int countsOffset;
    private int rawCountsWhenHomed;
    private CANCoder homeSensor;
    private int homeSensorForward;

    public SteeringController(
            int talonCanBusAddress, 
            int homeSensorCANCoder,
            int homeSensorForward){
        this.talon = new TalonFX(talonCanBusAddress, "Default Name");
        this.homeSensor = new CANCoder(homeSensorCANCoder, "Default Name");
        this.homeSensorForward = homeSensorForward;

        // configure the CANcoder to give us raw counts (0 to 4095)
        this.homeSensor.configFeedbackCoefficient(1.0, "count", SensorTimeBase.PerSecond);
        this.homeSensor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        this.homeSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10); // fast, until we're homed
        this.homeSensor.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 40); // don't care

        // neutral behavior
        talon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        
        // stop it
        talon.set(ControlMode.PercentOutput, 0.0);

        // current limits
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30.0, 30.0, 1.0));
        
        this.homed = false;
        
        talon.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, 10);
        
        talon.set(ControlMode.PercentOutput, 0.0);
        talon.setInverted(false); // counter-clockwise should be positive
        
        // these are the HOMING velocity control parameters
        talon.selectProfileSlot(0, 0);
        talon.config_kF(0, 0.1443, 10);
        talon.config_kP(0, 0.06, 10);
        talon.config_kI(0, 0.00, 10);
        talon.config_kD(0, 0, 10);
        talon.config_IntegralZone(0, 10);
        
        talon.configClosedloopRamp(0.0, 10);
        talon.configOpenloopRamp(0.0, 10);
    }

    @Override
    public boolean isHomed() {
        return homed;
    }

    @Override
    public double getMotorRevCount() {
        return ((double)this.getRawCounts() - this.rawCountsWhenHomed + this.countsOffset) 
            / (double)COUNTS_PER_MOTOR_REV;
    }

    // Raw motor counts
    public int getRawCounts() {
        return (int)talon.getSelectedSensorPosition();
    }

    @Override
    public void setTargetMotorRev(double targetMotorRev) {
        if (!homed) {
            this.tryHoming();
        }
        else {
            // standard execute code from tutorial document
            int sensorCounts = (int)(targetMotorRev * COUNTS_PER_MOTOR_REV);
            talon.set(ControlMode.MotionMagic, sensorCounts + this.rawCountsWhenHomed - this.countsOffset);
        }
    }

    public void tryHoming() {
        if (!homed) {
            referenceMotorPosition();
        }
    }

    @Override
    public double getMotorRevsPerSteeringRev() {
        return 12.8;
    }

    // This is needed to determine the value of the homeSensorForward constructor parameter
    public int getAbsoluteCANCoderPosition() {
        return (int)homeSensor.getAbsolutePosition();
    }

    private void referenceMotorPosition() {
        talon.set(ControlMode.PercentOutput, 0.0);

        this.rawCountsWhenHomed = this.getRawCounts();

        var cancoderCounts = this.getAbsoluteCANCoderPosition();
        double diffCounts = cancoderCounts - this.homeSensorForward;
        var halfCounts = CANCODER_COUNTS / 2;
        while(diffCounts > halfCounts) {
            diffCounts -= CANCODER_COUNTS;
        }
        while(diffCounts <= -halfCounts) {
            diffCounts += CANCODER_COUNTS;
        }
        var diffSteeringRevs = diffCounts / CANCODER_COUNTS;
        var diffMotorRevs = diffSteeringRevs * getMotorRevsPerSteeringRev();
        this.countsOffset = (int)(diffMotorRevs * COUNTS_PER_MOTOR_REV);

        this.homed = true;
        this.homeSensor.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 50); // fast, until we're homed

        // these are the POSITION control parameters
        this.talon.set(ControlMode.PercentOutput, 0.0);

        talon.config_kF(0, 0, 10);
        talon.config_kP(0, 0.50, 10);
        talon.config_kI(0, 0.00, 10);
        talon.config_kD(0, 0, 10);
        talon.config_IntegralZone(0, 10);
        
        talon.configClosedloopRamp(0.0, 10);
        talon.configOpenloopRamp(0.0, 10);

        talon.configMotionCruiseVelocity(143360, 10); // 143,360 counts per second = 70 rev/sec = 4200 RPM.  Max speed of motor 6400 RPM
        talon.configMotionAcceleration(400000, 10);
    }
}