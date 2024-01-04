package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

public class DriveController implements IDriveController {

    private final static double MAX_MOTOR_RPM = 6000.0;
    private final static double COUNTS_PER_MOTOR_REV = 2048.0;
    private final static double MAX_CURRENT_A = 40.0; // per motor, max current during accel
    private final double lowCurrent_A; // per motor, max current during docking to grid
    private final double motorRevsPerWheelRev;
    private final TalonFX talon;

    public DriveController(int talonFxCanBusAddress, double motorRevsPerWheelRev, double lowCurrent_A) {
        talon = new TalonFX(talonFxCanBusAddress, "Default Name");
        this.lowCurrent_A = lowCurrent_A;
        this.motorRevsPerWheelRev = motorRevsPerWheelRev;
        // neutral behavior
        talon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        // stop it
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.setInverted(false);

        talon.configVoltageCompSaturation(11.0); // compensates for voltage fluctuation, down to this many volts
        talon.enableVoltageCompensation(true);
        this.setMaxCurrentLimit();
        
        talon.selectProfileSlot(0, 0);
        talon.config_kF(0, 0.0505, 10);
        talon.config_kP(0, 0.060, 10);
        talon.config_kI(0, 0.000, 10);
        talon.config_kD(0, 0.0, 10);
        talon.config_IntegralZone(0, 600); // if kF is tuned well, error might sit around 150 or so
        
        talon.configClosedloopRamp(0.0, 10);
        talon.configOpenloopRamp(2.0, 10); // for kF tuning
    }

    public void setLowCurrentLimit() {
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, lowCurrent_A, lowCurrent_A, 0));
    }

    public void setMaxCurrentLimit() {
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, MAX_CURRENT_A, MAX_CURRENT_A, 0));
    }

    public void executeVelocityMode(double targetMotorRpmInput) {
        double targetMotorRPM = targetMotorRpmInput;

        // enforce sane limits
        if(targetMotorRPM > MAX_MOTOR_RPM) {
            targetMotorRPM = MAX_MOTOR_RPM;
        }
        if(targetMotorRPM < -MAX_MOTOR_RPM) {
            targetMotorRPM = -MAX_MOTOR_RPM;
        }

        double motorCountsPer100ms = COUNTS_PER_MOTOR_REV * targetMotorRPM / 600.0;
        talon.set(ControlMode.Velocity, motorCountsPer100ms);
    }

    public double getMotorRpm() {
        var rawVelocityCountsPer100ms = talon.getSelectedSensorVelocity();
        return 600.0 * rawVelocityCountsPer100ms / COUNTS_PER_MOTOR_REV;
    }

    public void executePositionMode(double targetMotorRev) {
        // not implementing yet
    }

    public double getMotorRevCount() {
        var rawCounts = talon.getSelectedSensorPosition();
        return rawCounts / COUNTS_PER_MOTOR_REV;
    }

    public double getMotorRevsPerWheelRev() {
        return motorRevsPerWheelRev;
    }

    public double getWheelDiameterInches() {
        return 3.93; // measured wheel at 3.75... measured with custom-made treads at 3.98, decided 3.93 4.8% more than 3.75 and roughly the same as auto overshoot
    }

    public double getMaxSpeedRpm() {
        return MAX_MOTOR_RPM;
    }

    public void tune_kF(double minusOneToOne) {
        // run the motor at x% output and see how fast it goes
        this.executePercent(minusOneToOne);
        var rawVelocityCountsPer100ms = talon.getSelectedSensorVelocity();

        // calculate what kF should be
        if(Math.abs(minusOneToOne) > 0.05 && Math.abs(rawVelocityCountsPer100ms) > 100) {
            double kF = (minusOneToOne * 1023.0) / rawVelocityCountsPer100ms;
            System.out.printf("%.2f", minusOneToOne); System.out.print(", ");
            System.out.print(rawVelocityCountsPer100ms); System.out.print(", ");
            System.out.printf("kF: %.6f", kF); System.out.print(", ");
            this.printInputCurrentAndRPM();
        }
    }

    public void printInputCurrentAndRPM() {
        System.out.printf("%.0f A, ", talon.getSupplyCurrent());
        System.out.printf("%.0f actual RPM", getMotorRpm());
        System.out.println();
    }

    private void executePercent(double minusOneToOne) {
        talon.set(ControlMode.PercentOutput, minusOneToOne);
    }

}