package frc.robot.subsystems.intake;

import com.ctre.phoenixpro.*;
import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.*;
import com.ctre.phoenixpro.hardware.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class TalonFxRollerSubsystemBase extends SubsystemBase {
    private final TalonFX talon;
    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0); // for simple open loop control
    private final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, false);
    private final NeutralOut m_brake = new NeutralOut();
    private boolean enabled = false;

    protected TalonFxRollerSubsystemBase(int canbusAddress, double kP, boolean inverted) {

        this.talon = new TalonFX(canbusAddress, "Default Name");
                
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // make sure we don't pop a breaker
        configs.CurrentLimits.SupplyCurrentLimit = 40;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 200;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        var slot = configs.Slot0;
        slot.kP = kP; // An error of 1 rotation per second results in ? amps output
        slot.kI = 0; // An error of 1 rotation per second increases output by ? amps every second
        slot.kD = 0; // A change of 1000 rotation per second squared results in 1 amp output (if value is 0.001)

        // Peak output in amps (this is stator current)
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 200;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = -200;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = this.talon.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }
        if(!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        this.talon.setInverted(inverted);
    }

    public void enable() {
        this.enabled = true;
    }

    public boolean isEnabled() {
        return this.enabled;
    }

    public void execute(double speedZeroToOne) {
        this.talon.setControl(this.dutyCycleOut.withOutput(speedZeroToOne));
    }

    protected void executeVelocity(double speed_rps, double feedForward_A) {
        if(!enabled || Math.abs(speed_rps) < 0.1) {
            this.talon.setControl(this.m_brake);
        }
        else {
            var controlRequest = torqueVelocity
                .withFeedForward(feedForward_A)
                .withVelocity(speed_rps);
            this.talon.setControl(controlRequest);
        }
    }

    public double getVelocity_rps() {
        return this.talon.getVelocity().getValue();
    }

    public double getSupplyCurrent_A() {
        return this.talon.getSupplyCurrent().getValue();
    }

    public double getTorqueCurrent_A() {
        return this.talon.getTorqueCurrent().getValue();
    }
}
