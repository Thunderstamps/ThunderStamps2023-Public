package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.communications.NetworkTableComms;
import frc.robot.utilities.OnDelayTimer;

public class ConveyorSubsystem extends TalonFxRollerSubsystemBase {

    private final static boolean INVERTED = true;
    private final static double KP = 10.0;
    
    private static final double CONE_FORWARD_SPEED_RPS = 50;
    private static final double CUBE_FORWARD_SPEED_RPS = 30;
    private static final double REVERSE_SPEED_RPS = -50;
    private static final double FEEDFORWARD_A = 35;

    private IntakeMode mode;
    private final Solenoid downSolenoid = new Solenoid(2, PneumaticsModuleType.REVPH, 0);
    private NetworkTableComms nt;
    private final OnDelayTimer downTimer = new OnDelayTimer(500);
    private final OnDelayTimer upTimer = new OnDelayTimer(500);

    // switchable channel controls the targeting light
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    private int targetLightCount;
    private boolean isTargetLightOn = false;
    private int counter;

    public ConveyorSubsystem(
        IntakeMode mode,
        NetworkTableComms nt) {
        super(6, KP, INVERTED);
        this.mode = mode;
        this.nt = nt;
    }
    
    @Override
    public void periodic() {
        var speed_rps = 0.0;
        var feedForward_A = 0.0;
        switch(this.mode.getMode()) {
            case ForwardDown:
            case ForwardUp:
                speed_rps = this.forwardSpeed_rps();
                feedForward_A = FEEDFORWARD_A;
                break;
            case ReverseDown:
            case Regrip:
                speed_rps = REVERSE_SPEED_RPS;
                feedForward_A = -FEEDFORWARD_A;
                break;
            default:
                speed_rps = 0.0;
                feedForward_A = 0.0;
                break;
        }
        this.executeVelocity(speed_rps, feedForward_A);
        
        // System.out.printf("Conv: %.2f rps, %.2f, %.2f A", speed_rps, this.getVelocity_rps(), this.getTorqueCurrent_A());
        // System.out.println();

        var down = false;

        switch(this.mode.getMode()) {
            case ForwardDown:
            case ReverseDown:
                this.Down();
                down = true;
                break;
            default:
                this.Up();
                break;
        }

        this.downTimer.execute(down);
        this.upTimer.execute(!down);

        this.counter++;
        if(counter >= 6) {
            counter = 0;
            if(down || this.targetLightCount > 0 && !this.isTargetLightOn) {
                if(this.targetLightCount >= 5) {
                    this.pdh.setSwitchableChannel(true);
                    this.isTargetLightOn = true;
                }
                else {
                    this.targetLightCount++;
                    var on = this.targetLightCount % 2 == 1;
                    this.pdh.setSwitchableChannel(on);
                }
            }
            else {
                this.targetLightCount = 0;
                this.pdh.setSwitchableChannel(false);
                this.isTargetLightOn = false;
            }
        }
    }

    private double forwardSpeed_rps() {
        if(this.nt.isGamepieceCube()) {
            return CUBE_FORWARD_SPEED_RPS;
        }
        return CONE_FORWARD_SPEED_RPS;
    }
    
    public void Up() {
        if(this.isEnabled()) {
            this.downSolenoid.set(false);
        }
    }

    public void Down() {
        if(this.isEnabled()) {
            this.downSolenoid.set(true);
        }
    }

    public boolean isUp() {
        return this.upTimer.getOutput();
    }

    public boolean isDown() {
        return this.downTimer.getOutput();
    }
}
