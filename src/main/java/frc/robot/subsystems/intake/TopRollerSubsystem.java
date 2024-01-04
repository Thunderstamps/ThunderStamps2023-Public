package frc.robot.subsystems.intake;

public class TopRollerSubsystem extends TalonFxRollerSubsystemBase {

    private final static boolean INVERTED = false;
    private final static double KP = 5.0;
    
    private static final double FORWARD_SPEED_RPS = 20;
    private static final double REVERSE_SPEED_RPS = -20;
    private static final double FEEDFORWARD_A = 4.5;

    private IntakeMode mode;

    public TopRollerSubsystem(
        IntakeMode mode) {
        super(5, KP, INVERTED);
        this.mode = mode;
    }
    
    @Override
    public void periodic() {
        var speed_rps = 0.0;
        var feedForward_A = 0.0;
        switch(this.mode.getMode()) {
            case ForwardDown:
            case ForwardUp:
                speed_rps = FORWARD_SPEED_RPS;
                feedForward_A = FEEDFORWARD_A;
                break;
            case ReverseDown:
                speed_rps = REVERSE_SPEED_RPS;
                feedForward_A = -FEEDFORWARD_A;
                break;
            case Regrip:
            default:
                speed_rps = 0.0;
                break;
        }
        
        this.executeVelocity(speed_rps, feedForward_A);
        
        // System.out.printf("Top roller: %.2f rps, %.2f, %.2f A", speed_rps, this.getVelocity_rps(), this.getTorqueCurrent_A());
        // System.out.println();
    }
}
