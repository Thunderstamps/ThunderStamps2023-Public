package frc.robot.subsystems.arm;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.controllers.IXboxController;
import frc.controllers.PovService;
import frc.robot.Robot;
import frc.robot.utilities.OnDelayTimer;

public class GripperSubsystem extends SubsystemBase {
    private final DoubleSolenoid gripperSolenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 2, 3);
    private final OnDelayTimer grippedTimer = new OnDelayTimer(200);

    private static final double OUTPUT_LIMIT = 0.4; // 0.4 = 40%
    private static final double CONTINUOUS_CURRENT_LIMIT = 18.0; // A
    private static final double LOW_CURRENT_THRESHOLD = 5.0; // A
    private final OnDelayTimer highCurrentTimer = new OnDelayTimer(3000);
    private final OnDelayTimer lowCurrentTimer = new OnDelayTimer(6000);
    private boolean limitCurrent = false;

    private final CANSparkMax rotationMotor = new CANSparkMax(55, MotorType.kBrushless);
    private final RelativeEncoder rotationEncoder;
    private final RelativeEncoder alternateEncoder;
    private double lastPosition_rot;
    private double lastVelocity_rps;

    private double jointVelocity_rps;
    private double jointPosition_rot;
    private IXboxController xboxController;
    private PovService povService;

    private boolean lastGripped;
    private boolean grippedPulse;
    private double gripPosition_rot;
    private boolean directionDetermined;
    private boolean stopHoldingPosition;
    private double target_rot;

    private final OnDelayTimer stoppedTimer = new OnDelayTimer(60);

    private static final boolean USE_ALTERNATE_ENCODER = true;

    public GripperSubsystem(IXboxController xboxController) {
        this.xboxController = xboxController;
        this.povService = xboxController.getPovService();
        this.rotationMotor.setSmartCurrentLimit(40, 10);
        this.rotationMotor.setIdleMode(IdleMode.kBrake);
        this.rotationEncoder = this.rotationMotor.getEncoder();
        this.alternateEncoder = this.rotationMotor.getAlternateEncoder(4096);
        this.Off();
    }

    @Override
    public void periodic() {
        var delayedGripped = this.isGrippedDelayed();
        this.grippedTimer.execute(this.isGripped());

        double position_rot;
        double velocity_rps;
        if(USE_ALTERNATE_ENCODER) {
            position_rot = -this.alternateEncoder.getPosition();
            velocity_rps = -this.alternateEncoder.getVelocity() / 60.0;
        }
        else {
            position_rot = this.GetPosition();
            var tempVelocity_rps = (position_rot - this.lastPosition_rot) / Robot.SCAN_TIME_S;
            this.lastPosition_rot = position_rot;
            velocity_rps = this.lastVelocity_rps + tempVelocity_rps / 2.0;
            this.lastVelocity_rps = tempVelocity_rps;
        }

        this.stoppedTimer.execute(Math.abs(velocity_rps) < 0.3);

        this.grippedPulse = delayedGripped && !this.lastGripped;
        this.lastGripped = delayedGripped;

        if(this.grippedPulse) {
            this.gripPosition_rot = position_rot;
            this.directionDetermined = false;
            this.stopHoldingPosition = false;
        }
        
        if(this.isGripped() && delayedGripped) {
            position_rot -= this.gripPosition_rot;
            if(!this.directionDetermined) {
                if(position_rot > 0.2) {
                    // cone was flange first
                    this.target_rot = 0.33;
                    this.directionDetermined = true;
                }
                else if(position_rot < -0.2) {
                    // cone was tip first
                    this.target_rot = -0.28;
                    this.directionDetermined = true;
                }
            }
        }
        else {
            this.directionDetermined = false;
        }

        var aButtonOn = this.xboxController.getXboxController().getAButton();

        var output = 0.0;
        if(delayedGripped && this.isGripped() && !aButtonOn) {
            var absJointVelocity_rps = Math.abs(this.jointVelocity_rps); 
            if(absJointVelocity_rps > 0.05) {
                //output = absJointVelocity_rps / 2.0; // counteract centripetal force
                output = OUTPUT_LIMIT;
            }
            if(USE_ALTERNATE_ENCODER) {
                output = -velocity_rps * 0.23;

                if(this.directionDetermined && !this.stopHoldingPosition) {
                    var totalTarget_rot = this.target_rot + this.jointPosition_rot;
                    var error_rot = totalTarget_rot - position_rot;
                    if(error_rot > 0.35) {
                        output = OUTPUT_LIMIT;
                    }
                    else if(error_rot < -0.075 && velocity_rps > 0.5) {
                        output = -OUTPUT_LIMIT;
                    }
                    else if(error_rot > 0.075 && velocity_rps < -0.5) {
                        output = OUTPUT_LIMIT;
                    }
                    //output += error_rot * 0.23;

                    // var current_A = this.rotationMotor.getOutputCurrent();
                    // System.out.printf("Rotation %.2f tar, %.2f rot, %.2f rps, %.2f A", totalTarget_rot, position_rot, velocity_rps, current_A);
                    // System.out.println();
                }
            }
            else {
                output = -velocity_rps / 5.0; // dampen rotation
            }
        }

        var pov = this.povService.getPOV();
        if(pov != PovService.POV_NONE) {
            output = OUTPUT_LIMIT;
            this.stopHoldingPosition = true;
        }

        output = limitOutput(output);
        output = limitCurrent(output);
        this.rotationMotor.set(output);

        // var current_A = this.rotationMotor.getOutputCurrent();
        
        // System.out.printf("Rotation %.2f rot, %.2f rps, %.2f A", position_rot, velocity_rps, current_A);
        // System.out.println();
    }

    public boolean isStopped() {
        return this.stoppedTimer.getOutput();
    }

    private static double limitOutput(double value) {
        var output = value;
        if(output > OUTPUT_LIMIT) {
            output = OUTPUT_LIMIT;
        }
        else if(output < -OUTPUT_LIMIT) {
            output = -OUTPUT_LIMIT;
        }
        return output;
    }

    // Spec sheet says a NEO550 can sustain 20A basically indefinitely (longer than a match), 
    // but 40A will cause a motor to fail at 27 seconds.
    // Try to detect a locked rotor condition (motor stuck) and turn off current to let it cool down.
    private double limitCurrent(double value) {
        var output = value;
        var current_A = this.rotationMotor.getOutputCurrent();
        var highCurrent = current_A >= CONTINUOUS_CURRENT_LIMIT;
        var lowCurrent = current_A <= LOW_CURRENT_THRESHOLD;
        this.highCurrentTimer.execute(highCurrent);
        this.lowCurrentTimer.execute(lowCurrent);
        if(highCurrentTimer.getOutput()) {
            this.limitCurrent = true;
        }
        else if(lowCurrentTimer.getOutput()) {
            this.limitCurrent = false;
        }

        if(this.limitCurrent) {
            output = 0.0;
        }

        return output;
    }

    public void Off() {
        this.gripperSolenoid.set(Value.kOff);
    }

    public void Grip() {
        this.gripperSolenoid.set(Value.kForward);
    }

    public void Release() {
        this.gripperSolenoid.set(Value.kReverse);
    }

    public boolean isGripped() {
        return gripperSolenoid.get() == Value.kForward;
    }

    public boolean isGrippedDelayed() {
        return this.grippedTimer.getOutput();
    }

    public double GetPosition() {
        return this.rotationEncoder.getPosition();
    }

    public void SetJointVelocity_rps(double value_rps) {
        this.jointVelocity_rps = value_rps;
    }

    public void SetJointPosition_rot(double value_rot) {
        this.jointPosition_rot = value_rot;
    }
}
