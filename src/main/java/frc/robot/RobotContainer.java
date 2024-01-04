// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.controllers.*;
import frc.robot.autonomous.*;
import frc.robot.autonomous.helpers.TestCommand;
import frc.robot.autonomous.helpers.TimeoutAndStop;
import frc.robot.communications.*;
import frc.robot.communications.gamepieces.*;
import frc.robot.communications.scorelocations.*;
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.arm.commands.*;
import frc.robot.subsystems.arm.commands.extension.*;
import frc.robot.subsystems.arm.commands.gripper.*;
import frc.robot.subsystems.arm.commands.joint.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.intake.commands.*;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.commands.*;

public class RobotContainer {
    
    // DRIVER (Xbox) BUTTONS
    private final JoystickButton resetGyroButton;
    private final JoystickButton intakeForwardButton;
    private final JoystickButton intakeUpAndArmToHoldButton;
    private final JoystickButton intakeReverseButton;
    private final JoystickButton autoLineupButton;
    private final JoystickButton releaseAndRetractArmButton;

    // OPERATOR (Joystick) BUTTONS
    private final JoystickButton selectCubeButton;
    private final JoystickButton selectConeButton;
    private final JoystickButton selectHighButton;
    private final JoystickButton selectMidButton;
    private final JoystickButton gripGripperButton;
    private final JoystickButton releaseGripperButton;
    private final JoystickButton operatorArmToScorePosition1;
    private final JoystickButton operatorArmToScorePosition2;
    private final JoystickButton operatorArmToHoldPosition1;
    private final JoystickButton operatorArmToHoldPosition2;

    // COMBINED
    private final Trigger armToScorePositionButtons;
    private final Trigger armToHoldPositionButtons;

    private final IXboxController xboxController;
    private final SwerveSubsystem swerveSubsystem;
    private final IntakeMode intakeMode;
    private final TopRollerSubsystem topRollerSubsystem;
    private final ConveyorSubsystem conveyorSubsystem;
    private final ArmHomeSensor armHomeSensor = new ArmHomeSensor();
    private final JointSubsystem jointSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final GripperSubsystem gripperSubsystem;
    private final NetworkTableComms nt;
    
    private final SendableChooser<Command> autoPathChooser = new SendableChooser<>();
    private final StopCommand stopCommand;
    private final AutoScoreHighCommand autoScoreHighCommand;
    private final AutoLeftScore2 autoLeftScore2;
    private final AutoLeftScore1Pick autoLeftScore1Pick;
    private final AutoLeftScore1PickAndBalance autoLeftScore1PickAndBalance;
    private final AutoBalance autoBalance;
    private final AutoBalanceWithMobility autoBalanceWithMobility;
    private final AutoRightScore2 autoRightScore2;
    private final AutoRightScore1Pick autoRightScore1Pick;
    private final AutoRightScore1PickAndBalance autoRightScore1PickAndBalance;
    private final AutoLeftScoreConeAndCube autoLeftScoreConeAndCube;
    private final AutoRightScoreConeAndCube autoRightScoreConeAndCube;
    private final TestCommand testCommand;
    private final IGyro gyro;

    public RobotContainer(
        NetworkTableComms nt,
        IXboxController xboxController,
        Joystick joystick,
        SwerveSubsystem swerveSubsystem,
        IGyro gyro) {

        this.nt = nt;
        this.gyro = gyro;
        this.gripperSubsystem = new GripperSubsystem(xboxController);
        this.jointSubsystem = new JointSubsystem(nt, armHomeSensor, gripperSubsystem);
        this.extensionSubsystem = new ExtensionSubsystem(jointSubsystem, nt, armHomeSensor, joystick);
        
        this.xboxController = xboxController;
        this.swerveSubsystem = swerveSubsystem;

        this.intakeMode = new IntakeMode(joystick);
        this.topRollerSubsystem = new TopRollerSubsystem(intakeMode);
        this.conveyorSubsystem = new ConveyorSubsystem(intakeMode, nt);
        
        this.resetGyroButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kBack.value);
        this.intakeForwardButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kX.value);
        this.intakeUpAndArmToHoldButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kY.value);
        this.intakeReverseButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kStart.value);
        this.autoLineupButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kB.value);
        this.releaseAndRetractArmButton = new JoystickButton(this.xboxController.getXboxController(), XboxController.Button.kA.value);

        this.selectCubeButton = new JoystickButton(joystick, 5);
        this.selectConeButton = new JoystickButton(joystick, 3);
        this.selectHighButton = new JoystickButton(joystick, 6);
        this.selectMidButton = new JoystickButton(joystick, 4);
        this.gripGripperButton = new JoystickButton(joystick, 1);
        this.releaseGripperButton = new JoystickButton(joystick, 2);
        this.operatorArmToScorePosition1 = new JoystickButton(joystick, 7);
        this.operatorArmToScorePosition2 = new JoystickButton(joystick, 8);
        this.operatorArmToHoldPosition1 = new JoystickButton(joystick, 11);
        this.operatorArmToHoldPosition2 = new JoystickButton(joystick, 12);

        this.armToScorePositionButtons = 
            this.operatorArmToScorePosition1
            .or(this.operatorArmToScorePosition2);

        this.armToHoldPositionButtons =
            this.intakeUpAndArmToHoldButton
            .or(this.operatorArmToHoldPosition1)
            .or(this.operatorArmToHoldPosition2);

        initGamepieces(); // binds dashboard buttons
        initScoreLocations(); // binds dashboard buttons

        initSwerve();
        initIntake(nt); // binds commands to intake mode
        initTopRoller();
        initConveyor();
        initArm();
        initGripper(joystick);

        this.stopCommand = new StopCommand(swerveSubsystem);
        this.autoScoreHighCommand = new AutoScoreHighCommand(gripperSubsystem, jointSubsystem, extensionSubsystem);
        this.autoLeftScore2 = new AutoLeftScore2(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);
        this.autoLeftScore1Pick = new AutoLeftScore1Pick(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);
        this.autoLeftScore1PickAndBalance = new AutoLeftScore1PickAndBalance(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode, gyro);
        this.autoBalance = new AutoBalance(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, gyro);
        this.autoBalanceWithMobility = new AutoBalanceWithMobility(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, gyro);
        this.autoRightScore2 = new AutoRightScore2(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);
        this.autoRightScore1Pick = new AutoRightScore1Pick(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);
        this.autoRightScore1PickAndBalance = new AutoRightScore1PickAndBalance(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode, gyro);
        this.autoLeftScoreConeAndCube = new AutoLeftScoreConeAndCube(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);
        this.autoRightScoreConeAndCube = new AutoRightScoreConeAndCube(nt, swerveSubsystem, gripperSubsystem, jointSubsystem, extensionSubsystem, intakeMode);
        this.testCommand = new TestCommand(swerveSubsystem);
        
        this.autoPathChooser.setDefaultOption("None", this.stopCommand);
        this.autoPathChooser.addOption("Score High (no move)", this.autoScoreHighCommand);
        this.autoPathChooser.addOption("Left Side 1.9", this.autoLeftScore2);
        this.autoPathChooser.addOption("Left Side Score High, Pick (mobility)", this.autoLeftScore1Pick);
        this.autoPathChooser.addOption("Left Side Score High, Pick, Balance", 
            new TimeoutAndStop(swerveSubsystem, this.autoLeftScore1PickAndBalance, 14.7));
        this.autoPathChooser.addOption("Center Score High, Auto Balance", this.autoBalance);
        this.autoPathChooser.addOption("Center Score High, Mobility, Auto Balance", 
            new TimeoutAndStop(swerveSubsystem, this.autoBalanceWithMobility, 14.7));
        this.autoPathChooser.addOption("Right Side 1.9", this.autoRightScore2);
        this.autoPathChooser.addOption("Right Side Score High, Pick (mobility)", this.autoRightScore1Pick);
        this.autoPathChooser.addOption("Right Side Score High, Pick, Balance", 
            new TimeoutAndStop(swerveSubsystem, this.autoRightScore1PickAndBalance, 14.7));
        this.autoPathChooser.addOption("Left 2.5", this.autoLeftScoreConeAndCube);
        this.autoPathChooser.addOption("Right 2.5", this.autoRightScoreConeAndCube);
        this.autoPathChooser.addOption("Test Wheels", this.testCommand);
        
        nt.initializeAutoChooser(autoPathChooser);
    }

    public void robotPeriodic() {
        this.extensionSubsystem.robotPeriodic();
        this.jointSubsystem.robotPeriodic();

        if(this.jointSubsystem.isAtPick()) {
            this.swerveSubsystem.setMaxRotationRate(Robot.MAX_ROTATION_RATE_RAD_S);
        }
        else {
            this.swerveSubsystem.setMaxRotationRate(Robot.MAX_ROTATION_RATE_RAD_S / 4.0);
        }

        var gyro_deg = Math.toDegrees(this.gyro.getFieldOrientation_rad());
        var pitch_deg = this.gyro.getPitch_deg();
        this.nt.setGyro(gyro_deg, pitch_deg);

        
        // var roll_deg = this.gyro.getRoll_deg();
        // System.out.printf("Roll: %.2f, Pitch: %.2f", roll_deg, pitch_deg);
        // System.out.println();
    }

    public void autonomousInit() {
        this.intakeMode.setMode(IntakeModeEnum.Off);
    }

    public void teleopInit() {
        this.intakeMode.setMode(IntakeModeEnum.Off);
    }

    public void disabledInit() {
        this.intakeMode.setMode(IntakeModeEnum.Off);
    }

    private void initGamepieces() {
        var cubeCommand = new CubeGamepieceCommand(nt);
        var coneCommand = new ConeGamepieceCommand(nt);

        this.nt.initializeGamepieceCommands(
            cubeCommand, 
            coneCommand);

        this.selectCubeButton
            .onTrue(cubeCommand);
            
        this.selectConeButton
            .onTrue(coneCommand);
    }

    private void initScoreLocations() {
        var highCommand = new HighScoreLocationCommand(nt);
        var midCommand = new MidScoreLocationCommand(nt);

        this.nt.initializeScoreLocationCommands(
            highCommand, 
            midCommand);

        this.selectHighButton
            .onTrue(highCommand);
            
        this.selectMidButton
            .onTrue(midCommand);
    }

    private void initSwerve() {
        this.resetGyroButton.onTrue(new GyroZeroCommand(swerveSubsystem));
        this.nt.initializeSwerveCommands(
          new GyroEnableCommand(this.swerveSubsystem), 
          new GyroDisableCommand(this.swerveSubsystem));

          this.autoLineupButton
            .onTrue(new SwerveScoringModeOnCommand(swerveSubsystem));
          this.autoLineupButton
            .onFalse(new SwerveScoringModeOffCommand(swerveSubsystem));
        this.releaseAndRetractArmButton
            .or(this.armToHoldPositionButtons)
            .onTrue(new SwerveScoringModeOffCommand(swerveSubsystem));
        
        // this.releaseAndRetractArmButton
        //     .and(jointSubsystem::isAtConePlace)
        //     .and(extensionSubsystem::isAtConePlace)
        //     .and(gyro::getEnabled)
        //     .and(this::gyroNearZero)
        //     .onTrue(new GyroZeroCommand(swerveSubsystem));
    }

    private boolean gyroNearZero() {
        var absoluteDegrees = Math.abs(Math.toDegrees(this.gyro.getFieldOrientation_rad()));
        return absoluteDegrees <= 15.0;
    }

    private void initIntake(NetworkTableComms nt) {
        this.intakeForwardButton.onTrue(new IntakeForwardDownCommand(intakeMode));
        this.intakeUpAndArmToHoldButton.onTrue(new IntakeForwardUpCommand(intakeMode, nt));
        this.intakeReverseButton.onTrue(new IntakeReverseCommand(intakeMode, 2.0));
    }

    private void initTopRoller() {
        this.topRollerSubsystem.enable();
    }

    private void initConveyor() {
        this.conveyorSubsystem.enable();
    }

    private void initArm() {
        this.initJoint();
        this.initExtension();

        var allowArmMotion = 
            new Trigger(jointSubsystem::isHomed)
            .and(extensionSubsystem::isHomed);

        var armToHoldCommand = new ArmToHoldCommand(gripperSubsystem, jointSubsystem, extensionSubsystem);

        this.armToScorePositionButtons
            .and(allowArmMotion)
            .and(gripperSubsystem::isGripped)
            .and(this::gyroNearZero)
            .onTrue(new ArmToScoreCommand(nt, jointSubsystem, extensionSubsystem));
        this.releaseAndRetractArmButton
            .and(allowArmMotion)
            .onTrue(new ArmToPickCommand(gripperSubsystem, jointSubsystem, extensionSubsystem));
        this.armToHoldPositionButtons
            .and(allowArmMotion)
            .and(jointSubsystem::isNotAtPick)
            .onTrue(armToHoldCommand);

        this.intakeForwardButton
            .and(allowArmMotion)
            .and(gripperSubsystem::isGripped)
            .and(jointSubsystem::isAtPick)
            .onTrue(armToHoldCommand);
    }

    private void initJoint() {
        this.jointSubsystem.setDefaultCommand(new DefaultJointCommand(jointSubsystem));
    }

    private void initExtension() {
        this.extensionSubsystem.setDefaultCommand(new DefaultExtensionCommand(extensionSubsystem));
    }

    private void initGripper(Joystick joystick) {
        this.gripperSubsystem.setDefaultCommand(new DefaultGripperCommand(gripperSubsystem));
        
        var raiseConveyorAndGripCommand = new SequentialCommandGroup(
            new IntakeForwardUpCommand(intakeMode, nt).until(conveyorSubsystem::isUp),
            new GripGripperCommand(gripperSubsystem),
            new IntakeOffCommand(intakeMode)
        );

        this.gripGripperButton
            .onTrue(raiseConveyorAndGripCommand);
        
        var releaseAndRunConveyorCommand = new SequentialCommandGroup(
            new ReleaseGripperCommand(gripperSubsystem),
            new IntakeRegripCommand(intakeMode),
            new IntakeForwardUpCommand(intakeMode, nt)
        );

        this.releaseGripperButton
            .onTrue(releaseAndRunConveyorCommand);
    }

    public Command getAutonomousCommand() {
        return this.autoPathChooser.getSelected();
    }
}
