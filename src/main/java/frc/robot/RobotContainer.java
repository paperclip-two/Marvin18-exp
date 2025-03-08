
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElevatorAlgaeComand;
import frc.robot.commands.drivetrain.planner.AligntoFeeder;
import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.DrivetrainTelemetry;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.LED.LEDColor;
import frc.robot.subsystems.LED.LEDSection;
import frc.robot.subsystems.LED.Rolling;
import frc.robot.subsystems.LED.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Use open-loop control for drive motors
  private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo); // Use Closed-loop control for drive motors at low speeds
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController Pilot = new CommandXboxController(0);
  private final CommandXboxController Copilot = new CommandXboxController(1);
  private final CommandXboxController test = new CommandXboxController(2);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SendableChooser<Command> autoChooser;

  public final Timer m_timer = new Timer();

  private final LED LEDController = LED.getInstance();

  public final Algae m_algae = new Algae();
  public final Elevator m_elevator = new Elevator();
  public final Coral m_coral = new Coral();
  public final Vision reef_vision = new Vision(Constants.Vision.reefCameraName, Constants.Vision.reefRobotToCam);
  public final Vision feeder_vision = new Vision(Constants.Vision.feederCameraName, Constants.Vision.feederRobotToCam);

  
  public final DrivetrainTelemetry m_Telemetry = new DrivetrainTelemetry(drivetrain);


  public RobotContainer() {

    NamedCommands.registerCommand("Nearest Tag Align Left",
        new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.LeftX, DynamicConstants.AlignTransforms.LeftY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.LeftRot))));
    NamedCommands.registerCommand("Nearest Tag Align Center",
        new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.CentX, DynamicConstants.AlignTransforms.CentY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.CentRot))));
    NamedCommands.registerCommand("Nearest Tag Align Right",
        new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.RightX, DynamicConstants.AlignTransforms.RightY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.RightRot))));
    NamedCommands.registerCommand("Elevator Setpoint L1", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL1));
    NamedCommands.registerCommand("Elevator Setpoint L2", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL2));
    NamedCommands.registerCommand("Elevator Setpoint L3", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL3));
    NamedCommands.registerCommand("Elevator Setpoint L4", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL4));
    NamedCommands.registerCommand("Elevator Setpoint Algae Ground", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeGround));
    NamedCommands.registerCommand("Elevator Setpoint Algae Processor", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTee));
    NamedCommands.registerCommand("Elevator Setpoint Algae Top", m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTop));
    NamedCommands.registerCommand("Zero Elevator", m_elevator.zeroElevatorCommand().withTimeout(2)); // ensure that the robot stops running elevator down if limit isn't read.
    NamedCommands.registerCommand("Score", m_coral.runIntake(1).withTimeout(0.5));
    NamedCommands.registerCommand("Passive Intake", m_coral.runIntake(-0.2).until(() -> m_coral.hasCoral()));

    configureBindings();
    configureLEDTriggers();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("auto chooser", autoChooser);
  }

  public void configureBindings() {
    m_coral.setDefaultCommand(m_coral.runIntake(-0.2));
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(deadband(-Pilot.getLeftY(), 0.1) * 0.5 * MaxSpeed) // Drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // Y
                                                                                                             // (forward)
            .withVelocityY(deadband(-Pilot.getLeftX(), 0.1) * 0.5 * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(deadband(-Pilot.getRightX(), 0.1) * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
        ));
    // Bumper and Trigger Controls
    Pilot.leftBumper().whileTrue(new ElevatorAlgaeComand(m_elevator, m_algae));
    Pilot.rightBumper().whileTrue(m_algae.outtake());
    Pilot.rightTrigger().whileTrue(m_coral.runIntake(1).alongWith(LEDController.setState(getRightTriggerColors())));
    Pilot.leftTrigger().onTrue(m_elevator.zeroElevatorCommand());



    // POV Controls
    Pilot.povLeft()
        .whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityX(-DynamicConstants.Drive.leftRight * MaxSpeed).withVelocityY(0)));
    Pilot.povRight()
        .whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityX(DynamicConstants.Drive.leftRight * MaxSpeed).withVelocityY(0)));
    Pilot.povUp()
        .whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityY(DynamicConstants.Drive.forwardBackward * MaxSpeed).withVelocityX(0)));
    Pilot.povDown()
        .whileTrue(drivetrain.applyRequest(() -> robotDrive.withVelocityY(-DynamicConstants.Drive.forwardBackward * MaxSpeed).withVelocityX(0)));

    // Face Button Controls
    Pilot.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
    Pilot.a().whileTrue(new AligntoFeeder(drivetrain, m_coral));
    Pilot.y().whileTrue(new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.CentX, DynamicConstants.AlignTransforms.CentY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.CentRot))));
    Pilot.x().whileTrue(new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.LeftX, DynamicConstants.AlignTransforms.LeftY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.LeftRot))));
    Pilot.b().whileTrue(new DriveCoralScorePose(drivetrain, new Transform2d(DynamicConstants.AlignTransforms.RightX, DynamicConstants.AlignTransforms.RightY, Rotation2d.fromDegrees(DynamicConstants.AlignTransforms.RightRot))));


    /// Copilot
    /// Elevator and drive controls
    Copilot.povUp().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTop));
    Copilot.povDown().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeGround));
    Copilot.povLeft().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeTee));
    Copilot.povRight().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevAlgaeBot));


    Copilot.start().whileTrue(m_elevator.climbingCommand());
    Copilot.back().whileTrue(m_elevator.setServoCommand(0));

    // Make sure to use copilot's left stick for reef side selection

    // Face Button Controls Height selection

    Copilot.a().onTrue(m_elevator.zeroElevatorCommand()); // Save for height selection
    Copilot.b().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL3)); // Save for
                                                                                                             // height
                                                                                                             // selection
    Copilot.x().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL2)); // Save for
                                                                                                             // height
                                                                                                             // selection
    Copilot.y().onTrue(m_elevator.setMotionMagicPositionCommand(DynamicConstants.ElevatorSetpoints.elevL4)); // Save for
                                                                                                             // height
                                                                                                             // selection

 

    drivetrain.registerTelemetry(logger::telemeterize);

    /*
     * Pilot.povLeft().whileTrue(m_elevator.setMotionMagicPosition(()-> 3));
     * Pilot.povRight().whileTrue(m_coralArm.setMotionMagicPosition(() ->
     * DynamicConstants.ArmSetpoints.armTestPos));
     * Pilot.povUp().whileTrue(m_coralArm.setMotionMagicPosition(() -> 0.0));
     * Pilot.povDown().whileTrue(m_elevator.setMotionMagicPosition(() -> 0.0));
     * Pilot.leftTrigger().whileTrue(mCoral_Hopper.runIntake(0.8));
     * Pilot.rightTrigger().whileTrue(mCoral_Hopper.runIntake(-1));
     * Copilot.povUp().whileTrue(new ArmElevatorGroup(m_elevator, m_coralArm, 3,
     * 0.3));
     * Copilot.rightTrigger().whileTrue(m_elevator.runVoltage(2));
     * Copilot.leftTrigger().whileTrue(m_elevator.runVoltage(-2));
     * Copilot.leftBumper().whileTrue(m_elevator.SafeHopperReturn(m_coralArm).
     * alongWith(m_coralArm.setTest(0, m_elevator)));
     * Copilot.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));
     * Copilot.x().whileTrue(m_coralArm.runVoltage(-1));
     */
    // Pilot.rightBumper().onTrue(m_coralArm.ArmPosVoltage(3));
    // Pilot.leftBumper().onTrue(m_coralArm.ArmPosVoltage(1));

    // Pilot.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // Pilot.b().whileTrue(drivetrain.applyRequest(() ->
    // point.withModuleDirection(new Rotation2d(-Pilot.getLeftY(),
    // -Pilot.getLeftX()))
    // ));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log

    Pilot.back().and(Pilot.y()).whileTrue(m_elevator.sysIdDynamic(Direction.kForward));
    Pilot.back().and(Pilot.x()).whileTrue(m_elevator.sysIdDynamic(Direction.kReverse));
    Pilot.start().and(Pilot.y()).whileTrue(m_elevator.sysIdQuasistatic(Direction.kForward));
    Pilot.start().and(Pilot.x()).whileTrue(m_elevator.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press

    drivetrain.registerTelemetry(logger::telemeterize);

    // Pilot.leftTrigger().whileTrue(mCoral_Hopper.runVoltageUntilIRReading(1));
  }

  public Command getAutonomousCommand() {
  //  m_coral.setDefaultCommand(m_coral.runIntake(-0.2));
    return autoChooser.getSelected();
    // return new Command() {
    //
    // };
    // return new PathfindingCommand(null, null, null, null, null, null, null, null)
  }

  public void configureTestBindings() {
    // Elevator Test Bindings
    // test.a().whileTrue(mCoral_Hopper.runAgitator(0.1));
    // test.x().whileTrue(m_algae.runAlgaeWheels(0.1));
    // test.leftTrigger().whileTrue(m_elevator.runVoltage(1));
    // test.rightTrigger().whileTrue(m_elevator.runVoltage(-1));
    // test.leftBumper().whileTrue(m_coralArm.runVoltage(0.5));
    // test.rightBumper().whileTrue(m_coralArm.runVoltage(-0.5));;
   // test.a().whileTrue(new FieldCentricPIDMove(drivetrain, new Pose2d(2, 3, new Rotation2d(0))));

    // test.x().whileTrue(mCoral_Hopper.runIntake(0.1));
    // test.x().whileTrue(m_algae.intake());
    test.y().whileTrue(m_algae.outtake());
    // test.y().whileTrue(mCoral_Hopper.runCoralAgitator(0.1));
    // test.a().whileTrue(mCoral_Hopper.runCoralAgitator(-0.1));

    test.leftTrigger().whileTrue(m_algae.intake());

    test.rightTrigger().whileTrue(m_algae.outtake());

    test.povDown().whileTrue(m_elevator.setServoCommand(0));
    test.povLeft().whileTrue(m_elevator.setServoCommand(0.5));
    test.povUp().whileTrue(m_elevator.setServoCommand(1));
    // test.povRight().whileTrue(m_elevator.setServo(45));

    test.leftBumper().whileTrue(new ElevatorSetpoint(m_elevator, 5, test.leftBumper().getAsBoolean()));
    // test.rightBumper().whileTrue(new ElevatorSetpoint(m_elevator, 5));

  }

  private void configureLEDTriggers() {
    // Pilot.rightTrigger().whileTrue(LEDController.setState(getRightTriggerColors()));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);

      }
    } else {
      return 0.0;
    }
  }

  private Map<LEDSection, State> getRightTriggerColors() {
    Map<LEDSection, State> map = new HashMap<>();
    map.put(LEDSection.CORALINTAKEUP, new State(LEDColor.BLUE, Rolling.FORWARD));
    map.put(LEDSection.CORALINTAKEDOWN, new State(LEDColor.BLUE, Rolling.REVERSE));
    return map;
  }

}
