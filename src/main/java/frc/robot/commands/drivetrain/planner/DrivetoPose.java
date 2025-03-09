// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivetrain.planner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DrivetoPose extends Command {
    public CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;
    private boolean flipIt;

    private PPHolonomicDriveController mDriveController = AutoConstants.kDriveController;
    private final SwerveRequest.ApplyRobotSpeeds mChassisSpeed;

  /** Creates a new DrivetoPose. */
  public DrivetoPose(CommandSwerveDrivetrain mSwerve, Pose2d goalPose, boolean shouldFlip) {
    this.mSwerve = mSwerve;
    this.goalPose = goalPose;
    flipIt = shouldFlip;
    mChassisSpeed = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mSwerve);
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;
        if (flipIt == true) {
            goalState = goalState.flip();
        }

        mSwerve.setControl(mChassisSpeed
                .withSpeeds(mDriveController.calculateRobotRelativeSpeeds(mSwerve.getState().Pose, goalState)));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mSwerve.setControl(
                new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));

        mSwerve.setControl(new SwerveRequest.SwerveDriveBrake());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);

        var rotation =  diff.getRotation().getDegrees() < AutoConstants.kRotationTolerance.getDegrees();
    

        var position = diff.getTranslation().getNorm() < AutoConstants.kPositionTolerance
                .in(Meters);
        var speed = mSwerve.getSpeedAsDouble() < AutoConstants.kSpeedTolerance
                .in(MetersPerSecond);

        boolean done = rotation && position && speed;
        return done;
    }
}
