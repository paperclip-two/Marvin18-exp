// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.planner;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants.AlignTransforms;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoseSelector extends Command {
  private Elevator elevator;
  private CommandSwerveDrivetrain dt;
  private SelectPose drivePose;
  private SelectPose L1;
  private SelectPose L2;
  private SelectPose L3;
  private SelectPose L4;
  private SelectPose R1;
  private SelectPose R2;
  private SelectPose R3;
  private SelectPose R4;
  private Command setpointCommand;

  /** Creates a new ElevatorAlign. */
  public PoseSelector(CommandSwerveDrivetrain drive, Elevator elev) {
    elevator = elev;
    dt = drive;
    L1 = new SelectPose(dt,
        new Transform2d(AlignTransforms.LeftXL1, AlignTransforms.LeftYL1,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
    L2 = new SelectPose(dt,
        new Transform2d(AlignTransforms.LeftXL2, AlignTransforms.LeftYL2,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
    L3 = new SelectPose(dt,
        new Transform2d(AlignTransforms.LeftXL3, AlignTransforms.LeftYL3,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
    L4 = new SelectPose(dt,
        new Transform2d(AlignTransforms.LeftXL4, AlignTransforms.LeftYL4,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
    R1 = new SelectPose(dt,
        new Transform2d(AlignTransforms.RightXL1, AlignTransforms.RightYL1,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
    R2 = new SelectPose(dt,
        new Transform2d(AlignTransforms.RightXL2, AlignTransforms.RightYL2,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
    R3 = new SelectPose(dt,
        new Transform2d(AlignTransforms.RightXL3, AlignTransforms.RightYL3,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
    R4 = new SelectPose(dt,
        new Transform2d(AlignTransforms.RightXL4, AlignTransforms.RightYL4,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevator.selectedLevel == 1) {
      if (dt.selectedSide == 0) {
        drivePose = L1;
      } else {
        drivePose = R1;
      }
    } else if (elevator.selectedLevel == 2) {
      if (dt.selectedSide == 0) {
        drivePose = L2;
      } else {
        drivePose = R2;
      }
    } else if (elevator.selectedLevel == 3) {
      if (dt.selectedSide == 0) {
        drivePose = L3;
      } else {
        drivePose = R3;
      }
    } else if (elevator.selectedLevel == 4) {
      if (dt.selectedSide == 0) {
        drivePose = L4;
      } else {
        drivePose = R4;
      }
    } else {
      drivePose = R2;
    }
    setpointCommand = drivePose.returnPlannerSetpoint();
    setpointCommand.schedule();

  }

  @Override
  public void end(boolean interrupted) {
    setpointCommand.cancel();
    dt.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
  }

  @Override
  public boolean isFinished() {
    return setpointCommand.isFinished();
  }
}