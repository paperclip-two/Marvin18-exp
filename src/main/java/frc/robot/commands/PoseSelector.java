// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants.AlignTransforms;
import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoseSelector extends Command {
  Elevator elevator;
  CommandSwerveDrivetrain dt;
  DriveCoralScorePose drivePose;

  /** Creates a new ElevatorAlign. */
  public PoseSelector(CommandSwerveDrivetrain drive, Elevator elev) {
    elevator = elev;
    dt = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }
  // Add your commands in the addCommands() call, e.g.
  // addCommands(new FooCommand(), new BarCommand());

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevator.selectedLevel == 1) {
      if (dt.side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL1, AlignTransforms.LeftYL1,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL1, AlignTransforms.RightYL1,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else if (elevator.selectedLevel == 2) {
      if (dt.side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL2, AlignTransforms.LeftYL2,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL2, AlignTransforms.RightYL2,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else if (elevator.selectedLevel == 3) {
      if (dt.side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL3, AlignTransforms.LeftYL3,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL3, AlignTransforms.RightYL3,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else if (elevator.selectedLevel == 4) {
      if (dt.side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL4, AlignTransforms.LeftYL4,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL4, AlignTransforms.RightYL4,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else {
      drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL1, AlignTransforms.LeftYL1,
          Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivePose.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivePose.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivePose.isFinished();
  }
}