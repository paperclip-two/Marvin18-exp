// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DynamicConstants.ElevatorSetpoints;
import frc.robot.constants.DynamicConstants.AlignTransforms;
import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAlign extends ParallelCommandGroup {
  Elevator elevator;
  CommandSwerveDrivetrain dt;
  DriveCoralScorePose drivePose;
  Command setpoint;

  /** Creates a new ElevatorAlign. */
  public ElevatorAlign(int side, CommandSwerveDrivetrain drive, Elevator elev) {
    elevator = elev;
    dt = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, dt);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (elevator.selectedLevel == 1) {
      setpoint = elevator.setMotionMagicPositionCommand(ElevatorSetpoints.elevL1);
      if (side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL1, AlignTransforms.LeftYL1,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL1, AlignTransforms.RightYL1,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else if (elevator.selectedLevel == 2) {
      setpoint = elevator.setMotionMagicPositionCommand(ElevatorSetpoints.elevL2);
      if (side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL2, AlignTransforms.LeftYL2,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL2, AlignTransforms.RightYL2,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else if (elevator.selectedLevel == 3) {
      setpoint = elevator.setMotionMagicPositionCommand(ElevatorSetpoints.elevL3);
      if (side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL3, AlignTransforms.LeftYL3,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL3, AlignTransforms.RightYL3,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else if (elevator.selectedLevel == 4) {
      setpoint = elevator.setMotionMagicPositionCommand(ElevatorSetpoints.elevL4);
      if (side == 0) {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.LeftXL4, AlignTransforms.LeftYL4,
            Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
      } else {
        drivePose = new DriveCoralScorePose(dt, new Transform2d(AlignTransforms.RightXL4, AlignTransforms.RightYL4,
            Rotation2d.fromDegrees(AlignTransforms.RightRot)));
      }
    } else {
      elevator.zeroElevatorCommand();

    }

    addCommands(drivePose, setpoint);

  }
}