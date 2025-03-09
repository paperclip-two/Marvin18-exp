// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.drivetrain.planner.DriveCoralScorePose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.DynamicConstants.AlignTransforms;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PoseSelector extends InstantCommand {
  Elevator elevator;
  CommandSwerveDrivetrain dt;
  DriveCoralScorePose drivePose;
  private DriveCoralScorePose L1 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.LeftXL1, AlignTransforms.LeftYL1,
          Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
  private DriveCoralScorePose L2 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.LeftXL2, AlignTransforms.LeftYL2,
          Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
  private DriveCoralScorePose L3 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.LeftXL3, AlignTransforms.LeftYL3,
          Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
  private DriveCoralScorePose L4 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.LeftXL4, AlignTransforms.LeftYL4,
          Rotation2d.fromDegrees(AlignTransforms.LeftRot)));
  private DriveCoralScorePose R1 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.RightXL1, AlignTransforms.RightYL1,
          Rotation2d.fromDegrees(AlignTransforms.RightRot)));
  private DriveCoralScorePose R2 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.RightXL2, AlignTransforms.RightYL2,
          Rotation2d.fromDegrees(AlignTransforms.RightRot)));
  private DriveCoralScorePose R3 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.RightXL3, AlignTransforms.RightYL3,
          Rotation2d.fromDegrees(AlignTransforms.RightRot)));
  private DriveCoralScorePose R4 = new DriveCoralScorePose(dt,
      new Transform2d(AlignTransforms.RightXL4, AlignTransforms.RightYL4,
          Rotation2d.fromDegrees(AlignTransforms.RightRot)));

  /** Creates a new ElevatorAlign. */
  public PoseSelector(CommandSwerveDrivetrain drive, Elevator elev) {
    elevator = elev;
    dt = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (elevator.selectedLevel == 1) {
      if (dt.side == 0) {
        drivePose = L1;
      } else {
        drivePose = R1;
      }
    } else if (elevator.selectedLevel == 2) {
      if (dt.side == 0) {
        drivePose = L2;
      } else {
        drivePose = R2;
      }
    } else if (elevator.selectedLevel == 3) {
      if (dt.side == 0) {
        drivePose = L3;
      } else {
        drivePose = R3;
      }
    } else if (elevator.selectedLevel == 4) {
      if (dt.side == 0) {
        drivePose = L4;
      } else {
        drivePose = R4;
      }
    } else {
      drivePose = R2;
    }
    drivePose.schedule();
  }

}