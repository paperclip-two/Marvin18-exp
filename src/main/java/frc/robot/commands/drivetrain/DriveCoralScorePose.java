// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveCoralScorePose extends Command {
  private static AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  private List<Pose2d> tagPoses = new ArrayList<>();

  /** Creates a new DriveCoralScorePose. */
  
  public DriveCoralScorePose(CommandSwerveDrivetrain drivetrain, Transform2d transform) {
    tagPoses = getPoseList(List.of(11, 12, 13));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public List<Pose2d> getPoseList(List<Integer> tagIntegers) {
    for (int i = 0; i < tagIntegers.size(); i++) {
      int tagInteger = tagIntegers.get(i);
      Pose2d tagPose = fieldLayout.getTagPose(tagInteger).get().toPose2d();
      System.out.println("Tag " + tagInteger + " is at " + tagPose.getX() + ", "
          + tagPose.getY() + ", " + tagPose.getRotation().getDegrees());
      tagPoses.add(tagPose);
    }
    return tagPoses;
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
