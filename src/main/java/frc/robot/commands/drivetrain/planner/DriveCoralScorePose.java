// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.planner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
//import frc.robot.constants.Constants.VisionFiducials;

import java.util.ArrayList;
import java.util.List;

public class DriveCoralScorePose extends Command {
  private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  private List<Pose2d> tagPoses = new ArrayList<>();
  private CommandSwerveDrivetrain dt;
  private Pose2d goalPose;
  private Transform2d trans;
  private PlannerSetpointGenerator plannerSetpointGenerator;

  /** Creates a new DriveCoralScorePose. */

  public DriveCoralScorePose(CommandSwerveDrivetrain drivetrain, Transform2d transform) {
    tagPoses = getPoseList(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20 ,21, 22));
    dt = drivetrain;
    trans = transform;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public List<Pose2d> getPoseList(List<Integer> tagIntegers) {
    List<Pose2d> tags = new ArrayList<>();
    for (int i = 0; i < tagIntegers.size(); i++) {
      int tagInteger = tagIntegers.get(i);
      Pose2d tagPose = fieldLayout.getTagPose(tagInteger).get().toPose2d();
      tags.add(tagPose);
    }
    return tags;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPose = dt.getState().Pose.nearest(tagPoses).plus(trans);

    plannerSetpointGenerator = new PlannerSetpointGenerator(dt, goalPose, false);

  }

  @Override
  public void execute() {
    // No need to call generateCommand here, as the command is already scheduled in
    // initialize()
    plannerSetpointGenerator.schedule();
  }

  @Override
  public boolean isFinished() {
    return plannerSetpointGenerator.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      plannerSetpointGenerator.cancel();
    }
  }
}