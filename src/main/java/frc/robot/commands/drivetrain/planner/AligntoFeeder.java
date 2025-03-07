// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.planner;

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
public class AligntoFeeder extends Command {
  /** Creates a new AligntoFeeder. */
  private static AprilTagFieldLayout fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
  private List<Pose2d> tagPoses = new ArrayList<>();
  private List<Pose2d> slotPoses = new ArrayList<>();
  private CommandSwerveDrivetrain dt;
  private Pose2d goalPose;
  private List<Transform2d> trans;
  private PlannerSetpointGenerator plannerSetpointGenerator;

  /** Creates a new DriveCoralScorePose. */

  public AligntoFeeder(CommandSwerveDrivetrain drivetrain, List<Transform2d> transforms) {
    tagPoses = getPoseList(List.of(1, 2, 12, 13));
    slotPoses = createLoadingSlots(tagPoses, transforms);
    dt = drivetrain;
    trans = transforms;
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

  public List<Pose2d> createLoadingSlots(List<Pose2d> tagPoses, List<Transform2d> transform2ds) {
    List<Pose2d> loadingSlots = new ArrayList<>();
    for (int i = 0; i < tagPoses.size(); i++) {
      Pose2d tagPose = tagPoses.get(i);
      for (int j = 0; j < transform2ds.size(); j++) {
        Transform2d transform = transform2ds.get(j);
        Pose2d loadingSlot = tagPose.plus(transform);
        loadingSlots.add(loadingSlot);
      }
    }
    return loadingSlots;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goalPose = dt.getState().Pose.nearest(slotPoses);
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
