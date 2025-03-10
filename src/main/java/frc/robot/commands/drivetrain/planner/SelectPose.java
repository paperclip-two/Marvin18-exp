package frc.robot.commands.drivetrain.planner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.CommandSwerveDrivetrain;

//import frc.robot.constants.Constants.VisionFiducials;

import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

public class SelectPose {
    private static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private List<Pose2d> tagPoses = new ArrayList<>();
    private CommandSwerveDrivetrain dt;
    private Pose2d goalPose;
    private Transform2d trans;
    private Command drive;

    /** Creates a new DriveCoralScorePose. */

    public SelectPose(CommandSwerveDrivetrain drivetrain, Transform2d transform) {
        tagPoses = getPoseList(List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22));
        dt = drivetrain;
        trans = transform;
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

    public Command returnPlannerSetpoint() {
        goalPose = dt.getState().Pose.nearest(tagPoses).plus(trans);

        drive = PlannerSetpointGenerator.generateCommand(dt, goalPose, Time.ofBaseUnits(3, Seconds), false);
        return drive;
    }

}
