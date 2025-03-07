package frc.robot.commands.drivetrain.planner;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.util.EnumUtil;
import frc.robot.util.EnumUtil.POSE_ENUM;

public class NearestAlign extends Command {
    private final PhotonVision mVision;
    private final CommandSwerveDrivetrain mDrivetrain;
    private int currentID;
    private Pose2d poseToDrive;
    private boolean flipBool = false;
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    List <Pose2d> tagPoses = getPoseList(List.of(7,8,9,10,11,18,19,20,21,22));

    private final EnumUtil.SIDE sideToDrive;
    private PlannerSetpointGenerator plannerSetpointGenerator;

    public NearestAlign(PhotonVision vision, CommandSwerveDrivetrain dt, EnumUtil.SIDE side) {
        mVision = vision;
        mDrivetrain = dt;
        currentID = -1;
        sideToDrive = side;
        addRequirements(vision, dt);
    }


    public Pose2d getTagPoseFromID(int id) {
    return layout.getTagPose(id).get().toPose2d();
    }

    public int getIdFromPose(Pose2d ps) {
    int idToGet = -1;
    for (int i = 0; i < tagPoses.size(); i++) {
        if(tagPoses.get(i).equals(ps)) {
            return i + 1;
        }
    }

    return idToGet;
    }

    public List<Pose2d> getPoseList(List<Integer> tagIntegers) {
    List<Pose2d> tagPoses = new ArrayList<>();
    for (int i = 0; i < tagIntegers.size(); i++) {
        int tagInteger = tagIntegers.get(i);
        Pose2d tagPose = layout.getTagPose(tagInteger).get().toPose2d();
        tagPoses.add(tagPose);
    }
    return tagPoses;
 }

    public POSE_ENUM integratedPoseId(Pose2d currentDrivePose) {
    Pose2d nearest = getNearest(currentDrivePose);
    int id = getIdFromPose(nearest);
    return EnumUtil.getIdEnum(id);
    }

    public POSE_ENUM justGetEnumPose(Pose2d pose) {
        int id = getIdFromPose(pose);

        return EnumUtil.getIdEnum(id);
    }


 public Pose2d getNearest(Pose2d curr) {
    return curr.nearest(tagPoses);
 }

    @Override
    public void initialize() {
        //EnumUtil.POSE_ENUM currentPoseEnum = EnumUtil.getIdEnum();
        Pose2d curr = mDrivetrain.getState().Pose;
        Pose2d closest = curr.nearest(tagPoses);

        POSE_ENUM currentPoseEnum  = integratedPoseId(curr);
        flipBool = integratedPoseId(mDrivetrain.getState().Pose).makeRed();

        if (sideToDrive == EnumUtil.SIDE.LEFT) {
            poseToDrive = currentPoseEnum.getPoseLeft();
        }
        if (sideToDrive == EnumUtil.SIDE.RIGHT) {
            poseToDrive = currentPoseEnum.getPoseRight();
        }
        if (sideToDrive == EnumUtil.SIDE.ALGAE) {
            poseToDrive = currentPoseEnum.getPoseAlgae();
        }

        plannerSetpointGenerator = new PlannerSetpointGenerator(mDrivetrain, closest, flipBool);

    }

    @Override
    public void execute() {
        // No need to call generateCommand here, as the command is already scheduled in initialize()
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