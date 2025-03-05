package frc.robot.util;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DynamicConstants;

import java.util.ArrayList;
import java.util.List;

public class EnumUtil {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    List <Pose2d> tagPoses = getPoseList(List.of(12,13,14,15,16,17,18,19,20,21));
    
    
public enum POSE_ENUM {

    // Blue Coral poses
    ID17(17, new Pose2d(4.16, 2.63, new Rotation2d(Math.toRadians(-30))), new Pose2d(3.84, 2.80, new Rotation2d(Math.toRadians(-30))), new Pose2d(), false),
    ID18(18, new Pose2d(3.1, 3.93, new Rotation2d(Math.toRadians(-85))), new Pose2d(3.14, 3.54, new Rotation2d(Math.toRadians(-85))), new Pose2d(), false),
    ID19(19, new Pose2d(3.81, 5.18, new Rotation2d(Math.toRadians(-155))), new Pose2d(3.52, 4.97, new Rotation2d(Math.toRadians(-150))), new Pose2d(), false),
    ID20(20, new Pose2d(5.10, 5.24, new Rotation2d(Math.toRadians(150))), new Pose2d(4.81, 5.40, new Rotation2d(150)), new Pose2d(), false),
    ID21(21, new Pose2d(5.87, 4.05, new Rotation2d(Math.toRadians(90))), new Pose2d(5.86, 4.42, new Rotation2d(Math.toRadians(90))), new Pose2d(), false),
    ID22(22, new Pose2d(5.48, 3.01, new Rotation2d(Math.toRadians(20))), new Pose2d(5.17, 2.84, new Rotation2d(Math.toRadians(20))), new Pose2d(), false),

    // Empty pose
    NOT_APPLICABLE(-1, new Pose2d(), new Pose2d(), new Pose2d(), false);


    private final int id;
    private final Pose2d poseLeft;
    private final Pose2d poseRight;
    private final Pose2d algaePose;
    private final boolean makeRed;

    // the first constructor allows us to assign a left and right. the second constructor
    // allows us to assign only one location, for things like algae and feeding. 
    POSE_ENUM(int id, Pose2d poseLeft, Pose2d poseRight, Pose2d algae, boolean makeRed) {
        this.id = id;
        this.poseLeft = poseLeft;
        this.poseRight = poseRight;
        this.algaePose = algae;
        this.makeRed = makeRed;
    }


    public int getId() {
        return id;
    }

    // if makeRed is true, activate flipping.
    public Pose2d getPoseLeft() {
        return poseLeft;
    }

    public Pose2d getPoseRight() {
        return poseRight;
    }

    public Pose2d getPoseAlgae() {
        return algaePose;
    }

    public boolean makeRed() {
        return makeRed;
    }
}



public static POSE_ENUM getIdEnum(int id) {
    if (id == 17) {
        return POSE_ENUM.ID17;
    }
    if (id == 18) {
        return POSE_ENUM.ID18;
    }
    if (id == 19) {
        return POSE_ENUM.ID19;
    }
    if (id == 20) {
        return POSE_ENUM.ID20;
    }
    if (id == 21) {
        return POSE_ENUM.ID21;
    }
    if (id == 22) {
        return POSE_ENUM.ID22;
    }
    else {
        return POSE_ENUM.NOT_APPLICABLE;
    }
}







public enum SIDE {
    LEFT,
    RIGHT,
    ALGAE
}

public enum ELEV {
    LOAD,
    ALGAE_GROUND,
    ALGAE_BOT,
    ALGAE_TOP,
    ALGAE_PROCESSOR,
    L1,
    L2,
    L3,
    L4,
    CLIMB
}



public static double getEnumSetpoint(ELEV level) {
    if (level == ELEV.ALGAE_GROUND) {
        return DynamicConstants.ElevatorSetpoints.elevAlgaeGround;
    }
    if (level == ELEV.ALGAE_BOT) {
        return DynamicConstants.ElevatorSetpoints.elevAlgaeBot;
    }
    if (level == ELEV.ALGAE_TOP) {
        return DynamicConstants.ElevatorSetpoints.elevAlgaeTop;
    }
    if (level == ELEV.ALGAE_PROCESSOR) {
        return DynamicConstants.ElevatorSetpoints.elevAlgaeTee;
    }
    if (level == ELEV.LOAD) {
        return DynamicConstants.ElevatorSetpoints.elevL1; // same as trough cuz its bottom
    }
    if (level == ELEV.L1) {
        return DynamicConstants.ElevatorSetpoints.elevL1;
    }
    if (level == ELEV.L2) {
        return DynamicConstants.ElevatorSetpoints.elevL2;
    }
    if (level == ELEV.L3) {
        return DynamicConstants.ElevatorSetpoints.elevL3;
    }
    if (level == ELEV.L3) {
        return DynamicConstants.ElevatorSetpoints.elevL4;
    }
    else {
        return 0; 
    } // just 0 if this enum doesn't exist
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

 public Pose2d getTagPoseFromID(int id) {
    return layout.getTagPose(id).get().toPose2d();
 }

 public int getIdFromPose(Pose2d ps) {
    int idToGet = -1;
    for (int i = 0; i < tagPoses.size(); i++) {
        Pose2d currPose = tagPoses.get(i);
        idToGet = i + 1;
        if (currPose == ps) {
            return idToGet;
        }
    }

    return idToGet;
 }

 public POSE_ENUM integratedPoseId(Pose2d currentDrivePose) {
    Pose2d nearest = getNearest(currentDrivePose);
    int id = getIdFromPose(nearest);
    return getIdEnum(id);
 }


 public Pose2d getNearest(Pose2d curr) {
    return curr.nearest(tagPoses);
 }



}