package frc.robot.commands.drivetrain.planner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.util.EnumUtil;

public class TagAssistedAlign extends Command {
    private final PhotonVision mVision;
    private final CommandSwerveDrivetrain mDrivetrain;
    private int currentID;
    private Pose2d poseToDrive;
    private boolean flipBool = false;

    private final EnumUtil.SIDE sideToDrive;
    private PlannerSetpointGenerator plannerSetpointGenerator;

    public TagAssistedAlign(PhotonVision vision, CommandSwerveDrivetrain dt, EnumUtil.SIDE side) {
        mVision = vision;
        mDrivetrain = dt;
        currentID = -1;
        sideToDrive = side;
        addRequirements(vision, dt);
    }

    @Override
    public void initialize() {
        currentID = mVision.getFiducialId();
        EnumUtil.POSE_ENUM currentPoseEnum = EnumUtil.getIdEnum(currentID);
        flipBool = currentPoseEnum.makeRed();

        if (sideToDrive == EnumUtil.SIDE.LEFT) {
            poseToDrive = currentPoseEnum.getPoseLeft();
        }
        if (sideToDrive == EnumUtil.SIDE.RIGHT) {
            poseToDrive = currentPoseEnum.getPoseRight();
        }
        if (sideToDrive == EnumUtil.SIDE.ALGAE) {
            poseToDrive = currentPoseEnum.getPoseAlgae();
        }

        plannerSetpointGenerator = new PlannerSetpointGenerator(mDrivetrain, poseToDrive, flipBool);

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