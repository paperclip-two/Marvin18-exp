package frc.robot.commands.drivetrain.planner;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PlannerSetpointGenerator extends Command {
    
    public CommandSwerveDrivetrain mSwerve;
    public final Pose2d goalPose;
    private int currentID;
    private Pose2d poseToDrive;
    private boolean flipIt;

    private PPHolonomicDriveController mDriveController = Constants.AutoConstants.kDriveController;

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;
    private final SwerveRequest.ApplyRobotSpeeds mChassisSpeed;



    private final BooleanPublisher endTriggerLogger = NetworkTableInstance.getDefault().getTable("logging").getBooleanTopic("PositionPIDEndTrigger").publish();

    public PlannerSetpointGenerator(CommandSwerveDrivetrain mSwerve, Pose2d goalPose, boolean shouldFlip) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;
        flipIt = shouldFlip;
        mChassisSpeed = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
        

        endTrigger = new Trigger(
            () -> {
                Pose2d diff = mSwerve.getState().Pose.relativeTo(goalPose);

                var rotation = MathUtil.isNear(
                    0.0, 
                    diff.getRotation().getRotations(), 
                    Constants.AutoConstants.kRotationTolerance.getRotations(),
                    0.0,
                    1.0);

                var position = diff.getTranslation().getNorm() < Constants.AutoConstants.kPositionTolerance.in(Meters);
                var speed = mSwerve.getSpeedAsDouble() < Constants.AutoConstants.kSpeedTolerance.in(MetersPerSecond);

                boolean done = rotation && position && speed;
                return done;
            });

            endTriggerDebounced = endTrigger.debounce(Constants.AutoConstants.kEndTriggerDebounce.in(Seconds));
    }

    public static Command generateCommand(CommandSwerveDrivetrain swerve, Pose2d goalPose, Time timeout, boolean flipIt){
        return new PlannerSetpointGenerator(swerve, goalPose, flipIt).withTimeout(timeout).finallyDo(() -> {
            
            swerve.setControl(
                new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds())
            );


            swerve.setControl(new SwerveRequest.SwerveDriveBrake());
        });
       
    }

    @Override
    public void initialize() {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;
        if (flipIt == true) {
            goalState = goalState.flip();
        }
        endTriggerLogger.accept(endTrigger.getAsBoolean());

        mSwerve.setControl(
            mChassisSpeed.withSpeeds(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getState().Pose, goalState
            ))
        );
    }

    @Override
    public void end(boolean interrupted) {
        endTriggerLogger.accept(endTrigger.getAsBoolean());
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}