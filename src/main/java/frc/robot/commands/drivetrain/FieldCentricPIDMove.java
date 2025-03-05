package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldCentricPIDMove extends Command {
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    private final Constraints profile = new Constraints(0.1 , .1);
    private final SwerveRequest.FieldCentricFacingAngle fieldDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
                    SteerRequestType.MotionMagicExpo);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double currY;
    private double currX;
    private Rotation2d currRot;

    private Rotation2d initialRot;
    private Pose2d goalPose;


    private ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, profile);
    private ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, profile);
    private PIDController rotController = new PIDController(2, 0, 0);
    private double accuracy = 0.01;
    // Wrpa from -pi to ip

    public FieldCentricPIDMove(CommandSwerveDrivetrain dt, Pose2d pose) {
        m_drivetrainSubsystem = dt;
        goalPose = pose;
        addRequirements(dt);

    }

    @Override
    public void initialize() {
        initialRot = m_drivetrainSubsystem.getState().Pose.getRotation();
        xController.setGoal(goalPose.getX());
        xController.setTolerance(accuracy);
        yController.setGoal(goalPose.getY());
        yController.setTolerance(accuracy);
    }

    @Override
    public void execute() {
        currY = m_drivetrainSubsystem.getState().Pose.getY();
        currX = m_drivetrainSubsystem.getState().Pose.getX();
        currRot = m_drivetrainSubsystem.getState().Pose.getRotation();

        m_drivetrainSubsystem.setControl(fieldDrive
                .withVelocityX(-xController.calculate(currX))
                .withVelocityY(-yController.calculate(currY))
                .withTargetDirection(initialRot));
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setControl(fieldDrive
                .withVelocityX(0)
                .withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        return (yController.atGoal() && xController.atGoal());
    }
}