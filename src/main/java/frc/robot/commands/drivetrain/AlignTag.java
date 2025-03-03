package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class AlignTag extends Command {
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    private final SwerveRequest.RobotCentric mAltAlign = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
                    SteerRequestType.MotionMagicExpo);

    private PhotonVision mCamera;
    private Pose2d mtolerance;
    private double offset;
    private double angle;

    public AlignTag(CommandSwerveDrivetrain drivetrainSubsystem, PhotonVision camera, double horizontal) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        mCamera = camera;
        mtolerance = new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(0.5));
        offset = horizontal;
        angle = Math.tan(1 / offset);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double xoffset = 0.0;
        double yoffset = 0.0;
        double yawoffset = 0.0;
        if (Math.abs(mCamera.getTagXDist() - 0.45) > mtolerance.getY()) {
            yoffset = mCamera.getTagXDist() - 0.45;
        }
        if (Math.abs(mCamera.getTagY() - offset) > mtolerance.getY()) {
            xoffset = -(mCamera.getTagY() - offset);
        }
        if (Math.abs(mCamera.getTagYaw()) > mtolerance.getRotation().getRadians()) {
            yawoffset = mCamera.getTagYaw();
        }
        m_drivetrainSubsystem.setControl(
                mAltAlign.withVelocityX(xoffset * 2).withVelocityY(yoffset * 2).withRotationalRate(yawoffset * 1)); // Drive

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setControl(mAltAlign.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(mCamera.getTagY() - offset) < mtolerance.getX()
                && (Math.abs(mCamera.getTagXDist()) - 0.45) < mtolerance.getY()
                && Math.abs(mCamera.getTagYaw()) < mtolerance.getRotation().getRadians()) {
            return true;
        } else {
            return false;
        }
    }
}
