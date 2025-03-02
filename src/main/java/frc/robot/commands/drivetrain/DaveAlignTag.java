package frc.robot.commands.drivetrain;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class DaveAlignTag extends Command {
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    private final SwerveRequest.RobotCentric mAltAlign = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
                    SteerRequestType.MotionMagicExpo);

    private PhotonVision mCamera;
    private Pose2d mtolerance;

    public DaveAlignTag(CommandSwerveDrivetrain drivetrainSubsystem, PhotonVision camera) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        mCamera = camera;
        mtolerance = new Pose2d(0.03, 0.03, Rotation2d.fromDegrees(1));
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double xoffset = 0.0;
        double yoffset = 0.0;
        double yawoffset = 0.0;
        if (Math.abs(mCamera.getTagXDist() - 1) > mtolerance.getY()) {
            yoffset = mCamera.getTagXDist() - 1;
        }
        if (Math.abs(mCamera.getTagY()) > mtolerance.getY()) {
            xoffset = -mCamera.getTagY();
        }
        if (Math.abs(mCamera.getTagYaw()) > mtolerance.getRotation().getRadians()) {
            yawoffset = mCamera.getTagYaw();
        }
        m_drivetrainSubsystem.setControl(mAltAlign.withVelocityX(xoffset*2).withVelocityY(yoffset*2).withRotationalRate(yawoffset*1)); // Drive
                                                                                                                                
    }
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setControl(mAltAlign.withVelocityX(0).withVelocityY(0).withRotationalRate(0));}

    @Override
    public boolean isFinished() {
        if (Math.abs(mCamera.getTagY()) < mtolerance.getX() && (Math.abs(mCamera.getTagXDist()) -1) < mtolerance.getY()
                && Math.abs(mCamera.getTagYaw()) < mtolerance.getRotation().getRadians()) {
            return true;
        }
        else {
            return false;
        }
    }
}
