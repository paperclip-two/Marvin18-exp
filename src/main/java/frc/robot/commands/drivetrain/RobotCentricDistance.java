package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.constants.TunerConstants;

public class RobotCentricDistance extends Command {
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    public Pose2d initial_pose;
    private final double m_distance;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagicExpo);

    public RobotCentricDistance(CommandSwerveDrivetrain drivetrainSubsystem, double speed, double distance) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        m_distance = distance;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        initial_pose = m_drivetrainSubsystem.getState().Pose;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.setControl(robotDrive.withVelocityX(0).withVelocityY(MaxSpeed * .1).withRotationalRate(0));
    }

    

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.setControl(robotDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return m_drivetrainSubsystem.getState().Pose.getTranslation().getDistance(initial_pose.getTranslation()) > m_distance;
    }

}