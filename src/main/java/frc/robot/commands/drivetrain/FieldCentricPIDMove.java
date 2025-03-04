package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldCentricPIDMove extends Command {
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    private final SwerveRequest.ApplyRobotSpeeds rc = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle fieldDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(
          SteerRequestType.MotionMagicExpo);
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double currY;
    private double currX;
    private Rotation2d currRot;

    private Rotation2d initialRot;
    private Pose2d goalPose;

    private PIDController xController = new PIDController(2, 0, 0);
    private PIDController yController = new PIDController(2, 0, 0);
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
        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(initialRot.getDegrees());
    }

    @Override
    public void execute() {
        currY = m_drivetrainSubsystem.getState().Pose.getY();
        currX = m_drivetrainSubsystem.getState().Pose.getX();
        currRot = m_drivetrainSubsystem.getState().Pose.getRotation();

        m_drivetrainSubsystem.setControl(fieldDrive
                .withVelocityX(yController.calculate(currY))
                .withVelocityY(-xController.calculate(currX))
                .withTargetDirection(initialRot));
    }

    @Override
    public boolean isFinished() {
        return (yController.calculate(currY) < accuracy && xController.calculate(currX) < accuracy);
    }
}