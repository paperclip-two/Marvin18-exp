package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;

public class AlignToTag extends Command{
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    private final SwerveRequest.RobotCentric mAltAlign = new SwerveRequest.RobotCentric();

    private PhotonVision mCamera;
    
    public AlignToTag(CommandSwerveDrivetrain drivetrainSubsystem, PhotonVision camera) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        mCamera = camera;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {

        m_drivetrainSubsystem.setControl(
            mAltAlign.withVelocityX(-mCamera.getTagY()*2)
                       .withVelocityY(0) // Drive left with negative X (left)
                       .withRotationalRate(mCamera.getTagYaw()*1) // Drive counterclockwise with
                                                                // negative X (left)
        ); 
    }

    // @Override
    // public boolean isFinished(){
    //     return mLimelight.hasTrapTag() && (Math.abs(mLimelight.getTrapTagX()) < 0.04 && Math.abs(mLimelight.getTrapTagRot()) < 10);
    // }
}
