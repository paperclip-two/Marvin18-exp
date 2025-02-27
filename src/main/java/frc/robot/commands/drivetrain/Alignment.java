package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import org.photonvision.proto.Photon;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;


public class Alignment extends Command{
    private final CommandSwerveDrivetrain m_drivetrainSubsystem;
    private final SwerveRequest.ApplyRobotSpeeds rc = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.RobotCentric mAltAlign = new SwerveRequest.RobotCentric();
    private PhotonVision mPhotonVision;
    private double currY;
    private double currX;
    private double currRot;
    private static final double MAX_AIM_VELOCITY = 1.5*Math.PI; // radd/s
    private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2

    private ProfiledPIDController rangeController = new ProfiledPIDController(2.3, 0, 0.01, new TrapezoidProfile.Constraints(0.5, 0.5));
    private ProfiledPIDController distController = new ProfiledPIDController(1, 0, 0.01, new TrapezoidProfile.Constraints(0.2, 0.5));

    private ProfiledPIDController aimController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));

     //Wrpa from -pi to ip

    public Alignment(CommandSwerveDrivetrain dt, PhotonVision pv) {
        m_drivetrainSubsystem = dt;
        mPhotonVision = pv;

        addRequirements(dt, pv);
        aimController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        currY = mPhotonVision.getTagY();
        currX = mPhotonVision.getTagXDist();
        
        currRot = mPhotonVision.getTagYaw();
        aimController.reset(0);
        distController.reset(0);
        
        aimController.setGoal(0); // tx=0 is centered
        rangeController.reset(0);
        rangeController.setGoal(currY + 0.3);
        distController.setGoal(currX);



    }

    @Override
    public void execute() {

        double rangeOutput = rangeController.calculate(currY);
        double distOutput = aimController.calculate(currX);
        double aimOutput = aimController.calculate(currRot);

        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
    //    m_drivetrainSubsystem.setControl(
    //        rc.withSpeeds(
       //         new ChassisSpeeds(
      //              rangeOutput,
       //             0,
        //            aimOutput)
       //     )
       // );
       m_drivetrainSubsystem.setControl(mAltAlign
       .withVelocityX(rangeOutput)
       .withVelocityY(currX)
       .withRotationalRate(0)); // offset
    }

    // @Override
    // public boolean isFinished(){
    //     return mLimelight.hasTrapTag() && (Math.abs(mLimelight.getTrapTagX()) < 0.04 && Math.abs(mLimelight.getTrapTagRot()) < 10);
    // }
}