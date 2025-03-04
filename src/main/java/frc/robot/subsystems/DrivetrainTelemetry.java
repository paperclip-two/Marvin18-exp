package frc.robot.subsystems;

import org.opencv.photo.Photo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainTelemetry extends SubsystemBase {
    private CommandSwerveDrivetrain dt;
    private PhotonVision m_reef;

    public DrivetrainTelemetry(CommandSwerveDrivetrain thedrivetrain, PhotonVision theReefCam) {
        dt = thedrivetrain;
        m_reef = theReefCam;
    }

    public boolean isRedAlliance(Alliance all) {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(all);
    }

    

   StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("AdvantageKitPose", Pose2d.struct).publish();
   StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
  .getStructArrayTopic("AdvantageKitPoseArray", Pose2d.struct).publish();



    @Override
    public void periodic() {
        SmartDashboard.putNumber("DrivetrainStates/PoseX", dt.getState().Pose.getMeasureX().baseUnitMagnitude());
        SmartDashboard.putNumber("DrivetrainStates/PoseY", dt.getState().Pose.getMeasureY().baseUnitMagnitude());
        SmartDashboard.putNumber("DrivetrainStates/RotationDegrees", dt.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("DrivetrainStates/RotationRadians", dt.getState().Pose.getRotation().getRadians());


        publisher.set(dt.getState().Pose);
        arrayPublisher.set(new Pose2d[] {dt.getState().Pose});
        SmartDashboard.putNumber("Last Fiducial", m_reef.getFiducialId());
    }
}
