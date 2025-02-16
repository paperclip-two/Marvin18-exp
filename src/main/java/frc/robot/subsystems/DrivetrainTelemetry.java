package frc.robot.subsystems;

import org.opencv.photo.Photo;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainTelemetry extends SubsystemBase {
    private CommandSwerveDrivetrain dt;
    private PhotonVision m_reef;

    public DrivetrainTelemetry(CommandSwerveDrivetrain thedrivetrain, PhotonVision theReefCam) {
        dt = thedrivetrain;
        m_reef = theReefCam;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("DrivetrainStates/PoseX", dt.getState().Pose.getMeasureX().baseUnitMagnitude());
        SmartDashboard.putNumber("DrivetrainStates/PoseY", dt.getState().Pose.getMeasureY().baseUnitMagnitude());
        SmartDashboard.putNumber("DrivetrainStates/RotationDegrees", dt.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("DrivetrainStates/RotationRadians", dt.getState().Pose.getRotation().getRadians());
        SmartDashboard.putNumber("TagYaw", m_reef.getTagYaw());
    }
}
