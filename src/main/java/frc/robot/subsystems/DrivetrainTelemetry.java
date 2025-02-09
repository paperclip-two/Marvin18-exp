package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainTelemetry extends SubsystemBase {
    private CommandSwerveDrivetrain dt;

    public DrivetrainTelemetry(CommandSwerveDrivetrain thedrivetrain) {
        dt = thedrivetrain;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("DrivetrainStates/PoseX", dt.getState().Pose.getMeasureX().baseUnitMagnitude());
        SmartDashboard.putNumber("DrivetrainStates/PoseY", dt.getState().Pose.getMeasureY().baseUnitMagnitude());
        SmartDashboard.putNumber("DrivetrainStates/RotationDegrees", dt.getState().Pose.getRotation().getDegrees());
        SmartDashboard.putNumber("DrivetrainStates/RotationRadians", dt.getState().Pose.getRotation().getRadians());
    }
}
