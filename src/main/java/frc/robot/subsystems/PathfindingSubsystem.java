package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathfindingSubsystem extends SubsystemBase {
    private final Pose2d targetPose;

// Create the constraints to use while pathfinding
 private final PathConstraints constraint; 
 private final CommandSwerveDrivetrain dt;
 SwerveRequest.Idle idle;
 
    public PathfindingSubsystem(Pose2d desiredPose, PathConstraints constraints, CommandSwerveDrivetrain drive) {
        targetPose = desiredPose;
        constraint = constraints;
        dt = drive;
        
    }

    public Command pathfinder() {
        return runEnd(() -> {
            AutoBuilder.pathfindToPose(targetPose, constraint);
        }, () -> {
            dt.setControl(idle);
        });
    }





}
