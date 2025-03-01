package frc.robot.commands.drivetrain;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AutoAlignment extends SequentialCommandGroup {
    /**
     * creates a precise auto-alignment command
     * NOTE: AutoBuilder must be configured!
     * the command has two steps:
     * 1. path-find to the target pose, roughly
     * 2. accurate auto alignment
     * */
    public AutoAlignment(
            PathConstraints constraints,
            HolonomicDriveController holonomicDriveController,
            Supplier<Pose2d> robotPoseSupplier,
            Consumer<ChassisSpeeds> robotRelativeSpeedsOutput,
            Subsystem driveSubsystem,
            Pose2d targetPose
    ) {
        /* tolerance for the precise approach */
        holonomicDriveController.setTolerance(new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(1)));
        final Command
                pathFindToTargetRough = AutoBuilder.pathfindToPose(targetPose, constraints, 0.5),
                preciseAlignment = new FunctionalCommand(
                        () -> {},
                        () -> robotRelativeSpeedsOutput.accept(holonomicDriveController.calculate(
                                robotPoseSupplier.get(),
                                targetPose,
                                0,
                                targetPose.getRotation()
                        )),
                        (interrupted) ->
                                robotRelativeSpeedsOutput.accept(new ChassisSpeeds()),
                        holonomicDriveController::atReference
                        );

        super.addCommands(pathFindToTargetRough);
        super.addCommands(preciseAlignment);

        super.addRequirements(driveSubsystem);
    }
}