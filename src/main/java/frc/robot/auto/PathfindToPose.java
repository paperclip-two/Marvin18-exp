package frc.robot.auto;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PathfindToPose extends Command {
    private final CommandSwerveDrivetrain dtsub;
    private Command command;
    private Pose2d pose;
    public PathfindToPose(CommandSwerveDrivetrain sub) {
        dtsub = sub;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = dtsub.getState().Pose;
        Translation2d currtranslate = currentPose.getTranslation();

        command = AutoBuilder.pathfindToPose(new Pose2d(15, 2, Rotation2d.fromDegrees(0)), new PathConstraints(LinearVelocity.ofBaseUnits(1.0, MetersPerSecond), LinearAcceleration.ofBaseUnits(0.5, MetersPerSecondPerSecond), AngularVelocity.ofBaseUnits(360, DegreesPerSecond), AngularAcceleration.ofBaseUnits(540, DegreesPerSecondPerSecond)));
        
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
