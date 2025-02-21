package frc.robot.commands;

import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hopper;

public class IntegratedScoringCommand extends SequentialCommandGroup {
    Elevator m_Elevator;
    CoralArm m_coralArm;
    Hopper m_CoralHopper;
    double requiredElev;
    double requiredArm;
    double currArmPos;
    double currElevPos;

    public IntegratedScoringCommand(Elevator elev, CoralArm arm, Hopper hop, double elevatorPosition, double armPosition) {
        m_Elevator = elev;
        m_coralArm = arm;
        m_CoralHopper = hop;
        requiredElev = elevatorPosition;
        requiredArm = armPosition;
        addRequirements(elev, arm, hop);

        addCommands(
        // m_Elevator.setMotionMagicPosition(requiredElev),
        // m_coralArm.setMotionMagicPosition(requiredArm)
        );
    }

}
