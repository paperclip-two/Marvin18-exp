package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class SafeArmTwo extends SequentialCommandGroup {
  Elevator m_Elevator;
  CoralArm m_CoralArm;
  double elevatorSafePosition = 6;
  double mInitialElevatorPosition;
  double mInitialArmPosition;
  double mTargetElevatorPosition;
  double mTargetArmPosition;

  public SafeArmTwo(Elevator elevator, CoralArm arm, double elevatorTarget, double armTarget) {
    m_Elevator = elevator;
    m_CoralArm = arm;
    mTargetElevatorPosition = elevatorTarget;
    mTargetArmPosition = armTarget;

    addRequirements(elevator, arm);

    addCommands(
      //  new ArmAndElevator(m_Elevator,  m_CoralArm, mTargetElevatorPosition, mTargetArmPosition).andThen(null)
    );

  }


}
