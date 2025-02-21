// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.constants.Constants.ElevatorSetpointConfigs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SafeMoveArm extends Command {
  Elevator m_Elevator;
  CoralArm m_CoralArm;
  double elevatorSafePosition = 6;
  double m_position;
  /** Creates a new SafeMoveArm. */
  public SafeMoveArm(Elevator elevator, CoralArm coralArm, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Elevator, m_CoralArm);
    m_position = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_Elevator.getPositionNormal() > ElevatorSetpointConfigs.ELEVATOR_SAFE_POSITION) {
      m_CoralArm.setMotionMagicPositionDB(m_position);
      }
      ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
