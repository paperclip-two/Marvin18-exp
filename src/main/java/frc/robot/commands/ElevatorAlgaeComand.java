// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorAlgaeComand extends Command {
  private final Elevator m_elevator;
  private final Algae m_algae;
  private final Timer m_time;

  /** Creates a new ElevatorAlgaeComand. */
  public ElevatorAlgaeComand(Elevator elevator, Algae algae, Timer time) {
    m_elevator = elevator;
    m_algae = algae;
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_algae);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_algae.setPercentage(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.advanceRotations(.25);
    Timer.delay(1);
    m_algae.setPercentage(.5);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
