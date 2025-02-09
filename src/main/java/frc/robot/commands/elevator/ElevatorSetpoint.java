package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorSetpoint extends Command {
    Elevator m_Elevator;
    double m_setpoint;
    public ElevatorSetpoint(Elevator elevator, double setpoint) {
        m_Elevator = elevator;
        m_setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        double position = m_Elevator.getAdjustedRotations(); // for future use
    }

    @Override
    public void execute() {
        m_Elevator.elevatorPositionVoltage(m_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.runVoltageRequest(0);
    }
}
