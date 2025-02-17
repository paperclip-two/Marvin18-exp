package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevator extends Command {
    Elevator m_Elevator;
    double m_setpoint;
    public ZeroElevator(Elevator elevator) {
        m_Elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        double position = m_Elevator.getAdjustedRotations(); // for future use
    }

    @Override
    public void execute() {
    //    m_Elevator.zeroElevatorWithLimit();
    }

    @Override
    public void end(boolean interrupted) {
        m_Elevator.runVoltageRequest(0);
    }
}
