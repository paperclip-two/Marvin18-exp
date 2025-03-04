package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorSetpoint extends Command {
    Elevator m_Elevator;
    double m_setpoint;
    boolean mDone;
    public ElevatorSetpoint(Elevator elevator, double setpoint, boolean done) {
        m_Elevator = elevator;
        m_setpoint = setpoint;
        mDone = done;
        addRequirements(elevator);
        
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //m_Elevator.setMotionMagic(() -> m_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        //m_Elevator.runVoltageRequest(0);
    }
    
    @Override
    public boolean isFinished() {
        return (m_Elevator.isNear(m_setpoint) || m_Elevator.getLimit() || mDone);
    }
}
