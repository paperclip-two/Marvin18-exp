package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class ArmSetpoint extends Command {
    CoralArm m_arm;
    double m_setpoint;
    public ArmSetpoint(CoralArm arm, double setpoint) {
        m_arm = arm;
        m_setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        double position = m_arm.getPosition(); // for future use
    }

    @Override
    public void execute() {
        m_arm.ArmPosVoltage(m_setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.runVoltage(0);
    }
}
