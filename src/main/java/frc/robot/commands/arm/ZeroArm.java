package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CoralArm;

public class ZeroArm extends Command {
    CoralArm m_arm;
    double m_setpoint;
    public ZeroArm(CoralArm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        double position = m_arm.getPosition(); // for future use
    }

    @Override
    public void execute() {
        m_arm.ArmPosVoltage(0);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.runVoltage(0);
    }
}
