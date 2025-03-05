package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.util.EnumUtil;

public class SetpointEnum extends Command {
    private final Elevator mElevator;
    private double setpoint;
    private boolean mDone;

    private final EnumUtil.ELEV elevLevel;
    public SetpointEnum(Elevator elev, EnumUtil.ELEV elv, boolean done) {
        mElevator = elev;
        mDone = done;
        elevLevel = elv;
        addRequirements(mElevator);
    }

    @Override
    public void initialize() {
        setpoint = EnumUtil.getEnumSetpoint(elevLevel);

    }

    @Override
    public void execute() {
        mElevator.setMotionMagic(() -> setpoint);       
    }

    @Override
    public void end(boolean interrupted) {
        mElevator.runVoltageRequest(0);
    }
    
    @Override
    public boolean isFinished() {
        return (mElevator.isNear(setpoint) || mElevator.getLimit() || mDone);
    }
}