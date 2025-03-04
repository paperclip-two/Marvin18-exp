package frc.robot.util;

import frc.robot.util.EnumUtil.ELEV;
import frc.robot.util.EnumUtil.SIDE;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EnumModes extends SubsystemBase {
    private SIDE currentSide = SIDE.ALGAE; // Default to CENTER, so that the robot is in a position to align left or right anyway.
    private ELEV currentElev = ELEV.LOAD; // default to loading position, where thingie is at bottom.

    public void setSide(SIDE side) {
        currentSide = side;
    }

    public void setElev(ELEV elv) {
        currentElev = elv;
    }

    public SIDE getSide() {
        return currentSide;
    }

    public ELEV getElev() {
        return currentElev;
    }

    // example implementation
/* 
 * 
 *     private void configureBindings() {
        // Left bumper sets side to LEFT
        new CommandXboxController(controller)
            .leftBumper()
            .onTrue(new InstantCommand(() -> driveModeSubsystem.setSide(SIDE.LEFT)));

        // Right bumper sets side to RIGHT
        new CommandXboxController(controller)
            .rightBumper()
            .onTrue(new InstantCommand(() -> driveModeSubsystem.setSide(SIDE.RIGHT)));
    }

    // in another one... do xboxController.x().whileTrue(new TagAssistedAlign(AlignMode.getSide()))
    // just pass in the current side in align mode
    // TODO: push current alignmode to SmartDashboard and also implement enum switch for elevator
 * 
*/
}