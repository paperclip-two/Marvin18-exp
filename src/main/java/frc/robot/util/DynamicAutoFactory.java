package frc.robot.util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.constants.Constants;
import frc.robot.constants.DynamicConstants;
// import frc.robot.constants.Constants.ElevatorSetpointConfigs;
import frc.robot.constants.DynamicConstants.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Meters;
// import static edu.wpi.first.units.Units.Millimeter;
// import static edu.wpi.first.units.Units.Radians;

import java.util.Set;

public class DynamicAutoFactory {

    private Coral coralSub;
    private Elevator elevSub;
    private Algae algSub;

    public Trigger hasScoredTrigger = new Trigger(this::isCoralInArm).negate().debounce(0.01);

    private DynaPreset lastInputtedPreset = DynaPreset.L4;

    public DynamicAutoFactory(Coral cor, Elevator elev, Algae alg) {
        coralSub = cor;
        elevSub = elev;
        algSub = alg;


        var tab = Shuffleboard.getTab("dynamicsLogging");
        tab.addBoolean("elevStowed", this::isElevStowed);
        tab.add("CommandScheduler", CommandScheduler.getInstance());




    }

    private record DynamicsSetpoint(double heightMeters, Rotation2d armAngle) {
    }

    public enum DynaPreset {
        LOAD(0, Rotation2d.fromDegrees(234.4421)),
        L1(DynamicConstants.ElevatorSetpoints.elevL1, Rotation2d.fromDegrees(47.900)),
        L2(DynamicConstants.ElevatorSetpoints.elevL2, Rotation2d.fromDegrees(47.900)),
        L3(DynamicConstants.ElevatorSetpoints.elevL3, Rotation2d.fromDegrees(58.10311200000001)),
        L4(DynamicConstants.ElevatorSetpoints.elevL4, Rotation2d.fromDegrees(14.33)),
        CLIMB(0.0, Rotation2d.fromDegrees(270+40)); // find climb setpoint

        private final DynamicsSetpoint setpoint;

        private DynaPreset(double meters, Rotation2d angle) {
            this.setpoint = new DynamicsSetpoint(meters, angle);
        }
    }

    //#region Composite Commands


    //TODO swap out target with actual position


    private boolean isElevAtSetpoint(double setpoint){
        System.out.println(Math.abs(setpoint - elevSub.getAdjustedRotations()));
        return Math.abs(setpoint - elevSub.getAdjustedRotations()) < 2 * 8.5; // MUST BE ELEVATOR HEIGHT TOLERANCE.
    }


    /**
     * @return Whether the arm is below the horizon and the elevator is too low to allow movement
     */


    /**
     * @return Whether the arm is below the horizon and the elevator is too low to allow movement
     */


    private boolean isElevStowed(){
        return  elevSub.getAdjustedRotations() + 0 < 0; // first one is the elevator height tolerance, second one is minimum safe height. FIX
    }

    private boolean isCoralInArm(){
        // return false;
        return coralSub.hasCoral();
    }



 

    /**
     * Moves the elevator first before moving the arm
     */
    private Command elevatorPriorityMove(DynamicsSetpoint setpoint){
        return Commands.parallel(
            elevSub.setMotionMagicPositionCommand(setpoint.heightMeters),
            Commands.waitUntil(() -> isElevAtSetpoint(setpoint.heightMeters)).withTimeout(2).andThen(
                coralSub.runIntake(-1)
            )
        );
    }

    public Command waitUntilPreset(DynaPreset setpoint){
        return Commands.waitUntil(() -> {
            return isElevAtSetpoint(setpoint.setpoint.heightMeters);
        });
    }

    /**
     * Moves the arm first before moving the elevator
     */

    //#endregion

    //#region small Commands

    public Command scoreHeight(DynaPreset scoringPoint){
        return Commands.sequence(
            elevatorPriorityMove(scoringPoint.setpoint)
        );
    }




    //#endregion


    public Command gotoScore(DynaPreset scorePreset){
        return scoreHeight(scorePreset)
        .withName("Goto " + scorePreset);
    }

    public Command gotoLastInputtedScore() {
        return Commands.defer(() -> gotoScore(lastInputtedPreset), Set.of())
        .withName("Goto Last");
    }

    /**
     * Runs gotoScore() and saves the input so we can automatically go there next time
     */
    public Command operatorScore(DynaPreset preset) {
        return Commands.runOnce(() -> lastInputtedPreset = preset)
                       .andThen(gotoScore(preset))
                       .withName("Operator Goto " + preset);
    }

    public Command score(){
        return Commands.deadline(
            Commands.waitUntil(
                hasScoredTrigger
            ).withTimeout(1.0),
            coralSub.runIntake(-1)
        ).andThen(coralSub.runIntake(0.2))
        .withName("Score");
    }

    public Command autoScore(DynaPreset scoringLocation){
        return Commands.sequence(
            Commands.waitUntil(() -> 
                isElevAtSetpoint(scoringLocation.setpoint.heightMeters)
            ),
            score()
        )
        .withName("Autonomous Score");
    }

    /**
     * Starts the intake immediately and ends the command once the funnel or manipulator LaserCAN detects coral. This will not stop the intake
     */
 // todo: add coral blockage check here

    /**
     * Intakes if there is no coral in the manipulator
     * @return a Command that will do the above actions 
     */
}