package frc.robot.commands.drivetrain;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignToTag extends Command {
    private final PhotonVision m_Vision;
    private final CommandSwerveDrivetrain m_Swerve;
    private final double holdDistance;
    private final SwerveRequest.FieldCentric m_alignRequest;
    private final ProfiledPIDController aimController;
    private final ProfiledPIDController rangeController;
    private final int cameraID;
    
        //From tunerconsts and robtocontainer.java
        private static final double MAX_AIM_VELOCITY = 1.5*Math.PI; // radd/s
        private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
        private static final double MAX_RANGE_VELOCITY = 1.0; // m/s
        private static final double MAX_RANGE_ACCELERATION = 0.5; // m/2^s
      
        // Todo - Tune later
        private static final double AIM_P = 0.1; //Proprotinal
        private static final double AIM_I = 0.01; //Gradual corretction
        private static final double AIM_D = 0.05; //Smooth oscilattions

        
        private static final double RANGE_P = 0.1;
        private static final double RANGE_I = 0.01;
        private static final double RANGE_D = 0.05;

    
        public AlignToTag(PhotonVision photon, CommandSwerveDrivetrain swerve, double holdDistance, int cameraID) {
            m_Vision = photon;
            m_Swerve = swerve;
            this.holdDistance = holdDistance;
            this.cameraID = cameraID;
        
        this.m_alignRequest = new SwerveRequest.FieldCentric().withDeadband(TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.1).withRotationalDeadband(0.1);

        aimController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
        rangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

        aimController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip

        addRequirements(m_Vision);
    }

    @Override
    public void initialize() {

       // create a state machine that stores the position of each tag on the field?
       rangeController.reset(m_Vision.getCoralTagDist()); //Init dist

        aimController.reset(0);
        
        aimController.setGoal(0); // tx=0 is centered
        rangeController.setGoal(holdDistance);
    }

    @Override
    public void execute() {
        double tx = m_Vision.getCoralYaw();
        double currCoralDist = m_Vision.getCoralTagDist();
        double rangeOutput = rangeController.calculate(currCoralDist);

        double rotationOutput = aimController.calculate(tx);

        Translation2d translation = new Translation2d(rangeOutput, 0);
                
        m_Swerve.setControl(m_alignRequest
            .withVelocityX(translation.getX())
            .withVelocityY(translation.getY())
            .withRotationalRate(rotationOutput));
    }
    
    @Override
    public void end(boolean interrupted) {
        m_Swerve.setControl(m_alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

}