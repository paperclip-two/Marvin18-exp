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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

// make another command just for ranging, and one just for aligning
public class AlignCommand extends Command {
    private final PhotonVision m_Vision;
    private final CommandSwerveDrivetrain m_Swerve;
    private final double holdDistance;
    private final SwerveRequest.ApplyRobotSpeeds mChassisSpeed;

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

        private static final double HORIZONTAL_P = 0.1;
        private static final double HORIZONTAL_I = 0.01;
        private static final double HORIZONTAL_D = 0.05;
    
        public AlignCommand(PhotonVision photon, CommandSwerveDrivetrain swerve, double holdDistance, int cameraID) {
            m_Vision = photon;
            m_Swerve = swerve;
            this.holdDistance = holdDistance;
            this.cameraID = cameraID;
        
        this.mChassisSpeed = new SwerveRequest.ApplyRobotSpeeds();

    }

    @Override
    public void initialize() {



    }

    @Override
    public void execute() {
        double tx = m_Vision.getTagYaw();
        double currCoralXDist = m_Vision.getTagXDist();
        double currCoralYDist = m_Vision.getTagY();



                
        m_Swerve.setControl(
            mChassisSpeed.withSpeeds(
                new ChassisSpeeds(
                    m_Vision.getTagY(), 
                    0,
                    m_Vision.getTagYaw())
            )
        );
    }
    
    @Override
    public void end(boolean interrupted) {
        m_Swerve.setControl(new SwerveRequest.Idle());
    }

}