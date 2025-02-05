package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class PhotonVision extends SubsystemBase {
    private CommandSwerveDrivetrain dt;
    private PhotonCamera camera;
    private PhotonPoseEstimator photonPoseEstimator;
    EstimatedRobotPose lastPose = null;
    private PoseStrategy strat;
    private Transform3d mRobotToCam;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public PhotonVision(CommandSwerveDrivetrain drive, String cameraName, PoseStrategy realStrat, Transform3d rbtc){
        dt = drive;
        camera = new PhotonCamera(cameraName);
        strat = realStrat;
        mRobotToCam = rbtc;
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, strat, mRobotToCam);
    }

    public PhotonCamera getCamera() {
        return camera;

    }

    public void resetPose(){
        Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose(dt.getState().Pose);
        PhotonPipelineResult latestResult = camera.getLatestResult();
        if(latestResult.hasTargets()){
            PhotonTrackedTarget trackedTarget = latestResult.getBestTarget();
            
            if(pose.isPresent() && trackedTarget != null
            && trackedTarget.getBestCameraToTarget() != null
            && ((trackedTarget.getBestCameraToTarget().getTranslation().getX() < 3.5 && (DriverStation.isDisabled() || DriverStation.isAutonomous())) 
            || (trackedTarget.getBestCameraToTarget().getTranslation().getX() < 6 && DriverStation.isTeleop())
            )){
                double stdev = 0.01 * trackedTarget.getBestCameraToTarget().getTranslation().getX();
                dt.setVisionMeasurementStdDevs(VecBuilder.fill(stdev, stdev, 0.5));
                dt.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
                lastPose = pose.get();
            }
        }
    }



    @Override
    public void periodic(){
        resetPose();
    }

    public double getRobotHeading(){
        double heading = 0;
        if(lastPose != null){
            heading = lastPose.estimatedPose.getRotation().toRotation2d().getDegrees();
        }
        PhotonPipelineResult plresult = camera.getLatestResult();
        if(plresult.getBestTarget() != null && plresult.hasTargets()){
            PhotonTrackedTarget trackedTarget = plresult.getBestTarget();
            if(trackedTarget != null && trackedTarget.getBestCameraToTarget() != null && aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).isPresent()){
                heading = (aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).get().getRotation().toRotation2d().getDegrees() - (trackedTarget.getBestCameraToTarget().getRotation().toRotation2d().getDegrees()));
            }
        }
        return heading;
    }

    public double getCoralTagDist() {
        double tagPitch = 0;
        
        List<PhotonPipelineResult> currentUnreadPipeline = camera.getAllUnreadResults();
        if (!currentUnreadPipeline.isEmpty()) {
            PhotonPipelineResult latestTag = currentUnreadPipeline.get(currentUnreadPipeline.size() - 1);
        // make another state machine that enables flashing led when target is not found 
            tagPitch = latestTag.getBestTarget().getPitch();
        } else {
            return 0.0;
        }
        
        // make state machine that tells this what thte current tag heigh is - when we want to make this work
        // make another state machine that enables flashing led when target is not found 
        return PhotonUtils.calculateDistanceToTargetMeters(mRobotToCam.getZ(), Constants.VisionConstants.CORAL_APRILTAG_HEIGHT, 0, tagPitch);
    }

    public double getCoralYaw() {
        Pose2d dtcurr = dt.getState().Pose;
        double tagYaw = 0;
        
        List<PhotonPipelineResult> currentUnreadPipeline = camera.getAllUnreadResults();
        if (!currentUnreadPipeline.isEmpty()) {
            PhotonPipelineResult latestTag = currentUnreadPipeline.get(currentUnreadPipeline.size() - 1);
            tagYaw = latestTag.getBestTarget().getYaw();
        } else {
            return 0.0;
        }
        
        // make state machine that tells this what thte current tag heigh is - when we want to make this work
        return tagYaw;
    }    

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(camera.getAllUnreadResults().get(camera.getAllUnreadResults().size() - 1));
    
    }

 
}
