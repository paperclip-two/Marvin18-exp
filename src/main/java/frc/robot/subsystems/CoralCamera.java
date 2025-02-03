package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralCamera extends SubsystemBase {
    private CommandSwerveDrivetrain dt;
    private PhotonCamera coralCam;
    private PhotonPoseEstimator photonPoseEstimator;
    EstimatedRobotPose lastPose = null;
    private PoseStrategy strat;
    private Transform3d mRobotToCam;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    public CoralCamera(CommandSwerveDrivetrain drive, String cameraName, PoseStrategy realStrat, Transform3d rbtc){
        dt = drive;
        coralCam = new PhotonCamera(cameraName);
        strat = realStrat;
        mRobotToCam = rbtc;
        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, strat, mRobotToCam);
    }

    public void resetPose(){
        Optional<EstimatedRobotPose> pose = getEstimatedGlobalPose(dt.getState().Pose);
        PhotonPipelineResult latestResult = coralCam.getLatestResult();
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
        PhotonPipelineResult plresult = coralCam.getAllUnreadResults().get(coralCam.getAllUnreadResults().size() - 1);
        if(plresult.getBestTarget() != null && plresult.hasTargets()){
            PhotonTrackedTarget trackedTarget = plresult.getBestTarget();
            if(trackedTarget != null && trackedTarget.getBestCameraToTarget() != null && aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).isPresent()){
                heading = (aprilTagFieldLayout.getTagPose(trackedTarget.getFiducialId()).get().getRotation().toRotation2d().getDegrees() - (trackedTarget.getBestCameraToTarget().getRotation().toRotation2d().getDegrees()));
            }
        }
        return heading;
    }

    public Translation3d getCoralCamTransform3d() {
        return new Translation3d(
            coralCam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation().getX(),
            coralCam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation().getY(),
            coralCam.getLatestResult().getBestTarget().getBestCameraToTarget().getTranslation().getZ()
        );
    }
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        int latestResultIndex = coralCam.getAllUnreadResults().size() - 1;
        return photonPoseEstimator.update(coralCam.getAllUnreadResults().get(latestResultIndex));
    }
}
