package frc.robot.subsystems;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.util.Records;
import frc.robot.util.Records.VisionMeasurement;
import frc.robot.util.Records.timestampedPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.targeting.PhotonPipelineResult;


/**
 * Manages all of the robot's cameras.
 */
@Logged
public final class VisionManager {

    private static VisionManager instance = null;

    public static VisionManager getInstance() {
        if (instance == null) instance = new VisionManager();
        return instance;
    }

    @NotLogged
    private final Camera[] cameras;

    private final AprilTagFieldLayout aprilTags;



    private VisionManager() {
        aprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        cameras = new Camera[] {
            new Camera("reef", Constants.Vision.reefRobotToCam),
            new Camera("feeder", Constants.Vision.feederRobotToCam),
        };

        // Hit the undocumented Photon Turbo Button™
        // https://github.com/PhotonVision/photonvision/pull/1662
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    /**
     * Resets cached measurements utilized by the pose estimators for seeding. It is
     * recommended to call this method after resetting the robot's pose or rotation.
     */
    //public void reset() {
  //      for (var camera : cameras) camera.clearHeadingData();
   // }

   // disabled functionality because GRR uses time-based programming. This is unecessary
   // as this particular estimate will only be used on a secondary filtered pose.

    /**
     * Gets unread results from all cameras.
     * @param pose Robot pose estimate from the last robot cycle.
     */
    public VisionMeasurement getUnreadTrigMeasurements(Camera cameraToUpdate, timestampedPose pose, VisionMeasurement measure) {

        cameraToUpdate.addReferenceHeading(pose);
        return cameraToUpdate.refreshTrigSolve(measure);
        
    }

    private class Camera {

        private final PhotonCamera camera;
        private final PhotonPoseEstimator precisionEstimator;
        private final PhotonPoseEstimator globalEstimator;
        private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;

        /**
         * Create a camera.
         * @param cameraName The configured name of the camera.
         * @param robotToCamera The {@link Transform3d} from the robot's center to the camera.
         */
        private Camera(String cameraName, Transform3d robotToCamera) {
            camera = new PhotonCamera(cameraName);
            precisionEstimator = new PhotonPoseEstimator(aprilTags, PNP_DISTANCE_TRIG_SOLVE, robotToCamera);
            globalEstimator = new PhotonPoseEstimator(aprilTags, MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
            constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 0.0));
        }



        /**
         * Adds reference poses to be utilized by the Photon pose estimator.
         * @param tmp Robot pose estimate to feed.
         */
        private void addReferenceHeading(timestampedPose tmp) {
            precisionEstimator.addHeadingData(tmp.timestamp(),tmp.visionPose().getRotation());
            
        }

        /**
         * Refreshes the provided lists with new unread results from the camera. Note
         * that this method does not remove any elements from the supplied lists.
         * @param measurements A list of vision measurements to add to.
         * @param targets A list of targets to add to.
         */
        private VisionMeasurement refreshTrigSolve(VisionMeasurement measurement) {
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                // If we are disabled, use Constrained SolvePNP to estimate the robot's heading.
                precisionEstimator.setPrimaryStrategy(
                    DriverStation.isEnabled() ? PNP_DISTANCE_TRIG_SOLVE : CONSTRAINED_SOLVEPNP
                );

                // Get an estimate from the PhotonPoseEstimator.
                var estimate = precisionEstimator.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);
                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                // Get the target AprilTag, and reject the measurement if the
                // tag is not configured to be utilized by the pose estimator.
                var target = estimate.get().targetsUsed.get(0);
                int id = target.fiducialId;
                if (!useTag(id)) continue;

                // Get the location of the tag on the field.
                var tagLocation = aprilTags.getTagPose(id);
                if (tagLocation.isEmpty()) continue;

                // Determine the distance from the camera to the tag.
                double distance = target.bestCameraToTarget.getTranslation().getNorm();

                // Calculate the pose estimation weights for X/Y location. As
                // distance increases, the tag is trusted exponentially less.
                double xyStd = 0.1 * distance * distance;

                // Calculate the angular pose estimation weight. If we're solving via trig, reject
                // the heading estimate to ensure the pose estimator doesn't "poison" itself with
                // essentially duplicate data. Otherwise, weight the estimate similar to X/Y.
                double angStd = !precisionEstimator.getPrimaryStrategy().equals(PNP_DISTANCE_TRIG_SOLVE)
                    ? 0.12 * distance * distance
                    : 1e5;

                

            }
            return new VisionMeasurement(null, 0, null);
        }

        /**
         * Returns {@code true} if an AprilTag should be utilized.
         * @param id The ID of the AprilTag.
         */
        private boolean useTag(int id) {
            return (DriverStation.getAlliance().get() == Alliance.Blue) ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
        }
    }
}