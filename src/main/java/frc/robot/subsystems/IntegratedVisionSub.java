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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.Records;
import frc.robot.util.Records.VisionMeasurement;
import frc.robot.util.Records.timestampedPose;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.ConstrainedSolvepnpParams;
import org.photonvision.targeting.PhotonPipelineResult;


/**
 * Manages all of the robot's cameras.
 */
@Logged
public class IntegratedVisionSub extends SubsystemBase {

    private final AprilTagFieldLayout aprilTags =  AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private PhotonCamera camera;
    private PhotonPoseEstimator precision;
    private PhotonPoseEstimator global;
    private boolean enablePrecision;
    private final Optional<ConstrainedSolvepnpParams> constrainedPnpParams;




    public IntegratedVisionSub(String cameraName, Transform3d robotToCamTransform, boolean enabprec) {
    
        camera = new PhotonCamera(cameraName);
        precision = new PhotonPoseEstimator(aprilTags, PNP_DISTANCE_TRIG_SOLVE, robotToCamTransform);
        global = new PhotonPoseEstimator(aprilTags, MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamTransform);
        enablePrecision = enabprec;

        constrainedPnpParams = Optional.of(new ConstrainedSolvepnpParams(true, 0.0));
        // Hit the undocumented Photon Turbo Button™
        // https://github.com/PhotonVision/photonvision/pull/1662
        NetworkTableInstance.getDefault().getBooleanTopic("/photonvision/use_new_cscore_frametime").publish().set(true);
    }

    /**
     * Gets unread results from all cameras.
     * @param pose Robot pose estimate from the last robot cycle.
     */
    public VisionMeasurement getUnreadTrigMeasurements(timestampedPose pose, VisionMeasurement measure) {

        addReferenceHeading(precision, pose);
        return refreshTrigSolve();
    }


        /**
         * Adds reference heading to be utilized by the Photon pose estimator.
         * @param tmp Robot heading estimate to feed.
         */
        public void addReferenceHeading(PhotonPoseEstimator estimator, timestampedPose tmp) {
            estimator.addHeadingData(tmp.timestamp(),tmp.visionPose().getRotation());
        }

        /**
         * Refreshes the provided lists with new unread results from the camera. Note
         * that this method does not remove any elements from the supplied lists.
         * @param measurements A list of vision measurements to add to.
         * @param targets A list of targets to add to.
         */
        private VisionMeasurement refreshTrigSolve() {
            double xystd = 1;
            double angstd = 1;
            Optional <EstimatedRobotPose> est;
            Pose2d reportedEstimate = new Pose2d();
            double reportedEstimateTimestamp = 0; // default to 0 if no estimate. This ensures no override.
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                // If we are disabled, use Constrained SolvePNP to estimate the robot's heading.
                precision.setPrimaryStrategy(
                    DriverStation.isEnabled() ? PNP_DISTANCE_TRIG_SOLVE : CONSTRAINED_SOLVEPNP
                );

                // Get an estimate from the PhotonPoseEstimator.
                est = precision.update(result, Optional.empty(), Optional.empty(), constrainedPnpParams);
                if (est.isEmpty() || est.get().targetsUsed.isEmpty()) continue;

                // Get the target AprilTag, and reject the measurement if the
                // tag is not configured to be utilized by the pose estimator.
                var target = est.get().targetsUsed.get(0);
                int id = target.fiducialId;
                if (!useTag(id)) continue;

                // Get the location of the tag on the field.
                var tagLocation = aprilTags.getTagPose(id);
                if (tagLocation.isEmpty()) continue;

                // Determine the distance from the camera to the tag.
                double distance = target.bestCameraToTarget.getTranslation().getNorm();

                // Calculate the pose estimation weights for X/Y location. As
                // distance increases, the tag is trusted exponentially less.
                xystd = 0.1 * distance * distance;

                // Calculate the angular pose estimation weight. If we're solving via trig, reject
                // the heading estimate to ensure the pose estimator doesn't "poison" itself with
                // essentially duplicate data. Otherwise, weight the estimate similar to X/Y.
                angstd = !precision.getPrimaryStrategy().equals(PNP_DISTANCE_TRIG_SOLVE)
                    ? 0.12 * distance * distance
                    : 1e5;

                if (est.isPresent()) {
                    reportedEstimate = est.get().estimatedPose.toPose2d();
                    reportedEstimateTimestamp = est.get().timestampSeconds;
                }
            }

            return new VisionMeasurement(
            reportedEstimate, 
            reportedEstimateTimestamp, 
            VecBuilder.fill(xystd, xystd, angstd)
            );
        }

        /**
         * Returns {@code true} if an AprilTag should be utilized.
         * @param id The ID of the AprilTag.
         */
        private boolean useTag(int id) {
            return (DriverStation.getAlliance().get() == Alliance.Blue) ? (id >= 17 && id <= 22) : (id >= 6 && id <= 11);
        }

        @Override
        public void periodic() {
            if (enablePrecision) {
                refreshTrigSolve();
            }
        }
    }
