package frc.robot.util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Records {
    public static final record VisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
        /**
         * Represents a measurement from vision to apply to the pose estimator.
         * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
         */
    }

    public static final record timestampedPose(Pose2d visionPose, double timestamp) {
        /**
         * Represents a timestamped instance of a pose.
         * @see {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
         */
    }
}
