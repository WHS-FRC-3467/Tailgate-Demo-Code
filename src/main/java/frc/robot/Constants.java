// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
/* 
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue; */

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class Constants {

    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final class RobotConstants {
        public static final boolean kIsTuningMode = true;
    }


    public static final class DriveConstants {
        public static final double headingAngleTolerance = 3.0;
        public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        public static final double driverSpeed = 0.6; //Multiplier to the controller input
    }

        public static class PhotonVisionConstants {
        public static class front_left_cam {
            public static final String kCameraName = "front_left";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(-0.09, 0.170,0.628),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(-15)));
        }

        public static class front_right_cam {
            public static final String kCameraName = "front_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(-0.09, -0.170,0.628),
                    new Rotation3d(0, Units.degreesToRadians(-20), Units.degreesToRadians(15)));
        }

        public static class back_right_cam {
            public static final String kCameraName = "back_right";
            public static final Transform3d kRobotToCam = new Transform3d(
                    new Translation3d(-0.143, -0.321,0.534),
                    new Rotation3d(0, 0, Units.degreesToRadians(-155)));
        }
       
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final double VISION_XY_MARGIN = 0.5;
        public static final double VISION_Z_MARGIN = 0.75;
        public static final double VISION_STD_XY_SCALE = 0.02 * 10;
        public static final double VISION_STD_ROT_SCALE = 0.035 * 10;


    }

    public static class LimelightConstants {
        public static final String kCameraName = "limelight";
    }

    public static class FieldConstants {

        public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5 + 12), Units.inchesToMeters(218.42), new Rotation2d(0));
        public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73 - 12), Units.inchesToMeters(218.42), new Rotation2d(Math.PI));
        public static final Pose2d BLUE_FEED = new Pose2d(1.25, 6.2, new Rotation2d(0));
        public static final Pose2d RED_FEED = new Pose2d(15.250, 6.2, new Rotation2d(0));
        public static final Pose2d BLUE_AMP = new Pose2d(Units.inchesToMeters(72.5),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final Pose2d RED_AMP = new Pose2d(Units.inchesToMeters(578.77),Units.inchesToMeters(323.00),new Rotation2d(-Math.PI/2));
        public static final double BLUE_AUTO_PENALTY_LINE = 9; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double RED_AUTO_PENALTY_LINE = 7.4; // X distance from origin to center of the robot almost fully crossing the midline
        public static final double FIELD_X_LENGTH = 16.5417;
        public static final double FIELD_Y_LENGTH = 8.0136;

    }
    // Rapid React
    public static final class GoalConstants {
        public static final Translation2d kGoalLocation = new Translation2d(8.23, 4.115);
        public static final Translation2d kWrongBallGoal = new Translation2d(5.50, 4.115);
        public static final Translation2d kHangerLocation = new Translation2d(2.00, 6.00);
    }
}

