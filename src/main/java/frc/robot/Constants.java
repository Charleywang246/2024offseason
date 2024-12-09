package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.FSLib.limelight.LimelightHelpers;
import frc.FSLib.math.LinearRegression;
import frc.FSLib.swerve.SwerveModuleConstants;
import frc.FSLib.util.AngularVelocity;
import frc.robot.Constants.UpperStateMachine.UpperState;

public class Constants {

    public static final class RobotConstants {
        public static final String canbusName = "GTX7130";
        public static final int DriverControllerID = 0;
        public static UpperState upperState = UpperState.DEFAULT;
        public static Rotation2d gyroYaw = new Rotation2d();
        public static SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];
        public static Pose2d odometryPose = new Pose2d();
    }

    public static final class FieldConstants {
        public static final Pose2d BLUE_LB = 
          new Pose2d(new Translation2d(0.7, 6.7), Rotation2d.fromDegrees(60));
        public static final Pose2d BLUE_MB =
          new Pose2d(new Translation2d(1.47, 5.5), Rotation2d.fromDegrees(0));
        public static final Pose2d BLUE_RB =
          new Pose2d(new Translation2d(0.7, 4.4), Rotation2d.fromDegrees(-60));
        public static final Pose2d RED_LB =
          new Pose2d(new Translation2d(15.85, 4.38), Rotation2d.fromDegrees(-120));
        public static final Pose2d RED_MB =
          new Pose2d(new Translation2d(15.07, 5.5), Rotation2d.fromDegrees(-180));
        public static final Pose2d RED_RB =
          new Pose2d(new Translation2d(15.85, 6.75), Rotation2d.fromDegrees(-120));

        public static final Pose2d X1 =
            new Pose2d(new Translation2d(2.56, 6.99), Rotation2d.fromDegrees(0));
        public static final Pose2d X2 =
            new Pose2d(new Translation2d(2.56, 5.55), Rotation2d.fromDegrees(0));
        public static final Pose2d X3 =
            new Pose2d(new Translation2d(2.56, 4.10), Rotation2d.fromDegrees(0));

        public static final Pose2d X2_STAGE = 
            new Pose2d(new Translation2d(4.12, 5.25), Rotation2d.fromDegrees(0));
      }

    public static final class UpperConstants {
    
        public static final int leftElbowMotorID = 18;
        public static final int rightElbowMotorID = 17;
        public static final int leftShooterMotorID = 41;
        public static final int rightShooterMotorID = 42;
        public static final int intakeMotorID = 43;

        public static final int leftLimitSwitchID = 7;
        public static final int rightLimitSwitchID = 8;

        public static final int elbowCancoderID = 4;
        public static final double elbowCancoderOffset = .0;

        public static boolean speedLimitEnabled = true;
    
        public static final double elbowKP = 4.5;
        public static final double elbowKI = 0.0;
        public static final double elbowKD = 0.5;
        public static final double elbowiWindup = 0.0;
        public static final double elbowiLimit = 0.0;
    
        public static final double shooterKP = 0.00001;
        public static final double shooterKD = 0.0;
    
        public static final Rotation2d ELBOW_AMP_POS = Rotation2d.fromRotations(-0.147949);
        public static final Rotation2d ELBOW_DEFAULT_POS = Rotation2d.fromRotations(-0.284668);
        public static final Rotation2d Elbow_ENDGAME_POS = Rotation2d.fromRotations(-0.398438);
        public static final Rotation2d ELBOW_GROUND_POS = Rotation2d.fromRotations(-0.398438);
        public static final Rotation2d ELBOW_PREENDGAME_POS = Rotation2d.fromRotations(-0.170166);
        public static final Rotation2d ELBOW_BASE_POSE = Rotation2d.fromRotations(-0.325);

        public static final AngularVelocity INTAKE_AMP_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_DEFAULT_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_ENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_GROUND_SPEED = AngularVelocity.fromRevPM(-3000);
        public static final AngularVelocity INTAKE_PREENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_SHOOT_SPEED = AngularVelocity.fromRevPM(-3500);
        public static final AngularVelocity INTAKE_SPEAKER_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_BASE_SPEED = AngularVelocity.fromRevPM(0);

        
        public static final double INTAKE_MAX_RPM = 5000;

        public static final AngularVelocity SHOOTER_AMP_SPEED = AngularVelocity.fromRevPM(-2500);
        public static final AngularVelocity SHOOTER_DEFAULT_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_ENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_GROUND_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_PREENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_SHOOT_SPEED = AngularVelocity.fromRevPM(-3000);
        public static final AngularVelocity SHOOTER_SPEAKER_SPEED = AngularVelocity.fromRevPM(-4000);
        public static final AngularVelocity SHOOTER_BASE_SPEED = AngularVelocity.fromRevPM(-4500);

        public static final double SHOOTER_LEGAL_SPEED = 4500;

    }

    public static final class UpperStateMachine {

        public static enum UpperState {
            AMP,
            DEFAULT,
            ENDGAME,
            GROUND,
            PREENDGAME,
            SHOOT,
            SPEAKER,
            BASE
        }

        public static Rotation2d elbowTarget;
        public static AngularVelocity intakeTarget;
        public static AngularVelocity shooterTarget;

        public UpperStateMachine () {
            elbowTarget = UpperConstants.ELBOW_DEFAULT_POS;
            intakeTarget = UpperConstants.INTAKE_DEFAULT_SPEED;
            shooterTarget = UpperConstants.SHOOTER_DEFAULT_SPEED;
        }

        public void update () {
            switch (RobotConstants.upperState) {
                case AMP:
                    elbowTarget = UpperConstants.ELBOW_AMP_POS;
                    intakeTarget = UpperConstants.INTAKE_AMP_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_AMP_SPEED;
                    break;
                case DEFAULT:
                    elbowTarget = UpperConstants.ELBOW_DEFAULT_POS;
                    intakeTarget = UpperConstants.INTAKE_DEFAULT_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_DEFAULT_SPEED;
                    break;
                case ENDGAME:
                    elbowTarget = UpperConstants.Elbow_ENDGAME_POS;
                    intakeTarget = UpperConstants.INTAKE_ENDGAME_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_ENDGAME_SPEED;
                    break;
                case GROUND:
                    elbowTarget = UpperConstants.ELBOW_GROUND_POS;
                    intakeTarget = UpperConstants.INTAKE_GROUND_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_GROUND_SPEED;
                    break;
                case PREENDGAME:
                    elbowTarget = UpperConstants.ELBOW_PREENDGAME_POS;
                    intakeTarget = UpperConstants.INTAKE_PREENDGAME_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_PREENDGAME_SPEED;
                    break;
                case SHOOT:
                    intakeTarget = UpperConstants.INTAKE_SHOOT_SPEED;
                    // shooterTarget = UpperConstants.SHOOTER_SHOOT_SPEED;
                    // double distance = DriverStation.getAlliance().get() == Alliance.Red ? 
                    //     RobotConstants.odometryPose.getTranslation().getDistance(FieldConstants.redSpeakerCoord)
                    //     : RobotConstants.odometryPose.getTranslation().getDistance(FieldConstants.blueSpeakerCoord);
                    // elbowTarget = Rotation2d.fromRotations(
                    //     LinearRegression.calculate(MapConstants.DISTANCE_TO_ELBOW_AND_SHOOTER, distance, 1)
                    // );
                    // if (RobotConstants.upperState == UpperState.SHOOT) {
                    //     intakeTarget = UpperConstants.INTAKE_SHOOT_SPEED;
                    // } else {
                    //     intakeTarget = UpperConstants.INTAKE_SPEAKER_SPEED;
                    // }
                    // shooterTarget = AngularVelocity.fromRadPM(
                    //     LinearRegression.calculate(MapConstants.DISTANCE_TO_ELBOW_AND_SHOOTER, distance, 2)
                    // );
                    // SmartDashboard.putNumber("distance to Speaker", distance);
                    break;
                case SPEAKER:
                    intakeTarget = UpperConstants.INTAKE_SPEAKER_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_SPEAKER_SPEED;
                    break;
                case BASE:
                    elbowTarget = UpperConstants.ELBOW_BASE_POSE;
                    intakeTarget = UpperConstants.INTAKE_PREENDGAME_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_BASE_SPEED;
                    break;
            }
        }
    }

    public static final class SwerveConstants {
        public static final double axisDeadBand = 0.05;
        public static final int pigeon1 = 1;
        public static final boolean invertGyro = false;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.583; // meters, length between two side's wheels, need to adjust
        public static final double wheelBase = 0.583; // meters, length between same side's wheels, need to adjust
        public static final double driveBaseRaius = 0.35234;
        public static final double wheelDiameter = Units.inchesToMeters(4.0); // need to adjust
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        // public static final double openLoopRamp = 0.25;
        // public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.12244897959 / 1.0); // 6.12:1 (6.12244897959), for MK4i(L3)
        public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, for MK4i(all)

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 10;
        public static final int driveContinuousCurrentLimit = 40;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.008;
        public static final double angleKI = 0.05;
        public static final double angleKD = 0.005;

        /* Angle Motor Auto-Facing PID Values */
        public static final double faceKP = 0.8;
        public static final double faceKI = 0.0;
        public static final double faceKD = 0.1;
        public static final double faceiWindup = 0.0;
        public static final double faceiLimit = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.08; // 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0025; 
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxModuleSpeed = 4; // m/s
        public static final double maxModuleAccleration = 3; // m/s
        public static final double maxAngularVelocity = 13.5; // rad/s
        public static final double maxAngularAccleration = 12; // rad/s

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = false;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Field Oriented */
        public static boolean fieldOriented = true;

        /* Slow Mode */
        public static boolean slow = false;

        public static final Translation2d LFModuleOffset = new Translation2d(wheelBase / 2.0, wheelBase / 2.0);
        public static final Translation2d RFModuleOffset = new Translation2d(wheelBase / 2.0, -wheelBase / 2.0);
        public static final Translation2d LRModuleOffset = new Translation2d(-wheelBase / 2.0, wheelBase / 2.0);
        public static final Translation2d RRModuleOffset = new Translation2d(-wheelBase / 2.0, -wheelBase / 2.0);
        
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 0;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.590682);
        public static final Translation2d moduleOffset = new Translation2d(wheelBase / 2.0, wheelBase / 2.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, moduleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 12;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.374756);
        public static final Translation2d moduleOffset = new Translation2d(wheelBase / 2.0, -wheelBase / 2.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, moduleOffset);
    }


    /* Rear Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 21;
        public static final int angleMotorID = 22;
        public static final int canCoderID = 2;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.456543);
        public static final Translation2d moduleOffset = new Translation2d(-wheelBase / 2.0, wheelBase / 2.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, moduleOffset);
    }

    /* Rear Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 31;
        public static final int angleMotorID = 32;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.053711);
        public static final Translation2d moduleOffset = new Translation2d(-wheelBase / 2.0, -wheelBase / 2.0);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, moduleOffset);
    }

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(3, 0, 0.005), // Translation constants 
        new PIDConstants(2, 0, 0), // Rotation constants 
        maxModuleSpeed, 
        LFModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
        new ReplanningConfig(), // Should the path be replanned at the start of path following if the robot is not already at the starting point?,
                                // Should the path be replanned if the error grows too large or if a large error spike happens while following the path?
        0.004
         );

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      SwerveConstants.LFModuleOffset, 
      SwerveConstants.RFModuleOffset, 
      SwerveConstants.LRModuleOffset, 
      SwerveConstants.RRModuleOffset
  );

  public static final edu.wpi.first.math.geometry.Translation2d[] swerveModuleLocations = {
    new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, trackWidth / 2.0),
    new edu.wpi.first.math.geometry.Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
    new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
    new edu.wpi.first.math.geometry.Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
};
    }

//     public static final class PoseEstimator {

//         private static SwerveDrivePoseEstimator m_Estimator;

//         public PoseEstimator () {
//             m_Estimator = new SwerveDrivePoseEstimator(
//                 new SwerveDriveKinematics(
//                     SwerveConstants.LFModuleOffset, 
//                     SwerveConstants.RFModuleOffset, 
//                     SwerveConstants.LRModuleOffset, 
//                     SwerveConstants.RRModuleOffset
//                 ),
//                 RobotConstants.gyroYaw,
//                 RobotConstants.swervePositions,
//                 RobotConstants.odometryPose
//             );
//         }

//         public void update () {
//                 m_Estimator.update(RobotConstants.gyroYaw, RobotConstants.swervePositions);
//                 boolean doVisionUpdate = true;
//                 LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
//                 if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
//                     // IMPORTANT but still has bug
//                     // if(mt1.rawFiducials[0].ambiguity > 0.7) doVisionUpdate = false;
//                     // if(mt1.rawFiducials[0].distToCamera > 3) doVisionUpdate = false;
//                 }
//                 if (mt1.tagCount == 0) doVisionUpdate = false;
//                 if (doVisionUpdate) {
//                     m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
//                     m_Estimator.addVisionMeasurement(
//                         mt1.pose,
//                         mt1.timestampSeconds
//                     );
//                 }
//             }
//     }

public static class Vision {
    public static final String OPCam = "OrangePiCamera";
    public static final String ATCam = "AprilTagCamera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToOPCam =
            new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    public static final Transform3d kRobotToATCam =
            new Transform3d(new Translation3d(0.28, 0.0, 0.5), new Rotation3d(0, - Math.PI / 4 , Math.PI));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Swerve Auto-align
    public static final double angleKP = 1.5;
}

    public static final class MapConstants {
        
        public static final double[][] SHOOTER_RPM_TO_OUTPUT = {
            {-Double.MAX_VALUE, -1},
            {-5000, -1},
            {0.0, 0.0},
            {5000, 1},
            {Double.MAX_VALUE, 1}
        };

        public static final double[][] DISTANCE_TO_ELBOW_AND_SHOOTER = {
            {-Double.MAX_VALUE, -0.38, 0},
            {1.0, -0.35, 0},
            {2.5, -0.34, 0},
            {4.0, -0.33, 0},
            {Double.MAX_VALUE, -0.30, 0}
        };

    }
}
