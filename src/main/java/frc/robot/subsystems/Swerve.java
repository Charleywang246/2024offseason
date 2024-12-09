// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants;
// import frc.robot.Vision;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;

public class Swerve extends SubsystemBase {
  private static Swerve mInstance;

  private final XboxController driver = new XboxController(RobotConstants.DriverControllerID);

  private final Pigeon2 gyro = new Pigeon2(SwerveConstants.pigeon1, RobotConstants.canbusName);

  public SwerveModule[] mSwerveMods = new SwerveModule[] {
    new SwerveModule(0, SwerveConstants.Mod0.constants),
    new SwerveModule(1, SwerveConstants.Mod1.constants),
    new SwerveModule(2, SwerveConstants.Mod2.constants),
    new SwerveModule(3, SwerveConstants.Mod3.constants)
  };

  private final SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(
      mSwerveMods[0].getSwerveModuleConstants().getModuleOffset(),
      mSwerveMods[1].getSwerveModuleConstants().getModuleOffset(),
      mSwerveMods[2].getSwerveModuleConstants().getModuleOffset(),
      mSwerveMods[3].getSwerveModuleConstants().getModuleOffset()
    );

  private SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, getGyroYaw(), getPositions());

  private Field2d m_field = new Field2d();

  // The robot pose estimator for tracking swerve odometry and applying vision corrections.
  private SwerveDrivePoseEstimator poseEstimator;

  private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

  private Pose2d mRobotPose = new Pose2d();

  ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

  public Swerve() {

    // Define the standard deviations for the pose estimator, which determine how fast the pose
        // estimate converges to the vision measurement. This should depend on the vision measurement
        // noise
        // and how many or how frequently vision measurements are applied to the pose estimator.
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        getGyroYaw(),
                        getModulePositions(),
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);

    AutoBuilder.configureHolonomic(
      this::getEstimatedPose, // Robot pose supplier (getOdometryPose or getPoseEstimated)
      this::setPose,  // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      SwerveConstants.pathFollowerConfig,
      () -> {
          if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
          return false;
      },
      this
    );

    setPose(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
        ? FieldConstants.RED_MB
        : FieldConstants.BLUE_MB);

    }


  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getGyroYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxModuleSpeed);
    for (SwerveModule mod : mSwerveMods) { 
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getOdometryPose() {
    return swerveOdometry.getPoseMeters();
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getGyroYaw(), getPositions(), pose);
    poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public void resetOdometryPose(Pose2d pose) {
      swerveOdometry.resetPosition(getGyroYaw(), getPositions(), pose);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getOdometryPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  // return Robot Relative Speeds
  public ChassisSpeeds getRobotRelativeSpeeds() {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        Constants.SwerveConstants.kinematics.toChassisSpeeds(getStates()),
        getGyroYaw());

    return chassisSpeeds;
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  // for pose estimater
  public Rotation2d getGyroYaw() {
    return gyro.getRotation2d();
  }

  public void setYaw(Rotation2d pos) {
    gyro.setYaw(pos.getDegrees());
  }

  public static Swerve getInstance() {
    if (mInstance == null) {
        mInstance = new Swerve();
    }
    return mInstance;
  }

  public void stop() {
    driveFieldRelative(new ChassisSpeeds());
  }

  public BooleanSupplier driverWantsControl() {
    return () -> 
      MathUtil.applyDeadband(driver.getLeftY(), Constants.SwerveConstants.axisDeadBand) != 0.0 ||
      MathUtil.applyDeadband(driver.getLeftX(), Constants.SwerveConstants.axisDeadBand) != 0.0 ||
      MathUtil.applyDeadband(driver.getRightX(), Constants.SwerveConstants.axisDeadBand) != 0.0;
  }

  private void updateSwervePoses() {
    for (SwerveModule module : mSwerveMods) {
      module.periodic();
  }

  // Update the odometry of the swerve drive using the wheel encoders and gyro.
  poseEstimator.update(getGyroYaw(), getModulePositions());
  }

  @Override
  public void periodic() {
    updateSwervePoses();
    // m_field.setRobotPose(mRobotPose);    
    // m_field.setRobotPose(getEstimatedPose());
    m_field.setRobotPose(getOdometryPose());
    SmartDashboard.putData("Field", m_field);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

  /**
     * Reset the estimated pose of the swerve drive on the field.
     *
     * @param pose New robot pose.
     */
    public void resetPose(Pose2d pose) {
      poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  /** Get the estimated pose of the swerve drive on the field. */
  public Pose2d getPose() {
      return poseEstimator.getEstimatedPosition();
  }

  /** The heading of the swerve drive's estimated pose on the field. */
  public Rotation2d getHeading() {
      return getPose().getRotation();
  }

  /**
     * Get the Pose2d of each swerve module based on kinematics and current robot pose. The returned
     * array order matches the kinematics module order.
     */
  public Pose2d[] getModulePoses() {
        Pose2d[] modulePoses = new Pose2d[mSwerveMods.length];

        /** i need to re-write the whole fking swerve module, i'll do it later */
        for (int i = 0; i < mSwerveMods.length; i++) {
            var module = mSwerveMods[i];
            modulePoses[i] =
                    getPose()
                            .transformBy(
                                    new Transform2d(
                                            module.getSwerveModuleConstants().getModuleOffset(), module.getAngle()));
        }
        return modulePoses;
    }
    /* used to be : */
    public SwerveModulePosition[] getModulePositions() {
      return Arrays.stream(mSwerveMods).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
    }
    
}
