package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.PID;
import frc.FSLib.math.simplePID;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.Vision;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  // private frc.robot.Vision s_Vision;
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;
  private double ema;

  public double KP = 0.008;
  public double KI = 0.0;
  public double KD = 0.01;
  public double WindUp = 0.0;
  public double Limit = 0.0;
  public double Smooth = 0.075;
  
  private PID facingPID = new PID(KP, KI, KD, WindUp, Limit);

  // Constants such as camera and target height stored. Change per robot and goal!
  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7.4);
  private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(54);
  // Angle between horizontal and the camera.
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(20);
  //chage the name accordingly
  PhotonCamera camera = new PhotonCamera("photonvision");

  public TeleopSwerve(Swerve s_Swerve, /*frc.robot.Vision s_Vision,*/ XboxController controller) {
    this.s_Swerve = s_Swerve;
    // this.s_Vision = s_Vision;
    this.driver = controller;
    addRequirements(s_Swerve);
  }

  @Override
  public void initialize() {
    rotationVal = 0;
  }

  @Override
  public void execute() {

    if (driver.getBackButton()) {
      s_Swerve.setPose(FieldConstants.BLUE_MB);
    }

    // var vision = camera.getLatestResult();
    
    /* Get Values, Deadband */
    translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(-driver.getLeftY(), Constants.SwerveConstants.axisDeadBand));
    strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(-driver.getLeftX(), Constants.SwerveConstants.axisDeadBand));

  //   if (driver.getLeftBumper() && vision.hasTargets()
  //   ) {
    
  //   /* Get Values, Deadband */
  //   translationVal = translationLimiter.calculate(
  //       MathUtil.applyDeadband(driver.getLeftY(), 0.01));
  //   strafeVal = strafeLimiter.calculate(
  //       MathUtil.applyDeadband(driver.getLeftX(), 0.01));

  //   double range = PhotonUtils.calculateDistanceToTargetMeters(
  //                     CAMERA_HEIGHT_METERS,
  //                     TARGET_HEIGHT_METERS,
  //                     CAMERA_PITCH_RADIANS,
  //                     Units.degreesToRadians(vision.getBestTarget().getPitch()));

  //   SmartDashboard.putNumber("range", range);


  //   ema = Smooth*facingPID.calculate(vision.getBestTarget().getYaw())+(1-Smooth)*ema;
  //   rotationVal = ema;
  //   rotationVal = - facingPID.calculate(vision.getBestTarget().getYaw());
  //   SmartDashboard.putNumber("output", rotationVal);
  //   SmartDashboard.putNumber("damm", vision.getBestTarget().getYaw());
  // }

    // if (s_Vision.hasTarget()) {
    //   rotationVal = s_Vision.getAlignRotationVal();
    // } else {
      rotationVal = 
      rotationLimiter
        .calculate(MathUtil.applyDeadband(-driver.getRightX(), Constants.SwerveConstants.axisDeadBand))
        * SwerveConstants.maxAngularVelocity;
    // }

    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxModuleSpeed),
        rotationVal, true,
        false
      );

    // SmartDashboard.putBoolean("isAiming", s_Vision.hasTarget());
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}