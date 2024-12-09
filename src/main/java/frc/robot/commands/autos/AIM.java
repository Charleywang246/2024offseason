// package frc.robot.commands.autos;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.FSLib.math.PID;
// import frc.FSLib.math.simplePID;
// import frc.robot.Constants;
// import frc.robot.Constants.FieldConstants;
// import frc.robot.Constants.SwerveConstants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;

// public class AIM extends Command {
//   private Swerve s_Swerve;
//   private Vision s_Vision;
//   private XboxController driver;

//   private static enum swerveState{
//     DEFAULT,
//     aimBot
//   }

//   private swerveState state = swerveState.DEFAULT;

//   private double ema;

//   public double KP = Constants.LimeLight.KPDefault;
//   public double KI = Constants.LimeLight.KIDefault;
//   public double KD = Constants.LimeLight.KDDefault;
//   public double WindUp = Constants.LimeLight.WindupDefault;
//   public double Limit = Constants.LimeLight.LimitDefault;
//   public double Smooth = Constants.LimeLight.SmoothDefault;

//   private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//   private NetworkTableEntry tx = table.getEntry("tx");
//   private NetworkTableEntry ty = table.getEntry("ty");
//   private NetworkTableEntry tpcs = table.getEntry("targetpose_cameraspace");
  
//   private PID facingPID = new PID(KP, KI, KD, WindUp, Limit);

//   private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
//   private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
//   private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

//   private double translationVal;
//   private double strafeVal;
//   private double rotationVal;

//   private Timer timer = new Timer();

//   public AIM(Swerve s_Swerve,Vision s_Vision) {
//     this.s_Swerve = s_Swerve;
//     this.s_Vision = s_Vision;
//     addRequirements(s_Swerve, s_Vision);
//   }

//   @Override
//   public void initialize() {
//     timer.reset();
//     timer.start();
//   }

//   @Override
//   public void execute() {
//       ema = 0.8*facingPID.calculate(tx.getDouble(0.0))+(1-0.8)*ema;
//       rotationVal = -ema;
//       s_Swerve.drive(
//         new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxModuleSpeed),
//         rotationVal * Constants.SwerveConstants.maxAngularVelocity, true,
//         true
//       );
//   }

//   @Override
//   public void end(boolean interrupted){
//     System.out.println("bro ur teleop swerve is fucked");
//   }

//   @Override
//   public boolean isFinished() {
//     if (timer.get() >= 0.5) {
//       return true;
//     } else {
//     return false;
//     }
//   }
// }