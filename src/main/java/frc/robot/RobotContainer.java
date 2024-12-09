// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.commands.autos.SHOOT;
import frc.robot.commands.autos.hasNote;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Upper;

public class RobotContainer {

  private final XboxController controller = new XboxController(RobotConstants.DriverControllerID);

  private static Swerve s_Swerve = new Swerve();

  // private static Vision s_Vision = new Vision();

  private static Upper s_Upper = new Upper();

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(s_Swerve, controller);
  private final TeleopUpper teleopUpper = new TeleopUpper(s_Upper, controller);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  private Field2d PP_field = new Field2d();

  // private final PhotonRunnable camera = new PhotonRunnable();

  public RobotContainer() {

    s_Swerve.setDefaultCommand(teleopSwerve);
    s_Upper.setDefaultCommand(teleopUpper);

    configureBindings();
    generateNamedCommands();

    autoChooser.setDefaultOption("CleanUp-Close", new PathPlannerAuto("CleanUp-Close"));
    autoChooser.addOption("3note", new PathPlannerAuto("3note"));

    SmartDashboard.putData("auto chooser", autoChooser);

    SmartDashboard.putData("Field", PP_field);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      PP_field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      PP_field.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      PP_field.getObject("path").setPoses(poses);
    });
  }

  public void configureBindings() {

    // wait for vision
    // SmartDashboard.putData("Face Note",
    // AutoBuilder.pathfindToPose(new
    // Pose2d(s_Swerve.getOdometryPose().getTranslation(),
    // Rotation2d.fromDegrees(camera.getNoteYaw())),
    // new PathConstraints(
    // 4.0, 2.0,
    // Units.degreesToRadians(360), Units.degreesToRadians(540)
    // )).until(s_Swerve.driverWantsControl()))
    // ;

    SmartDashboard.putData("Go To X1",
        AutoBuilder.pathfindToPose(FieldConstants.X1, new PathConstraints(
            4.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540))).until(s_Swerve.driverWantsControl()));

    SmartDashboard.putData("Go To X2",
        AutoBuilder.pathfindToPose(FieldConstants.X2, new PathConstraints(
            4.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540))).until(s_Swerve.driverWantsControl()));

    SmartDashboard.putData("Go To X3",
        AutoBuilder.pathfindToPose(FieldConstants.X3, new PathConstraints(
            4.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540))).until(s_Swerve.driverWantsControl()));

    SmartDashboard.putData("SHOOT",
        (AutoBuilder.pathfindToPose(FieldConstants.BLUE_MB, new PathConstraints(
            4.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540))).alongWith(new SHOOT(s_Upper)))
            .until(s_Swerve.driverWantsControl()));

    SmartDashboard.putData("MB-X2 pathfind", Commands.runOnce(() -> {

      Pose2d currentPose = s_Swerve.getOdometryPose();

      List<Translation2d> waypoints = PathPlannerPath.bezierFromPoses(currentPose, FieldConstants.BLUE_MB,
          FieldConstants.X2);

      AutoBuilder.pathfindThenFollowPath(new PathPlannerPath(waypoints,
          new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0, FieldConstants.X2.getRotation())),
          new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540)));
    }).until(s_Swerve.driverWantsControl()));

    SmartDashboard.putData("SHOOT mid-x2_stage_base pathfind+EventMarker", Commands.runOnce(() -> {
      Pose2d currentPose = s_Swerve.getOdometryPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), currentPose.getRotation());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(FieldConstants.BLUE_MB.getTranslation()),
          new Rotation2d());

      List<Translation2d> waypoint = PathPlannerPath.bezierFromPoses(startPos, FieldConstants.X2_STAGE, endPos);
      new EventMarker(3.0, new SHOOT(s_Upper));

      AutoBuilder.pathfindThenFollowPath(new PathPlannerPath(waypoint,
          new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0, FieldConstants.X2.getRotation())),
          new PathConstraints(2.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(540)));

      // another way, still need test
      // PathPlannerPath path = new PathPlannerPath(waypoint,
      // new PathConstraints(3.0, 3.0,
      // Units.degreesToRadians(360), Units.degreesToRadians(540)),
      // new GoalEndState(0, endPos.getRotation(), false), false);

      // path.preventFlipping = true;

      // AutoBuilder.followPath(path);
    }));
  }

  private Alliance getAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      return DriverStation.getAlliance().get();
    }
    return Alliance.Blue;
  }

  public Command Intake() {
    return s_Upper.IntakeCommand(-0.6, () -> s_Upper.hasNote()).repeatedly().until(() -> s_Upper.hasNote());
    // return new GROUND(s_Upper);
  }

  public Command Shoot() {
    return s_Upper.BaseShootCommand(0.9).until(() -> !s_Upper.hasNote());
    // return new SHOOT(s_Upper);
  }

  public Command AutoShoot() {
    return AutoBuilder.pathfindToPose(getAlliance() == Alliance.Blue ? FieldConstants.BLUE_MB : FieldConstants.RED_MB,
        new PathConstraints(
            4.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(540)))
        .alongWith(new SHOOT(s_Upper));
  }

  public Command hasNote() {
    return new hasNote(s_Upper);
  }

  public boolean NoteVaild() {
    return s_Upper.hasNote();
  }

  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return new PathPlannerAuto("CleanUp-Close").andThen(AutoShoot());
  }

  public void generateNamedCommands() {
    NamedCommands.registerCommand("Intake", Intake());
    NamedCommands.registerCommand("Shoot", Shoot());
    NamedCommands.registerCommand("AutoShoot", AutoShoot());
    NamedCommands.registerCommand("hasNote", hasNote());
    NamedCommands.registerCommand("NoteVaild", new WaitUntilCommand(() -> NoteVaild()));
  }
}
