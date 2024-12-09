// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.config.CANSparkMaxConfig;
import frc.FSLib.config.CANSparkMaxConfig.Usage;
import frc.FSLib.math.PID;
import frc.FSLib.swerve.MathUtility;
import frc.FSLib.swerve.OnboardModuleState;
import frc.FSLib.swerve.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase {

  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;
  
  private PID rotorPID;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private CANcoder angleEncoder;

  private final SparkPIDController driveController;

  private final SimpleMotorFeedforward FeedForward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

  private SwerveModuleConstants swerveModuleConstants;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.swerveModuleConstants = moduleConstants;
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    angleEncoder = new CANcoder(moduleConstants.cancoderID, RobotConstants.canbusName);
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxConfig.setCANSparkMaxBusUsage(angleMotor, Usage.kVelocityOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    rotorPID = new PID(Constants.SwerveConstants.angleKP, 0, Constants.SwerveConstants.angleKD, 0, 0);
    angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
  }
  
  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxConfig.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(Constants.SwerveConstants.driveKP);
    driveController.setI(Constants.SwerveConstants.driveKI);
    driveController.setD(Constants.SwerveConstants.driveKD);
    driveController.setFF(Constants.SwerveConstants.driveKFF);
    driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue()).plus(angleOffset);
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      getAngle()
    );
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
  
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxModuleSpeed * 0.01)) 
      ? lastAngle
      : desiredState.angle;

    desiredState.angle = angle;

    double error = getState().angle.getDegrees() - desiredState.angle.getDegrees();
    double constrainedError = MathUtility.constrainAngleDegrees(error);
    double rotorOutput = rotorPID.calculate(constrainedError);
    rotorOutput = MathUtility.clamp(rotorOutput, -1, 1);
    angleMotor.set(rotorOutput);
    lastAngle = angle;

    if(isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxModuleSpeed;
      percentOutput = MathUtility.clamp(percentOutput, -1, 1);
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
        desiredState.speedMetersPerSecond,
        ControlType.kVelocity,
        0,
        FeedForward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  public void setAngleMotor(double speed) {
    angleMotor.set(speed);
  }

  public void setDriveMotor(double speed) {
    driveMotor.set(speed);
  }

  public SwerveModuleConstants getSwerveModuleConstants() {
    return swerveModuleConstants;
  }
}
