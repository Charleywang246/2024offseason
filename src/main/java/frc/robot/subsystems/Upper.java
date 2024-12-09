package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.UpperConstants;
import frc.FSLib.util.AngularVelocity;
import frc.robot.Constants.RobotConstants;

public class Upper extends SubsystemBase {

    private final TalonFX leftElbow = new TalonFX(UpperConstants.leftElbowMotorID, RobotConstants.canbusName);
    private final TalonFX rightElbow = new TalonFX(UpperConstants.rightElbowMotorID, RobotConstants.canbusName);
    
    private final CANSparkMax leftShooter = new CANSparkMax(UpperConstants.leftShooterMotorID, MotorType.kBrushless);
    private final CANSparkMax rightShooter = new CANSparkMax(UpperConstants.rightShooterMotorID, MotorType.kBrushless);

    private final CANSparkMax intake = new CANSparkMax(UpperConstants.intakeMotorID, MotorType.kBrushless);

    private final CANcoder elbowCancoder = new CANcoder(UpperConstants.elbowCancoderID, RobotConstants.canbusName);

    private final DigitalInput leftLimitSwitch = new DigitalInput(UpperConstants.leftLimitSwitchID);
    private final DigitalInput rightLimitSwitch = new DigitalInput(UpperConstants.rightLimitSwitchID);

    public Upper () {
        configElbow();
        configShooter();
        configIntake();
        intake.setInverted(true);
    }

    // config
    public void configElbow () {
        leftElbow.setInverted(false);
        rightElbow.setInverted(true);
        leftElbow.setNeutralMode(NeutralModeValue.Brake);
        rightElbow.setNeutralMode(NeutralModeValue.Brake);
        elbowCancoder.getConfigurator().apply(
            new CANcoderConfiguration().MagnetSensor.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        );
    }

    public void configShooter () {
        leftShooter.restoreFactoryDefaults();
        rightShooter.restoreFactoryDefaults();
        leftShooter.setInverted(false);
        rightShooter.setInverted(false);
        leftShooter.setIdleMode(IdleMode.kCoast);
        rightShooter.setIdleMode(IdleMode.kCoast);
    }

    public void configIntake () {
        intake.setIdleMode(IdleMode.kBrake);
    }

    // elbow
    public double getElbowRotation () {
        return (elbowCancoder.getPosition().getValue() + UpperConstants.elbowCancoderOffset);
    }

    public void setElbow (double speed) {
        leftElbow.set(speed);
        rightElbow.set(speed);
    }

    // intake
    public double getIntakeVel () {
        return intake.getEncoder().getVelocity();
    }

    public void setIntake (double speed) {
        intake.set(speed);
    }

    // shooter
    public double getLeftShooterRPM () {
        return leftShooter.getEncoder().getVelocity();
    }

    public double getRightShooterRPM () {
        return rightShooter.getEncoder().getVelocity();
    }

    public double getShooterRPM () {
        return (getLeftShooterRPM() + getRightShooterRPM()) / 2;
    }

    public void setLeftShooter (double speed) {
        leftShooter.set(speed);
    }

    public void setRightShooter (double speed) {
        rightShooter.set(speed);
    }

    public void setShooter(double speed) {
        setLeftShooter(speed);
        setRightShooter(speed);
    }

    // Command factory
    public Command IntakeCommand (double intakespeed, BooleanSupplier hasNote) {
        Command c = new Command() {

            @Override
            public void execute() {
                if(!hasNote.getAsBoolean()) {
                    setIntake(intakespeed);
                }
            }

            @Override
            public void end(boolean interrupted) {
                setIntake(0);
                setIntake(0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }
    
    public Command BaseShootCommand(double speed) {
        Command c = new Command() {
            
            @Override
            public void execute() {
                setShooter(speed);
            }

            @Override
            public void end(boolean interrupted) {
                setShooter(0);
                setIntake(0);
                setIntake(0);
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        };
        c.addRequirements(this);
        return c;
    }

    // limitSwitch
    public boolean hasNote () {
        return (leftLimitSwitch.get() || !rightLimitSwitch.get());
    }

    public double runVolt(Voltage Volt) {
        var SySid = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(null, null, null));
        return intake.getAppliedOutput() * intake.getBusVoltage();
    }

    @Override
    public void periodic () {
        SmartDashboard.putNumber("elbowDEG", getElbowRotation());
        SmartDashboard.putNumber("rightShooterRPM", getRightShooterRPM());
        SmartDashboard.putNumber("leftShooterRPM", getLeftShooterRPM());
        SmartDashboard.putBoolean("hasNote", hasNote());
    }

}
