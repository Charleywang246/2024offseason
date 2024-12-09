package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.PID;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperStateMachine;
import frc.robot.Constants.UpperStateMachine.UpperState;
import frc.robot.subsystems.Upper;

public class GROUND extends Command {

  private Upper s_Upper;

  private final PID elbowPID = new PID(
      UpperConstants.elbowKP,
      UpperConstants.elbowKI,
      UpperConstants.elbowKD,
      UpperConstants.elbowiWindup,
      UpperConstants.elbowiLimit);

  public GROUND(Upper s_Upper) {
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    s_Upper.setShooter(0);
    if (!s_Upper.hasNote()) {
      RobotConstants.upperState = UpperState.GROUND;
      s_Upper.setElbow(-elbowPID.calculate(UpperStateMachine.elbowTarget.getRotations() - s_Upper.getElbowRotation()));
      s_Upper.setIntake(-0.5);
    }
    if (s_Upper.hasNote()) {
      RobotConstants.upperState = UpperState.DEFAULT;
      s_Upper.setIntake(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // System.out.println("GROUND end");
  }

  @Override
  public boolean isFinished() {
    if (s_Upper.hasNote()) {
      s_Upper.setIntake(0);
      return true;
    } else {
      return false;
    }
  }
}