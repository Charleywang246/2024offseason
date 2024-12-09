package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Upper;

public class DEFAULT extends Command {

  private Upper s_Upper;

  private Timer timer = new Timer();

  public DEFAULT(Upper s_Upper) {
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    s_Upper.setLeftShooter(0);
    s_Upper.setRightShooter(0);
    s_Upper.setIntake(0);
  }

  @Override
  public void execute() {
    s_Upper.setIntake(0);
    s_Upper.setLeftShooter(0);
    s_Upper.setRightShooter(0);
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished() {
    if (timer.get() >= 01) {
        return true;
    } else {
        return false;
    }
  }
}