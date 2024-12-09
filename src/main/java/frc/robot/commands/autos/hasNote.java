package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Upper;

public class hasNote extends Command {

  private Upper s_Upper;

  public hasNote(Upper s_Upper) {
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted){}

  @Override
  public boolean isFinished() {
    if (s_Upper.hasNote()) {
      return true;
    } else {
      return false;
    }
  }
}