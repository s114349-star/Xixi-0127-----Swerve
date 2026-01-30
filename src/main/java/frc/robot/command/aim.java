

package frc.robot.command;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.chassis;

public class aim extends Command {
  private final chassis chassis;

  public aim(chassis chassis) {
    this.chassis = chassis;
    addRequirements(chassis);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
