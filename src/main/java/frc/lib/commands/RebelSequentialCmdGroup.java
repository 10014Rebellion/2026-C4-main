package frc.lib.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.telemetry.Telemetry;

public class RebelSequentialCmdGroup extends SequentialCommandGroup {
  public RebelSequentialCmdGroup(Command... commands) {
    super(proxyAll(commands));
  }

  private static Command[] proxyAll(Command... commands) {
    if (commands == null) {
      Telemetry.log("RebelSequentialCmdGroup received null command array; creating empty group.");
      return new Command[0];
    }

    return Arrays.stream(commands)
        .map(RebelSequentialCmdGroup::sanitizeCommand)
        .map(Command::asProxy)
        .toArray(Command[]::new);
  }

  private static Command sanitizeCommand(Command command) {
    if (command == null) {
      Telemetry.log("RebelSequentialCmdGroup received null child command; substituting Commands.none().");
      return Commands.none();
    }

    return command;
  }
}