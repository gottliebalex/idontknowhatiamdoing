package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorCommands {
    private ElevatorCommands() {}


  public static Command Down(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.Down.distance());
  }

  public static Command L2Score(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L2.distance());
  }

  public static Command L4Score(ElevatorSubsystem elevator) {
    return elevator.setHeight(ElevatorSubsystem.ElevatorPosition.L4.distance());
  }
}
