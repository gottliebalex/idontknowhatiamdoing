// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem.WristPosition;
import frc.robot.subsystems.DiffyWrist.DiffyWristSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public ElevatorSubsystem elevator = new ElevatorSubsystem();
  public WristSubsystem wrist = new WristSubsystem();
  private final DiffyWristSubsystem m_DiffyWristSubsystem = new DiffyWristSubsystem();
  public CommandXboxController xboxController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureBindings();

    RobotModeTriggers.teleop().onTrue(
      setSafePositions());
    

  }

  /** 
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
//  private void configureBindings() {
    // xboxController.button(1).onTrue(ElevatorCommands.Down(elevator));
    // xboxController.button(2).onTrue(ElevatorCommands.L2Score(elevator));
    // xboxController.button(3).onTrue(ElevatorCommands.L4Score(elevator));
    // xboxController.button(4).onTrue(elevator.sysId());

    // xboxController.button(5).onTrue(WristCommands.Stowed(wrist));
    // xboxController.button(6).onTrue(WristCommands.AlgaeIntake(wrist));
    // xboxController.button(7).onTrue(WristCommands.TestWrist(wrist));
    // xboxController.button(8).onTrue(wrist.sysId());
    //}
  private final GenericHID apacController = new GenericHID(0); // 0 = USB port

  private void configureBindings() {
      new JoystickButton(apacController, 1).onTrue(ElevatorCommands.Down(elevator));
      new JoystickButton(apacController, 2).onTrue(ElevatorCommands.L2Score(elevator));
      new JoystickButton(apacController, 3).onTrue(ElevatorCommands.L4Score(elevator));
      new JoystickButton(apacController, 4).onTrue(ElevatorCommands.Zero(elevator));

      new JoystickButton(apacController, 5).onTrue(WristCommands.Stowed(wrist));
      new JoystickButton(apacController, 6).onTrue(WristCommands.AlgaeIntake(wrist));
      new JoystickButton(apacController, 7).onTrue(WristCommands.TestWrist(wrist));
      new JoystickButton(apacController, 8).onTrue(wrist.sysId());

      // xboxController.button(1).onTrue(elevator.startSignalLogger());
      // xboxController.button(2).onTrue(elevator.stopSignalLogger());

      // xboxController.button(3).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // xboxController.button(4).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // xboxController.button(5).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // xboxController.button(6).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      xboxController.button(1).whileTrue(m_DiffyWristSubsystem.setAngle(Degrees.of(15), Degrees.of(15)));
      xboxController.button(2).whileTrue(m_DiffyWristSubsystem.setAngle(Degrees.of(30), Degrees.of(45)));
      xboxController.button(3).whileTrue(m_DiffyWristSubsystem.set(0,0));
      xboxController.button(4).whileTrue(m_DiffyWristSubsystem.set(0.5,0));
      xboxController.button(5).whileTrue(m_DiffyWristSubsystem.set(0,0.5));
      xboxController.button(6).whileTrue(m_DiffyWristSubsystem.setAngle(Degrees.of(0), Degrees.of(0)));
      xboxController.button(7).whileTrue(m_DiffyWristSubsystem.setAngle(Degrees.of(90), Degrees.of(45)));
  
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");  
  }
  
  public Command setSafePositions() {
    return Commands.runOnce(() -> {
     elevator.setHeight(ElevatorPosition.Down.distance());
     wrist.setAngle(WristPosition.Stowed.angle());
   }, elevator, wrist);
  }
}
