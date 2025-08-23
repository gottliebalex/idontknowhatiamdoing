package frc.robot.subsystems.Elevator;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import com.ctre.phoenix6.controls.Follower;

import frc.robot.subsystems.SubsystemConstants;

public class ElevatorSubsystem extends SubsystemBase
{
  private final TalonFX                   elevatorMotor = new TalonFX(SubsystemConstants.ElevatorLEADER_ID, SubsystemConstants.CANBUS);
  private final TalonFX                   elevatorFollower = new TalonFX(SubsystemConstants.ElevatorFOLLOWER_ID, SubsystemConstants.CANBUS);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
      .withMechanismCircumference(Meters.of(Millimeters.of(5).in(Meters) * 36))
      .withClosedLoopController(5.84, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2))
      .withSimClosedLoopController(5, 0, 0, MetersPerSecond.of(2), MetersPerSecondPerSecond.of(2))
      .withSoftLimit(Meters.of(0), Meters.of(2.5))
      .withGearing(gearing(gearbox(2.4444444, 2.8)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(60))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25))
//      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ElevatorFeedforward(0.021, 0.232, 4.4959, 0.10008))
      .withSimFeedforward(new ElevatorFeedforward(0,0,0,0))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withFollowers(Pair.of(elevatorFollower,true));
      
  private final SmartMotorController       motor         = new TalonFXWrapper(elevatorMotor,
                                                                            DCMotor.getKrakenX60(2),
                                                                            motorConfig);
  private final MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private       ElevatorConfig          m_config           = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0.25))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(m_robotToMechanism)
      .withMass(Pounds.of(15));
  private final Elevator                m_elevator         = new Elevator(m_config);

  public static enum ElevatorPosition {
    Down(Inches.of(10)),
    Intake(Inches.of(16)),
    L1(Inches.of(10)),
    L2(Inches.of(36)),
    L3(Inches.of(50)),
    L4(Inches.of(80)),
    L2Algae(Inches.of(32)),
    L3Algae(Inches.of(45)),
    Barge(Inches.of(85));

    private final Distance distance;

    ElevatorPosition(Distance distance) {
      this.distance = distance;
    }

    public Distance distance() {
      return distance;
    }
  }


  public ElevatorSubsystem()
  {
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();

  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

  public Distance getHeight() {
    return m_elevator.getHeight();
  }

  public Command sysId()
  {
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }
}
