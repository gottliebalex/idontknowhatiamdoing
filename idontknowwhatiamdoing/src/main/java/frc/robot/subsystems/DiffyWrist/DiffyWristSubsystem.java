package frc.robot.subsystems.DiffyWrist;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.DifferentialMechanismConfig;
import yams.mechanisms.positional.DifferentialMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import org.littletonrobotics.junction.Logger;

public class DiffyWristSubsystem extends SubsystemBase{
    private final SparkMax                   leftMotor  = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig leftConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(16, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
            .withGearing(gearing(gearbox(3, 4, 5)))
  //      .withExternalEncoder(armMotor.getAbsoluteEncoder())
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("LeftMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorController       leftSMC    = new SparkWrapper(leftMotor,
            DCMotor.getNEO(1),
            leftConfig);
    private final SparkMax                   rightMotor  = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig rightConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(16, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
            .withGearing(gearing(gearbox(3, 4, 5)))
  //      .withExternalEncoder(armMotor.getAbsoluteEncoder())
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("RightMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorController       rightSMC    = new SparkWrapper(rightMotor,
                                                                            DCMotor.getNEO(1),
                                                                            rightConfig);
    private final DifferentialMechanismConfig config = new DifferentialMechanismConfig(leftSMC, rightSMC)
            .withStartingPosition(Degrees.of(90), Degrees.of(0))
            .withMOI(Meters.of(0.3), Pounds.of(4))
            .withTelemetry("DiffyMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
    private final DifferentialMechanism diffy     = new DifferentialMechanism(config);
  
    public DiffyWristSubsystem()
    {
    }
  
    public Command setAngle(Angle tilt, Angle twist) {
      return diffy.setPosition(tilt, twist);
    }
  
    public Command set(double tilt, double twist) {
      return diffy.set(tilt, twist);
    }

    private static final double TILT_SIGN = +1.0, TWIST_SIGN = +1.0;

    private Measure<AngleUnit> tiltZero = Radians.of(0);
    private Measure<AngleUnit> twistZero = Radians.of(0);

  
    
    public void zeroJoints(){
      tiltZero = diffy.getTiltPosition();
      twistZero = diffy.getTwistPosition();
    }

    public void periodic()
    {
      diffy.updateTelemetry();

      double tiltRad  = (diffy.getTiltPosition().in(Radians)  - tiltZero.in(Radians))  * TILT_SIGN;
      double twistRad = (diffy.getTwistPosition().in(Radians) - twistZero.in(Radians)) * TWIST_SIGN;

      Logger.recordOutput("DiffyWrist/TiltRad", tiltRad);
      Logger.recordOutput("DiffyWrist/TwistRad", twistRad);

      Logger.recordOutput("Field/RobotPose", new Pose2d()); // (0,0,0)
    }
  
    public void simulationPeriodic()
    {
      diffy.simIterate();
    }

}

