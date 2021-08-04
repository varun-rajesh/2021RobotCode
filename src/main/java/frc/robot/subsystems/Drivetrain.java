/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.MotionOfTheOcean;
import frc.libs.drivetrains.Swerve;
import frc.libs.drivetrains.SwerveModule;
import frc.libs.electronics.GenericMotorController;
import frc.libs.electronics.GenericMotorController.ZeroBehavior;
import frc.libs.electronics.MA3Encoder;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Drivetrain extends SubsystemBase {

  private SwerveModule[] swerveModules;
  private Swerve swerve;

  private boolean isRecording;
  private boolean isPlaying;

  public Drivetrain() {
    swerveModules = new SwerveModule[RobotMap.MODULES.length];
    for (int i = 0; i < swerveModules.length; i++) {
      GenericMotorController drive = new GenericMotorController(new TalonFX(RobotMap.MODULES[i][0]));
      drive.setZeroBehavior(ZeroBehavior.COAST);
      drive.setInverted(true);

      GenericMotorController steer = new GenericMotorController(new VictorSPX(RobotMap.MODULES[i][1]));
      steer.setInverted(true);
      steer.setZeroBehavior(ZeroBehavior.COAST);

      MA3Encoder steerEncoder = new MA3Encoder(RobotMap.MODULES[i][2], Constants.MODULE_OFFSETS[i]);

      swerveModules[i] = new SwerveModule(drive, steer, steerEncoder);
      swerveModules[i].configureSteerPIDControllers(Constants.STEER_GAINS_HIGH, Constants.STEER_GAINS_LOW, Constants.STEER_GAINS_THRESHOLD);
      swerveModules[i].configureTicksPerFoot(Constants.TICKS_PER_FOOT);
    }

    swerve = new Swerve(swerveModules, Constants.MODULE_LOCATIONS, new PigeonIMU(RobotMap.GYRO));
    swerve.configurePIDControllers(Constants.DRIVE_GAINS, Constants.ROTATE_GAINS_LOW, Constants.ROTATE_GAINS_HIGH, Constants.ROTATE_GAINS_THRESHOLD);

    isRecording = false;
    isPlaying = false;
  }

  public void drive(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotate) {
    if (isPlaying) {
      MotionOfTheOcean.Executor.executeRecording(() -> swerve.toPose(MotionOfTheOcean.Executor.getState().getPose()));
    } else {
      swerve.controlModules(-0.3 * x.getAsDouble(), 0.3 * y.getAsDouble(), 0.1 * rotate.getAsDouble());
    }
  }

  public void toggleRecording() {
    isRecording = !isRecording;

    if (isRecording) {
      MotionOfTheOcean.Recorder.resetRecorder(swerve::reset);
    }
  }

  public void togglePlayback() {
    isPlaying = !isPlaying;
    MotionOfTheOcean.Executor.resetExecutor(swerve::reset);
    if (isPlaying) {
      MotionOfTheOcean.Executor.selectRecording("/home/lvuser/recording.csv");
      isRecording = false;
    }
  }

  public void reset() {
    swerve.reset();
  }

  public void resetGyro() {
    swerve.resetGyro();
  }

  @Override
  public void periodic() {
    if (isRecording) {
      MotionOfTheOcean.Recorder.recordState(swerve::getPose);
    }
  }
}
