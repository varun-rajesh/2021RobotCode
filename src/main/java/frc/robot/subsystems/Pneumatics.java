package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.MotionOfTheOcean;
import frc.robot.RobotMap;

public class Pneumatics extends SubsystemBase {

  private Compressor compressor;
  private DoubleSolenoid intake;
  private DoubleSolenoid climberLock;
  private DoubleSolenoid hoodLock;
  private Solenoid hood;

  public Pneumatics() {
    compressor = new Compressor();
    intake = new DoubleSolenoid(RobotMap.INTAKE_EXTEND, RobotMap.INTAKE_RETRACT);
    climberLock = new DoubleSolenoid(RobotMap.CLIMBER_LOCK_EXTEND, RobotMap.CLIMBER_LOCK_RETRACT);
    hoodLock = new DoubleSolenoid(RobotMap.HOOD_LOCK_EXTEND, RobotMap.HOOD_LOCK_RETRACT);
    hood = new Solenoid(RobotMap.HOOD_EXTEND);
    
    MotionOfTheOcean.addCommand("Pneumatics.intakeToggle", this::intakeToggle);
  }

  public void compressorStart() {
    compressor.start();
  }

  public void compressorStop() {
    compressor.stop();
  }

  public void compressorToggle() {
    if(compressor.enabled()) {
      compressor.stop();
    } else {
      compressor.start();
    }
  }

  public void intakeExtend() {
    intake.set(Value.kForward);
  }

  public void intakeRetract() {
    intake.set(Value.kReverse);
  }

  public void intakeToggle() {
    MotionOfTheOcean.Recorder.recordCommand("Pneumatics.intakeToggle");
    if(intake.get() == Value.kForward) {
      intakeRetract();
    } else {
      intakeExtend();
    }
  }

  public void hoodExtend() {
    hood.set(true);
  }

  public void hoodRetract() {
    hood.set(false);
  }

  public void hoodToggle() {
    if(hood.get()) {
      hoodRetract();
    } else {
      hoodExtend();
    }
  }

  public void hoodLockExtend() {
    hoodLock.set(Value.kForward);
  }

  public void hoodLockRetract() {
    hoodLock.set(Value.kReverse);
  }

  public void hoodLockToggle() {
    if(hoodLock.get() == Value.kForward) {
      hoodLockRetract();
    } else {
      hoodLockExtend();
    }
  }

  public void climberLockExtend() {
    climberLock.set(Value.kForward);
  }

  public void climberLockRetract() {
    climberLock.set(Value.kReverse);
  }

  public void climberLockToggle() {
    if(climberLock.get() == Value.kForward) {
      climberLockRetract();
    } else {
      climberLockExtend();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("compressor", compressor.enabled());
  }
}
