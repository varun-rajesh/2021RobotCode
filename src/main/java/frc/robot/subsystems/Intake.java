/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.MotionOfTheOcean;
import frc.libs.electronics.GenericMotorController;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {
  
  private GenericMotorController intake;

  public Intake() {
    intake = new GenericMotorController(new VictorSPX(RobotMap.INTAKE));
    MotionOfTheOcean.addCommand("Intake.runIntake", this::runIntake);
    MotionOfTheOcean.addCommand("Intake.stopIntake", this::stopIntake);
  } 

  public void drive(DoubleSupplier speed) {
    intake.smartSet(speed.getAsDouble());
  }

  public void runIntake() {
    MotionOfTheOcean.Recorder.recordCommand("Intake.runIntake");
    
    intake.smartSet(0.5);
  }

  public void stopIntake() {
    MotionOfTheOcean.Recorder.recordCommand("Intake.stopIntake");
    
    intake.smartSet(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
