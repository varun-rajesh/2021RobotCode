/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  private TalonSRX climber;
  private VictorSPX spinner;

  public Climber() {
    climber = new TalonSRX(RobotMap.CLIMBER);
    spinner = new VictorSPX(RobotMap.SPINNER);
  }

  public void driveClimber(DoubleSupplier speed) {
    climber.set(ControlMode.PercentOutput, speed.getAsDouble());
  }

  public void runSpinner() {
    spinner.set(ControlMode.PercentOutput, 1);
  }

  public void stopSpinner() {
    spinner.set(ControlMode.PercentOutput, 0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
