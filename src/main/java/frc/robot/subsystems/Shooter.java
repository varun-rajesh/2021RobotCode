package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.electronics.GenericMotorController;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

  private GenericMotorController shooter1;
  private GenericMotorController shooter2;
  private PIDController controller;
  private double setPoint;
  private boolean toggle;

  public Shooter() {
    shooter1 = new GenericMotorController(new TalonFX(RobotMap.SHOOTER_1));
    shooter2 = new GenericMotorController(new TalonFX(RobotMap.SHOOTER_2));
    shooter1.setInverted(true);
    controller = new PIDController(0.0005, 0.000, 0);
  }

  public void run() {
    if(toggle) {
      double feedForward = setPoint * 0.0000526715 + 0.0413908;
      shooter1.smartSet(feedForward + controller.calculate(shooter2.getVelocity(), setPoint));
      shooter2.smartSet(feedForward + controller.calculate(shooter2.getVelocity(), setPoint));
    } else {
      stop();
    }
  }

  public void setSetpoint(double speed) {
    this.setPoint = speed;
  }

  public void toggle() {
    toggle = !toggle;
  }

  public void stop() {
    shooter1.smartSet(0);
    shooter2.smartSet(0);
  }

  @Override
  public void periodic() {
  }
}
