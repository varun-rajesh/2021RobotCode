package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.libs.MotionOfTheOcean;
import frc.libs.electronics.GenericMotorController;
import frc.robot.RobotMap;

public class ActiveFloor extends SubsystemBase {

  private GenericMotorController floor;
  private GenericMotorController queuer;

  public ActiveFloor() {
    floor = new GenericMotorController(new VictorSPX(RobotMap.FLOOR));
    queuer = new GenericMotorController(new VictorSPX(RobotMap.QUEUER));

    MotionOfTheOcean.addCommand("ActiveFloor.runIn", this::runIn);
    MotionOfTheOcean.addCommand("ActiveFloor.runOut", this::runOut);
    MotionOfTheOcean.addCommand("ActiveFloor.stop", this::stop);
  }

  public void drive(DoubleSupplier floor, DoubleSupplier queuer) {
    this.floor.smartSet(-floor.getAsDouble());
    this.queuer.smartSet(queuer.getAsDouble());
    System.out.println(floor.getAsDouble() + ", " + queuer.getAsDouble());
  }

  public void runIn() {
    MotionOfTheOcean.Recorder.recordCommand("ActiveFloor.runIn");
    floor.set(-0.65);
    queuer.set(0.85);
  }

  public void stop() {
    MotionOfTheOcean.Recorder.recordCommand("ActiveFloor.stop");
    floor.smartSet(0);
    queuer.smartSet(0);
  }

  public void runOut() {
    MotionOfTheOcean.Recorder.recordCommand("ActiveFloor.runOut");
    floor.smartSet(0.8);
    queuer.smartSet(-1);
  }

  @Override
  public void periodic() {

  }
}
