package frc.libs.electronics;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class GenericMotorController {

    private TalonFX falcon;
    private TalonSRX talon;
    private VictorSPX victor;
    private CANSparkMax spark;
    private int deviceID;

    private enum MotorType {FALCON, TALON, VICTOR, SPARK}
    private MotorType motorType;

    public enum ZeroBehavior {BRAKE, COAST}
    
    public GenericMotorController(TalonFX falcon) {
        this.falcon = falcon;
        this.deviceID = falcon.getDeviceID();
        motorType = MotorType.FALCON;
    }

    public GenericMotorController(TalonSRX talon) {
        this.talon = talon;
        this.deviceID = talon.getDeviceID();
        motorType = MotorType.TALON;
    }

    public GenericMotorController(VictorSPX victor) {
        this.victor = victor;
        this.deviceID = victor.getDeviceID();
        motorType = MotorType.VICTOR;
    }

    public GenericMotorController(CANSparkMax spark) {
        this.spark = spark;
        this.deviceID = spark.getDeviceId();
        motorType = MotorType.SPARK;
    }

    public void set(double speed) {
        switch(motorType) {
            case FALCON:
                falcon.set(ControlMode.PercentOutput, speed);
                break;
            case TALON:
                talon.set(ControlMode.PercentOutput, speed);
                break;
            case VICTOR:
                victor.set(ControlMode.PercentOutput, speed);
                break;
            case SPARK:
                spark.set(speed);
                break;
            default:
                break;
            
        }
    }

    private double lastSpeed;
    public void smartSet(double speed) {
        if (speed != lastSpeed) set(speed);
        lastSpeed = speed;
    }

    public double getPosition() {
        switch (motorType) {
            case FALCON:
                return falcon.getSelectedSensorPosition();
            case TALON:
                return talon.getSelectedSensorPosition();
            case SPARK:
                return spark.getEncoder().getPosition();
            default:
                return 0;
        }
    }

    public double getVelocity() {
        switch (motorType) {
            case FALCON:
                return falcon.getSelectedSensorVelocity();
            case TALON:
                return talon.getSelectedSensorVelocity();
            case SPARK:
                return spark.getEncoder().getVelocity();
            default:
                return 0;
        }
    }

    public boolean setInverted(boolean isInverted) {
        switch (motorType) {
            case FALCON:
                falcon.setInverted(isInverted);
                return true;
            case TALON:
                talon.setInverted(isInverted);
                return true;
            case VICTOR:
                victor.setInverted(isInverted);
                return true;
            case SPARK:
                spark.setInverted(isInverted);
                return true;
            default:
                return false;
        }
    }

    public void setZeroBehavior(ZeroBehavior behavior) {
        switch(motorType) {
            case FALCON:
                falcon.setNeutralMode(behavior == ZeroBehavior.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
                break;
            case TALON:
                talon.setNeutralMode(behavior == ZeroBehavior.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
                break;
            case VICTOR:
                victor.setNeutralMode(behavior == ZeroBehavior.BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
                break;
            case SPARK:
                spark.setIdleMode(behavior == ZeroBehavior.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
                break;
            default:
                break;
        }
    }

    public void resetEncoder() {
        switch (motorType) {
            case FALCON:
                falcon.setSelectedSensorPosition(0);
                break;
            case TALON:
                talon.setSelectedSensorPosition(0);
                break;
            case SPARK:
                spark.getEncoder().setPosition(0);
                break;
            default:
                break;
        }
    }
}
