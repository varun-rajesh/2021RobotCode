package frc.libs.drivetrains;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.libs.electronics.GenericMotorController;
import frc.libs.electronics.MA3Encoder;

public class SwerveModule {
    
    private PIDController steerPIDController;
    private double[] steerPIDControllerGainsHigh;
    private double[] steerPIDControllerGainsLow;
    private double steerPIDControllerGainsThreshold;

    private GenericMotorController drive;
    private GenericMotorController steer;
    private MA3Encoder steerEncoder;

    private double ticksPerFoot;

    private double lastX = 0;
    private double lastY = 0;
    private double lastDrivePosition = 0;

    /**
     * Constructor 
     * 
     * @param drive GenericMotorController of the drive motor
     * @param steer GenericMotorController of the steer motor
     * @param steerEncoder MA3Encoder of the steer encoder
     */
    public SwerveModule(GenericMotorController drive, GenericMotorController steer, MA3Encoder steerEncoder) {
        this.drive = drive;
        this.steer = steer;
        this.steerEncoder = steerEncoder;

        this.ticksPerFoot = 1;

        this.reset();
    }

    /**
     * Configure the gains of the steer PID controller
     * 
     * @param steerGains PID gains for the steer PID controller
     */
    public void configureSteerPIDControllers(double[] steerPIDControllerGainsHigh, double[] steerPIDControllerGainsLow, double steerPIDControllerGainsThreshold) {
        this.steerPIDControllerGainsHigh = steerPIDControllerGainsHigh;
        this.steerPIDControllerGainsLow = steerPIDControllerGainsLow;
        this.steerPIDControllerGainsThreshold = steerPIDControllerGainsThreshold;
    }

    /**
     * Set the number of drive encoder ticks per foot of motion
     * 
     * @param ticksPerFoot Drive encoder ticks per foot
     */
    public void configureTicksPerFoot(double ticksPerFoot) {
        this.ticksPerFoot = ticksPerFoot;
    }

    /**
     * Drive a module
     * 
     * @param vector direction and magnitude of desired swerve motion
     * @param gyro yaw of swerve drive
     */
    public void drive(double[] vector, double gyro) {
        double[] velocityAndError = minimizeHeadingChange(vector);

        if (velocityAndError[0] == 0) {
            drive.smartSet(0);
            steer.smartSet(0);
        } else {

            // Gain scheduling
            steerPIDController = velocityAndError[0] < steerPIDControllerGainsThreshold ? 
                new PIDController(steerPIDControllerGainsHigh[0], steerPIDControllerGainsHigh[1], steerPIDControllerGainsHigh[2]) : 
                new PIDController(steerPIDControllerGainsLow[0], steerPIDControllerGainsLow[1], steerPIDControllerGainsLow[2]);

            drive.smartSet(velocityAndError[0]);
            steer.smartSet(steerPIDController.calculate(velocityAndError[1]));
        }

        double absoluteHeading = steerEncoder.getValue() + gyro;
        lastX += (drive.getPosition() - lastDrivePosition) * Math.cos(absoluteHeading) / ticksPerFoot;
        lastY += (drive.getPosition() - lastDrivePosition) * Math.sin(absoluteHeading) / ticksPerFoot;

        lastDrivePosition = drive.getPosition();
    }

    /**
     * Choose orientation of module to reduce module yaw change
     * 
     * @param vector Desired direction and magnitude of module
     * @return Transformed direction and magnitude 
     */
    public double[] minimizeHeadingChange(double[] vector) {
        double error = (vector[1] - steerEncoder.getValue()) % (2 * Math.PI);

        // Clamp error such that -pi <= error <= pi
        if (error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }

        // Choose to run module in reverse
        if (error > (Math.PI / 2)) {
            vector[0] = -vector[0];
            error -= Math.PI;
        } else if (error < (-Math.PI / 2)) {
            vector[0] = -vector[0];
            error += Math.PI;
        }

        return new double[] { vector[0], error };
    }

    /**
     * Get the pose of the module
     * 
     * @return x, y pose of the module
     */
    public double[] getPose() {
        return new double[] { lastX, lastY };
    }

    /**
     * Set the intial pose of the module
     * 
     * @param x x-coordinate of the module
     * @param y y-coordinate of the module
     */
    public void setPose(double x, double y) {
        lastX = x;
        lastY = y;
        lastDrivePosition = 0;
    }

    /**
     * Reset the swerve module
     */
    public void reset() {
        lastDrivePosition = 0;
        drive.resetEncoder();
        steer.resetEncoder();
        steerEncoder.reset();
    }
}
