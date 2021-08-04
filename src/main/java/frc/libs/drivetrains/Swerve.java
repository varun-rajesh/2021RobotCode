package frc.libs.drivetrains;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.controller.PIDController;

public class Swerve {
    
    private SwerveModule[] swerveModules;
    private double[][] moduleLocations;
    private PigeonIMU gyro;

    private PIDController translationalPIDController;
    private double[] translationalPIDControllerGains;

    private PIDController rotationalPIDController;
    private double[] rotationalPIDControllerGainsHigh;
    private double[] rotationalPIDControllerGainsLow;
    private double rotationalPIDControllerGainsThreshold;

    private double driveDeadband;
    private double holdAngleDeadband;

    private double gyroHoldAngle;
    private boolean hasHoldAngleBeenSet;

    /**
     * Constructor
     * 
     * @param swerveModules   array of Swerve Modules
     * @param moduleLocations relative positions of the Swerve Modules from center
     * @param gyro            PigeonIMU gyro
     */
    public Swerve(SwerveModule[] swerveModules, double[][] moduleLocations, PigeonIMU gyro) {
        this.swerveModules = swerveModules;
        this.moduleLocations = moduleLocations;
        this.gyro = gyro;
    }

    public void configurePIDControllers(double[] translationalPIDControllerGains, double[] rotationalPIDControllerGainsHigh, double[] rotationalPIDControllerGainsLow, double rotationalPIDControllerGainsThreshold) {
        this.translationalPIDControllerGains = translationalPIDControllerGains;
        this.rotationalPIDControllerGainsLow = rotationalPIDControllerGainsLow;
        this.rotationalPIDControllerGainsHigh = rotationalPIDControllerGainsHigh;
        this.rotationalPIDControllerGainsThreshold = rotationalPIDControllerGainsThreshold;
    }

    public void configureDeadbands(double driveDeadband, double holdAngleDeadband) {
        this.driveDeadband = driveDeadband;
        this.holdAngleDeadband = holdAngleDeadband;
    }

    /**
     * Control swerve modules given desired x, y, and rotate velocities
     * 
     * @param x      desired x velocity
     * @param y      desired y velocity
     * @param rotate desired rotate velocity
     */
    public void controlModules(double x, double y, double rotate) {

        // Only move if movement is large enough
        if (Math.hypot(x, y) < driveDeadband) {
            x = 0;
            y = 0;
        }

        if (Math.abs(rotate) < holdAngleDeadband) {
            if (!hasHoldAngleBeenSet) {
                gyroHoldAngle = getGyroAngle();
                hasHoldAngleBeenSet = true;
            } else {
                rotate = 0;
            }

            //Gain scheduling
            rotationalPIDController = Math.hypot(x, y) < rotationalPIDControllerGainsThreshold ?
                new PIDController(rotationalPIDControllerGainsHigh[0], rotationalPIDControllerGainsHigh[1], rotationalPIDControllerGainsHigh[2]) :
                new PIDController(rotationalPIDControllerGainsLow[0], rotationalPIDControllerGainsLow[1], rotationalPIDControllerGainsLow[2]);
            rotate = rotationalPIDController.calculate(getGyroAngle(), gyroHoldAngle);
        } else {
            hasHoldAngleBeenSet = false;
        }

        // Transform controls to be field centric
        double fieldCentricX = x;
        double fieldCentricY = y;

        fieldCentricX = Math.hypot(x, y) * Math.cos(Math.atan2(y, x) - getGyroAngle());
        fieldCentricY = Math.hypot(x, y) * Math.sin(Math.atan2(y, x) - getGyroAngle());

        double[][] vectors = new double[swerveModules.length][2];

        // Don't normalize unless max velocity > 1
        double maxVelocity = 1;

        // Compute Vectors for each module
        for (int i = 0; i < swerveModules.length; i++) {
            double velocityX = fieldCentricX
                    + rotate * moduleLocations[i][0] * Math.cos(moduleLocations[i][1] + Math.PI / 2);
            double velocityY = fieldCentricY
                    + rotate * moduleLocations[i][0] * Math.sin(moduleLocations[i][1] + Math.PI / 2);

            vectors[i][0] = Math.hypot(velocityX, velocityY);
            vectors[i][1] = Math.atan2(velocityY, velocityX);

            // Update max velocity to be normalized later
            maxVelocity = vectors[i][0] > maxVelocity ? vectors[i][0] : maxVelocity;
        }

        // Normalize Vectors for each module
        for (int i = 0; i < vectors.length; i++) {
            vectors[i][0] = vectors[i][0] / maxVelocity;
        }

        // Drive modules
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].drive(vectors[i], getGyroAngle());
        }
    }

    /**
     * Get the pose of the swerve drive
     * 
     * @return x, y, theta triplet
     */
    public double[] getPose() {
        double poseX = 0;
        double poseY = 0;
        for (int i = 0; i < swerveModules.length; i++) {
            poseX += swerveModules[i].getPose()[0] / swerveModules.length;
            poseY += swerveModules[i].getPose()[1] / swerveModules.length;
        }

        return new double[] { poseX, poseY, getGyroAngle() };
    }

    /**
     * Move the swerve drive to a specified pose
     * 
     * @param pose the specified pose
     */
    public void toPose(double[] pose) {
        double[] currentPose = getPose();
        double x = translationalPIDController.calculate(currentPose[0], pose[0]);
        double y = translationalPIDController.calculate(currentPose[1], pose[1]);

        double rotate = rotationalPIDController.calculate(currentPose[2], pose[2]);
        controlModules(x, y, rotate);
    }

    /**
     * Wrapper method to get the yaw of the swerve drive
     * 
     * @return yaw of the swerve drive
     */
    public double getGyroAngle() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return Math.toRadians(ypr[0]);
    }

    /**
     * Reset the swerve drive
     */
    public void reset() {
        for (SwerveModule module : swerveModules) {
            module.reset();
        }

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setPose(moduleLocations[i][0] * Math.cos(moduleLocations[i][1]),
                    moduleLocations[i][0] * Math.sin(moduleLocations[i][1]));
        }
        hasHoldAngleBeenSet = true;
    }


    /**
     * Reset the gyro
     */
    public void resetGyro() {
        gyroHoldAngle = 0;
        gyro.setYaw(0);
    }

}
