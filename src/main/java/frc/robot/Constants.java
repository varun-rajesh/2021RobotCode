/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants {
    public static final double SHOOTING_POSITION = -0.2;

    public static int[] MODULE_OFFSETS = new int[] {886, 1755, 1074, 1572};
    public static double[][] MODULE_LOCATIONS = new double[][] {{1.3275, .830}, {1.3275, 2.312}, {1.3275, 3.971}, {1.3275, 5.453}};
    public static double[] STEER_GAINS_HIGH = new double[] {0.6, 0.0, 0};
    public static double[] STEER_GAINS_LOW = new double[] {0.4, 0.0, 0};
    public static double STEER_GAINS_THRESHOLD = 0.05;
    public static double TICKS_PER_FOOT = 12520; 

    public static double[] DRIVE_GAINS = new double[] {0.38, 0, 0};
    public static double[] ROTATE_GAINS_LOW = new double[] {0.2, 0, 0};
    public static double[] ROTATE_GAINS_HIGH = new double[] {0.5, 0, 0};
    public static double ROTATE_GAINS_THRESHOLD = 0.05;
}
