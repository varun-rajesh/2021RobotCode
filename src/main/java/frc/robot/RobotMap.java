package frc.robot;

public class RobotMap {
    
    public static final int[][] MODULES = new int[][] {
        {0, 10, 0},
        {2, 12, 1},
        {3, 13, 2},
        {1, 11, 3}
    };

    public static final int GYRO = 0;

    public static final int INTAKE = 16;
    public static final int FLOOR = 15;
    public static final int QUEUER = 14;
    public static final int SHOOTER_1 = 5;
    public static final int SHOOTER_2 = 4;
    public static final int CLIMBER = 6;
    public static final int SPINNER = 17;

    public static final int INTAKE_EXTEND = 0;
    public static final int INTAKE_RETRACT = 1;
    public static final int HOOD_EXTEND = 4;
    public static final int HOOD_LOCK_EXTEND = 6;
    public static final int HOOD_LOCK_RETRACT = 7;
    public static final int CLIMBER_LOCK_EXTEND = 2;
    public static final int CLIMBER_LOCK_RETRACT = 3;

}
