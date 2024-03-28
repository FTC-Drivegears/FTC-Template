package org.firstinspires.ftc.teamcode.util;

//Cir is circumference

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class Specifications {
    public final double length = 43.18;
    public final int FINALPOS = 1230;
    //b
    //length front to middle = 15.91
    public final double SideOdometryToCentre = 12.8;

    //making this zero makes things super godo for some reason
    public final double lengthFromOdometrySideToFront = 0.52;
    public final double odometryCir = 3.5*Math.PI;
    public final double odometryTick = 8192;
    //beta
    public final double sideOdometryAngleFromCentre = 0;
    public final double frontOdometryAngleFromCentre = Math.atan(15.91/7.2);


    public static double WHEEL_RADIUS = 4.8;
    public static final double MAX_RPM = 5400;
    public static double MAX_ANGULAR_VEL = MAX_RPM/30*Math.PI;
    public static double MAX_VEL = MAX_ANGULAR_VEL*WHEEL_RADIUS;
    public static double MIN_VEL = -MAX_VEL;
    public static final String INTAKE_MOTOR = "intakeMotor";
    public static final String HANG_MOTOR = "rightEncoder";
    public static final String INTAKE_SERVO = "intakeLinkage";
    public static final String INTAKE_ROLLER = "intakeRoller";
    public static final String EXTENSION_MOTOR_MAIN = "extension1";
    public static final String EXTENSION_MOTOR_AUX1 = "extension2";
    public static final String FTLF_MOTOR = "leftForward";
    public static final String FTRT_MOTOR = "rightForward";
    public static final String BKLF_MOTOR = "leftBack";
    public static final String BKRT_MOTOR = "rightBack";
    public static final String BK_ENCODER = "intakeMotor"; //3
    public static final String LF_ENCODER = "extension2"; //1 control
    public static final String RT_ENCODER = "rightEncoder"; //2 control
    public static final String IMU = "imu";

    //output stuff, fill in names later
    public static final String LEFT_OUTPUT_ARM = "leftArm";
    public static final String RIGHT_OUTPUT_ARM = "rightArm";
    public static final String LEFT_OUTPUT_TILT = "leftTilt";
    public static final String RIGHT_OUTPUT_TILT = "rightTilt";
    public static final String PIXEL_GATE = "pixelGate";

    public static final String DRONE_LAUNCHER = "droneShooter";
    public static final String FIRST_COLOR_SENSOR = "pixelSensor";
    public static final String SECOND_COLOR_SENSOR = "pixelSensor2";
    public static final String LED = "led";

    public enum NavSystem{
        IMU,
        ODOMETRY,
        MIXED
    }
    public NavSystem navSystem = NavSystem.ODOMETRY;

    public Specifications() {
    }
}
