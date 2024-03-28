package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class IntakeCommand {
    private SingleMotorSubsystem intake;
    private Servo intakeServo;
    private CRServo intakeRoller;

    public IntakeCommand(HardwareMap hardwareMap) {
        intake = new SingleMotorSubsystem(hardwareMap, Specifications.INTAKE_MOTOR);
        intakeServo = hardwareMap.get(Servo.class, Specifications.INTAKE_SERVO);
        intakeRoller = hardwareMap.get(CRServo.class, Specifications.INTAKE_ROLLER);


        intakeRoller.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public double intakeValue() {
        return intakeServo.getPosition();
    }

    public void raiseIntake() {
        intakeServo.setPosition(0);
    }

    public void lowerIntake() {
        intakeServo.setPosition(0.4);
    }
    public void halfIntake(){intakeServo.setPosition(0.2);}
    public void intakeRollerIn(){
        intakeRoller.setPower(-1);
    }
    public void intakeRollerOut(){
        intakeRoller.setPower(1);
    }
    public void intakeRollerOut(double power){
        intakeRoller.setPower(power);
    }
    public void intakeRollerStop(){
        intakeRoller.setPower(0);
    }

    public void intakeIn(double power) {
        intake.motorTurnPurePower(true, Math.abs(power));
        intakeRollerIn();
    }

    public void slowIntakeIn(){
        intake.motorTurnPurePower(true, -0.5);
    }

    public void intakeOut(double power) {
        intake.motorTurnPurePower(true, -Math.abs(power));
    }

    public void stopIntake() {
        intake.motorTurnPurePower(true, 0);
        intakeRoller.setPower(0);
    }
}