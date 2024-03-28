package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.movement.TimerList;

public class OutputCommand {

    //Assuming lift is at the front of robot and intake at back
    private SingleMotorSubsystem hang;
    private Servo leftArm;
    private Servo rightArm;
    private Servo leftTilt;
    private Servo rightTilt;

    private Servo droneShooter;
    private Servo gate;
    private IntakeCommand intakeCommand;
    private TimerList timers = new TimerList();
//    private MultiMotorCommand multiMotorCommand;


    public OutputCommand(HardwareMap hardwareMap) {
        hang = new SingleMotorSubsystem(hardwareMap, Specifications.HANG_MOTOR);
        leftArm = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        rightArm = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
        leftTilt = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        droneShooter = hardwareMap.get(Servo.class, Specifications.DRONE_LAUNCHER);
        rightTilt = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);
        gate = hardwareMap.get(Servo.class, Specifications.PIXEL_GATE);
        intakeCommand = new IntakeCommand(hardwareMap);

        leftArm.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.FORWARD);
        droneShooter.setDirection(Servo.Direction.FORWARD);

        leftTilt.setDirection(Servo.Direction.FORWARD);
        rightTilt.setDirection(Servo.Direction.REVERSE);

        gate.setDirection(Servo.Direction.FORWARD);

    }

    public void initialize(){
        closeGate();
        armToIdle();
        tiltToIdle();

    }

    public void droneToShoot(){
        droneShooter.setPosition(0);
    }
    public void droneToNotShoot(){
        droneShooter.setPosition(0.2);
    }

    public void droneToIdle(){
        droneShooter.setPosition(0.5);
    }
    public void openGate(){
        gate.setPosition(0.445);
    }
    public void closeGate(){
        gate.setPosition(0.55);
    }
    public void outputWheelOut(){
        intakeCommand.intakeRollerOut();
    }
    public void outputWheelIn(){
        intakeCommand.intakeRollerIn();
    }
    public void outputWheelStop(){
        intakeCommand.intakeRollerStop();
    }
    public void armToPos(double position){
        leftArm.setPosition(position);
        rightArm.setPosition(position);
    }
    public void armToIdle(){
        //TODO: Find value
        leftArm.setPosition(0.285);
        rightArm.setPosition(0.285);
    }

    public void armToBoard(){
        leftArm.setPosition(0.455);
        rightArm.setPosition(0.455);
    }

    public void tiltToIdle(){
        //leftTilt.setPosition(0.125);
        //rightTilt.setPosition(0.125);
        leftTilt.setPosition(0.05);
        rightTilt.setPosition(0.05);
    }
    public void tiltToBoard(){
        //leftTilt.setPosition(0.955);
        //rightTilt.setPosition(0.955);
        leftTilt.setPosition(0.95);
        rightTilt.setPosition(0.95);
    }
    public void hangArmUp(double power) {
        hang.motorTurnPurePower(true, Math.abs(power));
    }
    public void hangArmDown(double power) {
        hang.motorTurnPurePower(true, -Math.abs(power));
    }

    public double getGatePosition(){
        return gate.getPosition();
    }
    public double getLeftArmPosition(){
        return leftArm.getPosition();
    }
    public double getRightArmPosition(){
        return rightArm.getPosition();
    }
    public double getLeftTiltPosition(){
        return leftTilt.getPosition();
    }
    public double getRightTiltPosition(){
        return rightTilt.getPosition();
    }

    public Servo getleftTilt(){
        return leftTilt;
    }

    public Servo getRightTile(){
        return rightTilt;
    }


}
