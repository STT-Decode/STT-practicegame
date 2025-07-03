package org.firstinspires.ftc.teamcode.RobotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class All_Parts
{
    private DcMotorEx lf;
    private DcMotorEx rf;
    private DcMotorEx lb;
    private DcMotorEx rb;
    private DcMotorEx arm;
    private Servo claw;
    private Servo rotationServo;
    private double maxPower;


    public void init(HardwareMap map)
    {

        lf = map.get(DcMotorEx.class, "left_front");
        rf = map.get(DcMotorEx.class, "right_front");
        lb = map.get(DcMotorEx.class, "left_back");
        rb = map.get(DcMotorEx.class, "right_back");
        arm = map.get(DcMotorEx.class, "arm");
        claw = map.get(Servo.class, "claw");
        rotationServo = map.get(Servo.class,"rotationServo");

        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public void drive0(double forward, double right, double rotate, double power)
    {
        double leftFrontPower = -forward - right + rotate;
        double rightFrontPower = -forward + right - rotate;
        double rightRearPower = -forward - right - rotate;
        double leftRearPower = -forward + right + rotate;

        maxPower = power;
        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));

        lf.setPower(leftFrontPower / maxPower);
        rf.setPower(-rightFrontPower / maxPower);
        rb.setPower(-rightRearPower / maxPower);
        lb.setPower(leftRearPower / maxPower);
    }

    public void drive1(float fl, float fr, float br, float bl, double weight)
    {
        lf.setPower(fl / weight);
        rf.setPower(fr / weight);
        rb.setPower(br / weight);
        lb.setPower(bl / weight);
    }

    public void setArmPower(double power)
    {
        arm.setPower(power);
    }

    public void setClawPos(double pos)
    {
        claw.setPosition(pos);
    }

    public void setRotationPos(double pos)
    {
        rotationServo.setPosition(pos);
    }

}