package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleOpExample extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        //initialization
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        arm = hardwareMap.get(Servo.class, "arm");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            leftMotor.setPower(-gamepad1.left_stick_y);
            rightMotor.setPower(-gamepad1.right_stick_y);
            if (gamepad1.a) {
                arm.setPosition(1);
            }
            else if(gamepad1.b){
                arm.setPosition(0.5);
            }

            else {
                arm.setPosition(0);
            }
        }
        //actual opmode
    }
}
