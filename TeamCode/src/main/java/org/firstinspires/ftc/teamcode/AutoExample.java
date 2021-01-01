package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoExample extends LinearOpMode {
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

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        leftMotor.setTargetPosition(500);
        rightMotor.setTargetPosition(500);

    }
        public void movePosition(int position) {
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            while (leftMotor.isBusy() || rightMotor.isBusy()) {
                //do nothing
            }
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setTargetPosition(position);
        rightMotor.setTargetPosition(position);
        leftMotor.setPower(1);
        rightMotor.setPower(1);



        //actual opmode
    }
}

