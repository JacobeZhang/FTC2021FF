package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ShooterTesting extends LinearOpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            double leftPower, rightPower;
            leftPower = 0.5;
            rightPower = 0.5;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower,rightPower);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
