package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareBIGBRAINBOTS.HardwareBIGBRAINBOTS;


@TeleOp(name = "BBBTeleOp")
public class BBBTeleOp extends LinearOpMode {
    HardwareBIGBRAINBOTS robot   = new HardwareBIGBRAINBOTS();   // Use BIGBRAINBOTS's hardware

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            boolean intake = gamepad1.left_bumper;
            boolean transfer = gamepad1.right_bumper;
            boolean armup = gamepad1.y;
            boolean armdown = gamepad1.a;
            boolean drop = gamepad1.x;
            boolean clawopen = gamepad1.dpad_left;
            boolean clawclose = gamepad1.dpad_right;


            double FLPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
            double FRPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            double BLPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
            double BRPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            if (intake){
                robot.IntakeDrive.setPower(1);
            }
            else
                robot.IntakeDrive.setPower(0);

            if (transfer){
                robot.TransferDrive.setPower(1);
            }
            else
                robot.TransferDrive.setPower(0);

            if (armup) {
                robot.WobbleGoalArmDrive.setPosition(Servo.MAX_POSITION);
            }
            if (armdown) {
                robot.WobbleGoalArmDrive.setPosition(Servo.MIN_POSITION); //going to be edited later prob
            }
            if (drop) {
                robot.WobbleGoalArmDrive.setPosition(Servo.MAX_POSITION / 2);
                robot.WobbleGoalClaw.setPosition(Servo.MIN_POSITION);
            }
            if (clawopen) {
                robot.WobbleGoalClaw.setPosition(Servo.MIN_POSITION);
            }
            if (clawclose) {
                robot.WobbleGoalClaw.setPosition(Servo.MAX_POSITION);
            }

            robot.FrontLeftDrive.setPower(FLPower);
            robot.FrontRightDrive.setPower(FRPower);
            robot.RearLeftDrive.setPower(BLPower);
            robot.RearRightDrive.setPower(BRPower);

            telemetry.addData("Mode", "Running");
            telemetry.addData("Power", "Frontleft=%.3f, Frontright=%.3f", FLPower, FRPower);
            telemetry.addData("Power", "Backleft=%.3f, Backright=%.3f", BLPower, BRPower);

            idle();
        }
    }
}
