package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareBIGBRAINBOTS;


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
            double strafe = gamepad1.right_stick_x;
            double turn = gamepad1.left_stick_x;
            boolean armup = gamepad2.dpad_up;
            boolean armdown = gamepad2.dpad_down;
            boolean clawopen = gamepad2.dpad_left;
            boolean clawclose = gamepad2.dpad_right;
            boolean IntakeTransferForward = gamepad1.right_bumper;
            boolean IntakeTransferReverse = gamepad1.left_bumper;
            boolean shooteron = gamepad2.b;
            boolean shooteroff = gamepad2.x;
            boolean shooterpush = gamepad2.y;
            double FLPower = Range.clip(-drive + strafe + turn, -1.0, 1.0);
            double FRPower = Range.clip(-drive - strafe - turn, -1.0, 1.0);
            double BLPower = Range.clip(-drive - strafe + turn, -1.0, 1.0);
            double BRPower = Range.clip(-drive + strafe - turn, -1.0, 1.0);

            if (IntakeTransferForward) {
                robot.IntakeTransferDrive.setPower(1);
            }
            if (IntakeTransferReverse) {
                robot.IntakeTransferDrive.setPower(-1);
            }
            if (armdown) {
                robot.WobbleGoalArmDrive.setTargetPosition(-200);
                robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.WobbleGoalArmDrive.setPower(-0.25);
                while (robot.WobbleGoalArmDrive.isBusy()) {

                }
                robot.WobbleGoalArmDrive.setPower(0);
            }
            else {
                robot.WobbleGoalArmDrive.setTargetPosition(0);
                robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.WobbleGoalArmDrive.setPower(0.1);
                while (robot.WobbleGoalArmDrive.isBusy()) {

                }
                robot.WobbleGoalArmDrive.setPower(0);
            }

            if (armup) {
                robot.WobbleGoalArmDrive.setTargetPosition(200);
                robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.WobbleGoalArmDrive.setPower(1);
                while (robot.WobbleGoalArmDrive.isBusy()) {

                }
                robot.WobbleGoalArmDrive.setPower(0);
            }
          //  else {
            //    robot.WobbleGoalArmDrive.setPower(0);
            //}
            if (clawopen) {
                robot.WobbleGoalClaw.setPosition(Servo.MIN_POSITION);
            }
            if (clawclose) {
                robot.WobbleGoalClaw.setPosition(Servo.MAX_POSITION);
            }
            //checks if the button wasn't pressed last loop but is pressed this loop
            if (shooterpush){
                robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);
            }
            else {
                robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
            }
            if (shooteron) {
                robot.ShooterFlywheel.setPower(1);
            }
            if (shooteroff) {
                robot.ShooterFlywheel.setPower(0);
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
