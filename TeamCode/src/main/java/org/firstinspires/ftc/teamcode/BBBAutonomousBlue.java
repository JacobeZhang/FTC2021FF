package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "BBBAutonomousBlue")
public class BBBAutonomousBlue extends LinearOpMode {
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();   // Use BIGBRAINBOTS's hardware

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();
        robot.resetEncoders();
        robot.drive(-0.25,-2500);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.strafe(0.25,-500);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.ShooterFlywheel.setPower(1);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);
        sleep(1500);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);
        sleep(1500);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);
        sleep(1500);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(1000);
        robot.resetEncoders();
        robot.drive(-0.25,-400);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.WobbleGoalArmDrive.setTargetPosition(-350);
        robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WobbleGoalArmDrive.setPower(-0.25);
        while (robot.WobbleGoalArmDrive.isBusy()) {

        }
        robot.WobbleGoalArmDrive.setPower(0);
        robot.WobbleGoalClaw.setPosition(Servo.MAX_POSITION);
        sleep(3000);
        robot.WobbleGoalArmDrive.setTargetPosition(250);
        robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WobbleGoalArmDrive.setPower(0.25);
        while (robot.WobbleGoalArmDrive.isBusy()) {

        }
        robot.WobbleGoalArmDrive.setPower(0);
        telemetry.addData("Over", "0");
        telemetry.update();


    }
}
