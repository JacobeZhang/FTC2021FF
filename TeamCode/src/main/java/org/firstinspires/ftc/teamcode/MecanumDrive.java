package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Mecanum Drive")
public class MecanumDrive extends LinearOpMode {
    private DcMotor FLDrive;
    private DcMotor FRDrive;
    private DcMotor BLDrive;
    private DcMotor BRDrive;
    private DcMotor IntakeMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        FLDrive = hardwareMap.get(DcMotor.class, "FL_DCmotor");
        FRDrive = hardwareMap.get(DcMotor.class, "FR_DCmotor");
        BLDrive = hardwareMap.get(DcMotor.class, "BL_DCmotor");
        BRDrive = hardwareMap.get(DcMotor.class, "BR_DCmotor");
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");

        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            boolean intake = gamepad1.right_bumper;
            double FLPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
            double FRPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            double BLPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
            double BRPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            if (intake){
                IntakeMotor.setPower(1);
            }
            else
                IntakeMotor.setPower(0);
            FLDrive.setPower(FLPower);
            FRDrive.setPower(FRPower);
            BLDrive.setPower(BLPower);
            BRDrive.setPower(BRPower);

            telemetry.addData("Mode", "Running");
            telemetry.addData("Power", "Frontleft=%.3f, Frontright=%.3f", FLPower, FRPower);
            telemetry.addData("Power", "Backleft=%.3f, Backright=%.3f", BLPower, BRPower);

            idle();
        }
    }
}
