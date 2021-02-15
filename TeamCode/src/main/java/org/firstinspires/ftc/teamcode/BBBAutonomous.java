package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HardwareBIGBRAINBOTS;

import java.util.List;

import static java.lang.Math.abs;

@Autonomous (name = "BBBAutonomous")
public class BBBAutonomous extends LinearOpMode {
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();   // Use BIGBRAINBOTS's hardware
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AbhyEVr/////AAABmZM8yCLGH0QTjtItBa5rMYRy02Ki/kP8EoxIHn4Ppn0WctnbSjg1lXOvzubEKaIMeNZa6Nm888zj2QpeyEnhHmrMhDQZdMc8Bm2edRuJHZQElwdFrJNIPSw79HQlZ9ZcNDrWCJRqKrnqVwhxiASqbE6/tbTMTZbL7p/ImK9VdkhTajlntfNZYe1NZy75vq1CrtIHfn66TXK61TBTxITKcuT/m/zAWFLcxfv3f7SvdmaQEtvVBqXwyiDp/z2I+9gCcah7h9VFSrjVsbhsXKTRXef/PTnWTvy5RLYC5eM1u77PC+9ugrD6a/6lHAs+r9rOnTEkSNsNmDcXReKQqIpOkFwZLZpdhPD3LPzAYjnIWuwS";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        //initVuforia();
        //initTfod();
        robot.init(this.hardwareMap);
        telemetry.addData("Mode", "waiting");
        telemetry.update();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 1.78);
        }
        waitForStart();
        robot.drive(-0.50, -1200);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.strafe(-0.25, -300);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.ShooterFlywheel.setPower(1);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);
        sleep(1000);
        robot.strafe(-0.25, 300);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(1000);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.1);
        sleep(1000);
        robot.strafe(-0.25, 300);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(750);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION - 0.05);
        sleep(750);
        robot.ShooterPush.setPosition(Servo.MAX_POSITION / 2);
        sleep(750);
        robot.resetEncoders();
        robot.drive(0.50, 500);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.strafe(0.50, 1500);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.drive(-0.50, -3500);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        robot.resetEncoders();
        robot.WobbleGoalClaw.setPosition(Servo.MIN_POSITION + 0.1);
        sleep(1000);
        robot.WobbleGoalArmDrive.setTargetPosition(-300);
        robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WobbleGoalArmDrive.setPower(-0.10);
        while (abs(robot.WobbleGoalArmDrive.getCurrentPosition() - (-250)) > 100) {

        }
        sleep(1000);
        robot.WobbleGoalClaw.setPosition(Servo.MAX_POSITION / 2);
        robot.WobbleGoalArmDrive.setPower(0);
        robot.WobbleGoalArmDrive.setTargetPosition(0);
        robot.WobbleGoalArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.WobbleGoalArmDrive.setPower(0.1);
        while (robot.WobbleGoalArmDrive.isBusy()) {

        }
        robot.WobbleGoalArmDrive.setPower(0);
        robot.drive(1, 1000);
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Over", "0");
        telemetry.update();
        if (tfod != null) {
            tfod.shutdown();
        }
    }
    /*
    //this is my stuff

    private void waitForMotor() throws InterruptedException{
        while (robot.FrontLeftDrive.isBusy() || robot.FrontRightDrive.isBusy() || robot.RearLeftDrive.isBusy() || robot.RearRightDrive.isBusy()) {
            telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d",
                    robot.FrontLeftDrive.getCurrentPosition(),
                    robot.FrontRightDrive.getCurrentPosition(),
                    robot.RearLeftDrive.getCurrentPosition(),
                    robot.RearRightDrive.getCurrentPosition());
            telemetry.update();
        }
    }

     */
    /*
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

        private void initTfod () {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId",
                    "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.8f;
            tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
    private void objectDetection () {
        if (opModeIsActive()) {
            int count=0;
            while (opModeIsActive() && count<=20) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if(updatedRecognitions.size()==0){
                            telemetry.addData("TFOD", "No items detected");
                            telemetry.addData("Target Zone","A");
                        }else{
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                if(recognition.getLabel().equals("Single")){
                                    telemetry.addData("Target Zone","B");
                                }else if(recognition.getLabel().equals("Quad")){
                                    telemetry.addData("Target Zone","C");
                                }else{
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                }
                            }
                        }
                        telemetry.addData("count= ", "%d", count);
                        count++;
                        telemetry.update();

                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

    }
*/
}
