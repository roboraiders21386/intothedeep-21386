package org.firstinspires.ftc.teamcode.opmodes.TELEOP;
//libraries are a collection of resources

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name = "the best teleop clearly", group = "00-Teleop")
public class TeleOpimtrying extends LinearOpMode{
//what are these controls bro ToT
    //declaring a variable giving a name to a device
    private DcMotor lift;
    private CRServo Intake;
    private Servo Extension;
    private Servo Rotation;
    private Servo Claw;
    ColorSensor colorSensor;    // Hardware Device Object
    LED RLED, LLED;

    //declaring a variable - giving a name to a value
    private double dp = 0.5;
    private double intPow = 0.5;
    private double liftPow = 0.25;
    @Override
    public void runOpMode() throws InterruptedException {
        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort
        double LOW_BASKET_LIFT = 1000; //TODO Make sure this is accruate when we have game elements
        //helps you debug code by sending a message to the driver hub
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","21386");
        telemetry.update();

        //initializing your devices connecting your name to the configuration you inputted into the driver hub
        lift = hardwareMap.get(DcMotor.class, "lift");
        Intake = hardwareMap.get(CRServo.class, "intake");
        Extension = hardwareMap.get(Servo.class, "extend");
        Rotation = hardwareMap.get(Servo.class, "rotate");//is this the thing that rotates backwards
        Claw = hardwareMap.get(Servo.class, "claw"); //Specimen Claw
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        //setting a direction for your motor and initializing the encoder - a device that tracks how far your robot is going using ticks
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        //if you are using a mecanum drive this is what the program will use
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            //FINALLY getting into the driving part
            while (opModeIsActive()) {
                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","21386");
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                //forward backward
                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,//var that gives you more control over your driving
                                //strafing
                                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                        ),
                        //turning
                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
                )); //this is more concise drivetrain :D

                drive.updatePoseEstimate();

                //all the encoder stuff relaying the robots position to the driver hub
                telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

                //gives you a number stating where the robot is and where it's going
                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();
            }
            //referencing the tuning files if you are using a tank drive
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            //remember this is tank drive!!!!
            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                0.0 //no strafing :(
                        ),
                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
                ));

                drive.updatePoseEstimate();

                //the encoder for tank drive
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
        //ps5 code
        if (gamepad1.circle || gamepad1.b){
            lift.setTargetPosition(0); //set to start position
            telemetry.addData("Set to", "start value");
            telemetry.update();
        }
        if (gamepad1.triangle || gamepad1.y){
            lift.setTargetPosition(1000);//set to low position
            telemetry.addData("Set to", "low value");
            telemetry.update();
        }
        if (gamepad1.cross || gamepad1.a){
            lift.setTargetPosition(700);//set to rigging/specimen position
            telemetry.addData("Set to", "specimen value");
            telemetry.update();
        }
        if (gamepad1.square || gamepad1.x){
            lift.setTargetPosition(1500);//set to high position
            telemetry.addData("Set to", "high value");
            telemetry.update();
        }
        if (gamepad1.dpad_up){
            lift.setTargetPosition(lift.getCurrentPosition()+50);
            telemetry.addData("Incrementing by", "50");
            telemetry.update();
        }
        if (gamepad1.dpad_down){
            lift.setTargetPosition(lift.getCurrentPosition()-50);
            telemetry.addData("Decrementing by", "50");
            telemetry.update();
        }
        if (gamepad1.left_bumper){
            Intake.setPower(-1);
        }
        if (gamepad1.right_bumper){
            Intake.setPower(1);
        }
        if (gamepad1.left_trigger>0){
            Claw.setPosition(0);
        }
        if (gamepad1.right_trigger>0){
            Claw.setPosition(1);
        }
        //color sensor!!
        Gamepad.LedEffect reddetect = new Gamepad.LedEffect.Builder()
                .addStep(1, 0, 0, 250) // Show red for 250ms
                .build();
        Gamepad.LedEffect bluedetect = new Gamepad.LedEffect.Builder()
                .addStep(0, 0, 1, 250) // Show blue for 250ms
                .build();
        Gamepad.LedEffect neutral = new Gamepad.LedEffect.Builder()
                .addStep(1, 1, 0, 250) // Show yellow(????) for 250ms
                .build();
        if ((colorSensor.red() > 100)){
            gamepad1.runLedEffect(reddetect);
        }else if ((colorSensor.blue()>100)){
            gamepad1.runLedEffect(bluedetect);
        } else{
            gamepad1.runLedEffect(neutral);
        }
    }//I'm screwed T-T
}
