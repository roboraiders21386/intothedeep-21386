package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Autonomous(name = "Autonomous1", group = "00-Autonomous")
public class AutonomousIteration1 extends LinearOpMode {
    public static String TEAM_NAME = "Tx-Rx";
    public static int TEAM_NUMBER = 21386;
    public MecanumDrive drive;

    private Servo specimen;
    private DcMotor Lift;
    private int SPECIMEN_LIFT = 2000;
    private double OPEN_CLAW = 0.8;
    private double CLOSE_CLAW = 0.5;
    private double liftPow = 0.5;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }

    public enum ALLIANCE {
        BLUE,
        RED
    }

    public static START_POSITION startPosition;
    public static ALLIANCE alliance;
    //initializing poses
    Pose2d initpose;
    Pose2d specimenpose;
    Pose2d parkPose;
    Pose2d basketpose, specimenpickpose;

    @Override
    public void runOpMode() throws InterruptedException {
        specimen = hardwareMap.get(Servo.class, "specimen");
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotor.Direction.REVERSE);

       selectStartingPosition();
       //telemetry.addData("Selected Starting Position", startPosition);

        //startPosition = START_POSITION.RED_RIGHT;

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);
        }
        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonomousMode();
        }
    }

    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addData("Initializing Autonomous adopted for Team:",
                    TEAM_NAME, " ", TEAM_NUMBER);
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on PS5 Gamepad 1:", "");
            telemetry.addData("    Blue Left   ", "(Square)");
            telemetry.addData("    Blue Right ", "(Triangle)");
            telemetry.addData("    Red Left    ", "(Circle)");
            telemetry.addData("    Red Right  ", "(Cross)");
            if (gamepad1.square) {
                startPosition = START_POSITION.BLUE_LEFT;
                alliance = ALLIANCE.BLUE;
                break;
            }
            if (gamepad1.triangle) {
                startPosition = START_POSITION.BLUE_RIGHT;
                alliance = ALLIANCE.BLUE;
                break;
            }
            if (gamepad1.circle) {
                startPosition = START_POSITION.RED_LEFT;
                alliance = ALLIANCE.RED;
                break;
            }
            if (gamepad1.cross) {
                startPosition = START_POSITION.RED_RIGHT;
                alliance = ALLIANCE.RED;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();

    }





   /* public void SpecimenLift () {
        Lift.setTargetPosition(SPECIMEN_LIFT);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);
    }

    */
   public void runAutonomousMode() {
       //Initialize Pose2d as desired
       Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
       Pose2d specimenDropPose = new Pose2d(0, 0, 0);
       Pose2d midwayPose1 = new Pose2d(0, 0, 0);
       Pose2d intakeStack = new Pose2d(0, 0, 0);
       Pose2d midwayPose2 = new Pose2d(0, 0, 0);
       Pose2d parkPose = new Pose2d(0, 0, 0);
       double waitSecondsBeforeDrop = 0;
       MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

       initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose


       switch (startPosition) {
           case BLUE_LEFT:
               drive = new MecanumDrive(hardwareMap, initPose);
               specimenDropPose = new Pose2d(15, 24, Math.toRadians(0));
               midwayPose1 = new Pose2d(15, 18, Math.toRadians(0));
               parkPose = new Pose2d(48,0,Math.toRadians(0));
               break;

           case BLUE_RIGHT:
               drive = new MecanumDrive(hardwareMap, initPose);
               specimenDropPose = new Pose2d(-15, 24, Math.toRadians(0));
               midwayPose1 = new Pose2d(-15, 18, Math.toRadians(0));
               parkPose = new Pose2d(48,0,Math.toRadians(0));
               break;

           case RED_LEFT:
               drive = new MecanumDrive(hardwareMap, initPose);
               specimenDropPose = new Pose2d(15, 24, Math.toRadians(0));
               midwayPose1 = new Pose2d(-15, -18, Math.toRadians(0));
               parkPose = new Pose2d(48,0,Math.toRadians(0));
               break;

           case RED_RIGHT:
               drive = new MecanumDrive(hardwareMap, initPose);
               specimenDropPose = new Pose2d(28,10,0);
               midwayPose1 = new Pose2d(20, 5, Math.toRadians(0));
               midwayPose2 = new Pose2d(10, -18, Math.toRadians(45));
               parkPose = new Pose2d(0, -36, Math.toRadians(90));
               //specimenDropPose = new Pose2d(15, 24, Math.toRadians(0));
               //midwayPose1 = new Pose2d(15, 18, Math.toRadians(0));
               //parkPose = new Pose2d(48,0,Math.toRadians(0));
               break;


       }

       //Move to midwayPose1
       Actions.runBlocking(
               drive.actionBuilder(drive.pose)
                       .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                       .build());

       safeWaitSeconds((0.5));

       //TODO : Code to raise slide and drop specimen
       Lift.setTargetPosition(2000);
       Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Lift.setPower(liftPow);

       safeWaitSeconds(0.5);
       Lift.setPower(0);

       //Move to specimenDropPose
       Actions.runBlocking(
               drive.actionBuilder(drive.pose)
                       .strafeToLinearHeading(specimenDropPose.position, specimenDropPose.heading)
                       .build());

       safeWaitSeconds(0.5);

       Lift.setTargetPosition(1900);
       Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Lift.setPower(0.1);

       safeWaitSeconds(0.5);

       specimen.setPosition(OPEN_CLAW);


       //Move robot to midwayPose1
       Actions.runBlocking(
               drive.actionBuilder(drive.pose)
                       .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                       .build());


       safeWaitSeconds(0.5);

       //Bring the lift down
       Lift.setTargetPosition(0);
       Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       Lift.setPower(liftPow);

       safeWaitSeconds(0.5);

       //Move robot to park in Backstage
       Actions.runBlocking(
               drive.actionBuilder(drive.pose)
                       //TODO: after specimen go here
                       .strafeToLinearHeading(parkPose.position, parkPose.heading)
                       .build());


   }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

}




