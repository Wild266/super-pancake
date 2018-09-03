/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

//@Autonomous(name="JavaAutoR", group="Autonomous")

public class Team8535JavaAutonomous extends LinearOpMode {

    public static final int ALLIANCE_RED=1;
    public static final int ALLIANCE_BLUE=2;

    public static final int SIDE_LEFT=1;
    public static final int SIDE_RIGHT=2;

    public static final int BALL_RED=1;
    public static final int BALL_BLUE=2;

    public static double BALL_ARM_UP=0; //fill these in after testing on prod bot
    public static double BALL_ARM_DOWN=0.7; //fill these in after testing on prod bot //was 0.5 in last event

    public static boolean useEncoderMoves=true; //if true uses encoders for moves, if false used timed moves

    //VuMarks
    VuforiaLocalizer vuforia;

    private int alliance;
    private int side;

    private boolean prodbot = false;

    /*  A few notes to help when we move to encoders to guide moves (also see log file that we now generate)
    Ball Forward

    Before Ball 0 0 0 0
    After Ball -113 -146 -159 0
    After Move -1761 -2125 -2081 0
    After Move Close -1382 -2715 -2665 0
    After Move Back -1869 -2241 -2189 0
    After Move In -1193 -2980 -3135 0
    After Move Out -1516 -2631 -2756 0

    Ball Backward

    After Ball 183 120 127 0
    After Move -1290 -1566 -1569 0
    After Move Close -956 -2163 -2148
    After Move Back -1400 -1742 -1699
    After Move In -598 -2663 -2716
    After Move Out -945 -2351 -2416

    Ball Forward (Success)

    After Ball -94 -130 -125
    After Move -1732 -2028 -1953
    After Move Close -1388 -2557 -2457
    After Move Back -1829 -2126 -2005
    After Move In -1278 -2808 -2855
    After Move Out -1624 -2506 -2554

    blue red 74 blue 120
    red red 112 blue 47
    blue red 40 blue 43
    blue red 55 blue 91

     */

    //Speed Factor for Fast/Slow Mode
    private double speedFactor = 1.0; //default full speed

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double vacuumTime = 0.0;
    private double vacuumTime2 = 0.0; //added for second vacuum pump
    private double lastLoopTime=0.0;
    private double currentLoopTime=0.0;
    private double lastHold=0.0;

    //Drive Motors
    private DcMotor lf=null;
    private DcMotor rf=null;
    private DcMotor lb=null;
    private DcMotor rb=null;

    //Front Gripper
    private DcMotor gripperLiftMotor = null;
    private Servo gripperLeftServo = null;
    private Servo gripperRightServo = null;

    private double gripperLeftPosition = 0.0; //initial position of left gripper (tune this)
    private double gripperRightPosition = 1.0; //initial position of right gripper (tune this)
    private double gripperLeftClosingSpeed = 4.0; //range per second
    private double gripperRightClosingSpeed = 4.0; //range per second
    private double gripperLeftOpeningSpeed = 5.0; //range per second
    private double gripperRightOpeningSpeed = 5.0; //range per second

    //Ball Arm
    private Servo ballArmServo; //we'll need a servo to raise/lower the ball arm
    private double ballArmPosition = 0.0;//initial position of ball arm servo (tune this)
    private double ballArmSpeed = 0.5; //range per second

    //Block Tilt
    private Servo blockTiltServo = null;
    private double blockTiltPosition = 0.47; //was 0.95
    private double blockDumpPosition = 0.25; //was 0.35
    private double blockTiltSpeed = 0.7;

    //Vacuum
    private DcMotor vacuumMotor = null;
    private DcMotor vacuumMotor2 = null; //added
    private Servo vacuumReleaseServo = null;
    private Servo vacuumReleaseServo2 = null; //added

    private double vacuumReleasePosition = 1.0; //changed to 1.0 on 2/10 (to match teleop)
    private double vacuumReleasePosition2 = 1.0;

    private double vacuumReleaseSpeed = 1.0;

    private boolean vacuumRunning = false;
    private boolean vacuumRunning2 = false;

    //Base
    private ColorSensor bottomColorSensor = null;

    //Ball Color Sensor
    ColorSensor ballColorSensor; //we'll need a color sensor to detect ball color
    private int ballColor=0;
    private int minColorThresh=20;

    private List<String> encoderStates=new ArrayList<String>();

    //Gyro Sensor
    ModernRoboticsI2cGyro gyro; //a gyro would be really useful

    //private DcMotor vacuum=null;
    //private DcMotor vacuumRelease=null; //this will be eliminated or change to standard servo

    private static boolean SHOW_CAMERA=true; //whether to show the camera on the phone screen
    private static boolean JOYSTICK_SCALING=true; //whether to scale joystick values by cubing value (more precision for small movements)

    private static final int STATE_START=0; //at start of the run
    private static final int STATE_LOOKING=1; //actively looking for the vumark
    private static final int STATE_MOVE_ARM_DOWN=2; //moving arm down
    private static final int STATE_SENSE_BALL_COLOR=3; //sensing the ball's color
    private static final int STATE_ROTATE_BALL_OFF=4; //rotating the arm to knock the ball off
    private static final int STATE_MOVE_ARM_UP=5; // moving arm up
    private static final int STATE_MOVING=6; //moving to cryptobox
    private static final int STATE_MOVE_CLOSE=7;
    private static final int STATE_DUMPING_BLOCK = 8;
    private static final int STATE_MOVE_BACK = 9;
    private static final int STATE_MOVE_IN = 10;
    private static final int STATE_MOVE_OUT = 11;
    private static final int STATE_DONE=12;
    private static final int STATE_EXTRA_MOVE = 13;
    private static final int STATE_EXTRA_ROTATE = 14;

    private static String[] stateNames={
            "Starting", //0
            "Looking", //1
            "Moving Arm Down", //2
            "Sensing Ball Color", //3
            "Rotating to Knock Ball Off", //4
            "Moving Arm Up", //5
            "Moving to CryptoBox", //6
            "Moving Close to Cryptobox", //7
            "Dumping Block", //8
            "Move Back", //9
            "Move In", //10
            "Move Out", //11
            "Done", //12
            "Extra Move", //13
            "Extra Rotate" //14
    }; //state names for telemetry

    private int state; //the current state our robot is in
    RelicRecoveryVuMark vuMark; //currently detect vuMark

    //persistent telemetry items
    private String ballSeen="";
    private String posterSeen="";

    private int currentLeft=0;

    public Team8535JavaAutonomous(int alliance,int side) {
        this.alliance=alliance;
        this.side=side;
    }

    private static Map<RelicRecoveryVuMark,Integer> distMap=new HashMap<RelicRecoveryVuMark,Integer>();

    private DcMotor getMotor(String motorName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(DcMotor.class,motorName));
        } catch (Exception e) {
            return(null);
        }
    }

    private I2cDevice getDevice(String deviceName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(I2cDevice.class,deviceName));
        } catch (Exception e) {
            return(null);
        }
    }

    private ColorSensor getColorSensor(String sensorName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(ColorSensor.class,sensorName));
        } catch (Exception e) {
            return(null);
        }
    }

    private Servo getServo(String servoName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(Servo.class,servoName));
        } catch (Exception e) {
            return(null);
        }
    }

    private ModernRoboticsI2cGyro getGyro(String gyroName) { //these could be made generic using type notation
        try {
            return(hardwareMap.get(ModernRoboticsI2cGyro.class,gyroName));
        } catch (Exception e) {
            return(null);
        }
    }

    private boolean needsExtra() {
        return((alliance == ALLIANCE_BLUE && side == SIDE_RIGHT) || (alliance == ALLIANCE_RED && side == SIDE_LEFT));
    }

    /**
     * For debugging pause the state machine
     * @param time current elapsed time counter
     */
    private void holdup(ElapsedTime time) {
        telemetry.addData("Next State",stateNames[state]); //show the next planned state
        telemetry.update(); //update telemetry before the pause
        //while(!gamepad1.x && (time.time()-lastHold)>1.0) {}; //wait for X to be hit to proceed
        //lastHold=time.time(); //debounce X
    }

    private void mecanumMove(double lsx,double lsy,double rsx) {
        if (JOYSTICK_SCALING) {
            lsy=Math.pow(lsy,5.0);
            lsx=Math.pow(lsx,5.0);
            rsx=Math.pow(rsx,5.0);
        }
        mecanumMoveNoScale(lsx,lsy,rsx);
    }

    /**
     * Set motors for a mecanum move
     * @param lsx left stick x (left/right)
     * @param lsy left stick y (front/back)
     * @param rsx right stick x (rotation)
     */
    private void mecanumMoveNoScale(double lsx,double lsy,double rsx) {

        double r = Math.sqrt(lsy*lsy+lsx*lsx);
        double robotAngle = Math.atan2(-1*lsy,lsx) - Math.PI / 4;
        double rightX = rsx;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        lf.setPower(v1);
        rf.setPower(v2);
        lb.setPower(v3);
        rb.setPower(v4);
    }

    private void autonomousMove(double lsx,double lsy,double rsx,DcMotor motor,int target,int timeout,int moveTime) {
        if (useEncoderMoves) {
            mecanumEncoderMove(lsx,lsy,rsx,motor,target,timeout);
        } else {
            mecanumTimeMove(lsx,lsy,rsx,moveTime);
        }
    }

    private void autonomousRelativeMove(double lsx,double lsy,double rsx,DcMotor motor,int target,int timeout,int moveTime) {
        if (useEncoderMoves) {
            mecanumEncoderRelativeMove(lsx,lsy,rsx,motor,target,timeout);
        } else {
            mecanumTimeMove(lsx,lsy,rsx,moveTime);
        }
    }
    private void mecanumTimeMove(double lsx,double lsy,double rsx,int moveTime) {
        mecanumMoveNoScale(lsx,lsy,rsx);
        double startTime = runtime.milliseconds();
        while (opModeIsActive() && (runtime.milliseconds() - startTime) < moveTime) {
        }
        mecanumMoveNoScale(0,0,0);
    }

    private void mecanumEncoderMove(double lsx,double lsy,double rsx,DcMotor motor,int target,int timeout) {
        int start=motor.getCurrentPosition();
        boolean dir=(start<target); //true if we're moving up, false if moving down
        int current=start;
        mecanumMoveNoScale(lsx,lsy,rsx);
        double startTime = runtime.milliseconds();
        StringBuilder sb=new StringBuilder();
        sb.append("Dir="+dir+" Target="+target+" Current="+current+" opModeActive="+opModeIsActive()+" runtime="+runtime.milliseconds()+" startTime="+startTime+"\n");
        while (opModeIsActive() && ((dir && current<target) || (!dir && current>target)) && ((runtime.milliseconds()-startTime)<timeout)) {
            current=motor.getCurrentPosition();
        }
        mecanumMoveNoScale(0,0,0);
        sb.append("Dir="+dir+" Target="+target+" Current="+current+" opModeActive="+opModeIsActive()+" runtime="+runtime.milliseconds()+" startTime="+startTime+"\n");
        writeToTeamLog(sb.toString());
    }

    private void mecanumEncoderRelativeMove(double lsx,double lsy,double rsx,DcMotor motor,int target,int timeout) {
        int start=motor.getCurrentPosition();
        target=target+start; //relative to current position
        boolean dir=(start<target); //true if we're moving up, false if moving down
        int current=start;
        mecanumMoveNoScale(lsx,lsy,rsx);
        double startTime = runtime.milliseconds();
        StringBuilder sb=new StringBuilder();
        sb.append("Dir="+dir+" Target="+target+" Current="+current+" opModeActive="+opModeIsActive()+" runtime="+runtime.milliseconds()+" startTime="+startTime+"\n");
        while (opModeIsActive() && ((dir && current<target) || (!dir && current>target)) && ((runtime.milliseconds()-startTime)<timeout)) {
            current=motor.getCurrentPosition();
        }
        mecanumMoveNoScale(0,0,0);
        sb.append("Dir="+dir+" Target="+target+" Current="+current+" opModeActive="+opModeIsActive()+" runtime="+runtime.milliseconds()+" startTime="+startTime+"\n");
        writeToTeamLog(sb.toString());
    }

    private void gyroTurn(int degrees,ElapsedTime runtime, double maxTime) {
        /*
        int current=gyro.getHeading();
        int target=current+degrees;
        telemetry.addData("Gyro Turn","At %d Target %d",current,target);
        telemetry.update();
        */
        double startTime=runtime.time();
        if (degrees>0) {
            mecanumMoveNoScale(0,0,-0.3);
            while(true/*gyro.getHeading()<target*/) {
                //telemetry.addData("Turning","At %d",gyro.getHeading());
                //telemetry.update();
                if ((runtime.time()-startTime)>maxTime) break;
            }
        } else {
            mecanumMoveNoScale(0,0,0.3);
            while(true/*gyro.getHeading()>target*/) {
                //telemetry.addData("Turning","At %d",gyro.getHeading());
                //telemetry.update();
                if ((runtime.time()-startTime)>maxTime) break;
            }
        }
        mecanumMoveNoScale(0,0,0);
    }

    private void writeToTeamLog(String line) {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File(sdCard.getAbsolutePath() + "/team8535");
        dir.mkdirs();
        try {
            File file=new File(dir,"autonomous.log");
            PrintWriter out = new PrintWriter(new BufferedWriter(new FileWriter(file, true)));
            out.println(line);
            out.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void recordState(String label) {
        int lfpos = lf.getCurrentPosition(); //show positions to help with auto mode
        int rfpos = rf.getCurrentPosition();
        int lbpos = lb.getCurrentPosition();
        int rbpos = rb.getCurrentPosition();
        encoderStates.add(String.format(label+":lf=%d rf=%d lb=%d rb=%d", lfpos, rfpos, lbpos, rbpos));
        writeToTeamLog(String.format(label+":lf=%d rf=%d lb=%d rb=%d", lfpos, rfpos, lbpos, rbpos));
    }

    @Override
    public void runOpMode() {
        writeToTeamLog("Beginning Run --------------");
        writeToTeamLog((new Date()).toString());
        if (alliance==ALLIANCE_BLUE) {
            writeToTeamLog("Alliance=Blue");
        } else {
            writeToTeamLog("Alliance=Red");
        }
        if (side==SIDE_LEFT) {
            writeToTeamLog("Side=Left");
        } else {
            writeToTeamLog("Side=Right");
        }
        VuforiaLocalizer.Parameters parameters=null;
        if (SHOW_CAMERA) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = "Ab01nl7/////AAAAGeQnfaGoXUZ+i+4cRvO5jFNG9p0WO71bT/iVJiyCR32g6mazT1g6HiB2OmYcVTUVAWWGDIMKhNlGGjHAS/MCdmgK9VR4jbeUxBD0HT1xXebg7sD5+o2+4HSKheLgOnGdjVMwuUZK/3pnthEADVlvUZsDtrIxxYKBQEQSTf3uWP6vYFTax3kjPSIczUrmjUh6HhIhEm8NcrP4FgE/IjOr4xABtOU8QK4pdMDSxI5UatrszXVfs5jeUJ1gsciJBhwb95YN3e5Eqp/Mhr0K4iqdfGlPZLSYsm2757vfocnlHXaCM1jaU6jM42f8PR0/FLqZX9nIDSbtj+LAo9ufa6qi5/gnW3Ps3Vm1xpiGr7Tp10WN";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //use the back camera for VuForia
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        telemetry.addData("Status", "Initialized");
        if (getDevice("drivebot") != null) {
            prodbot = false;
            telemetry.addData("Bot", "DriveBot");
        } else {
            prodbot = true;
            telemetry.addData("Bot", "ProdBot");
        }
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        lf  = hardwareMap.get(DcMotor.class, "lf");
        rf  = hardwareMap.get(DcMotor.class, "rf");
        lb  = hardwareMap.get(DcMotor.class, "lb");
        rb  = hardwareMap.get(DcMotor.class, "rb");

        //Ball Arm
        ballArmServo = getServo("ball_arm");
        if (ballArmServo !=null) ballArmServo.setPosition(ballArmPosition);
        ballColorSensor = getColorSensor("ball_color");

        //Block Tilt
        blockTiltServo = getServo("block_tilt");
        if (blockTiltServo !=null) blockTiltServo.setPosition(blockTiltPosition);

        lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
        rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
        lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
        rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

        if (gripperLeftServo!=null) gripperLeftServo.setDirection(Servo.Direction.REVERSE); //changed to reverse to flip left gripper servo direction
        if (gripperRightServo!=null) gripperRightServo.setDirection(Servo.Direction.FORWARD);
        if (ballArmServo!=null) ballArmServo.setDirection(Servo.Direction.FORWARD);

        if (prodbot) {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor!=null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        } else {
            lf.setDirection(DcMotor.Direction.REVERSE); //was REVERSE
            rf.setDirection(DcMotor.Direction.FORWARD); //was REVERSE
            lb.setDirection(DcMotor.Direction.REVERSE); //was FORWARD
            rb.setDirection(DcMotor.Direction.FORWARD); //was FORWARD

            if (gripperLiftMotor!=null) gripperLiftMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoders
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //runs again
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        relicTrackables.activate();

        double time = 0; //move timer

        distMap.put(RelicRecoveryVuMark.LEFT,1350); //in milliseconds of movement
        distMap.put(RelicRecoveryVuMark.CENTER,1600);
        distMap.put(RelicRecoveryVuMark.RIGHT,1850);

        state=STATE_START; //in the start state

        while (opModeIsActive()) {

            telemetry.addData("State",stateNames[state]); //echo the state we're in
            telemetry.addData("Poster Seen",posterSeen);
            telemetry.addData("Ball Seen",ballSeen);

            //use a switch statement to take action based on the state we're in
            switch(state) {
                case STATE_START:
                    time = runtime.milliseconds();
                    state = STATE_LOOKING; //start looking for the VuMark
                    break;

                case STATE_LOOKING:
                    vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        telemetry.addData("VuMark", "%s visible", vuMark);
                        posterSeen=vuMark.toString();
                        writeToTeamLog("PosterSeen="+posterSeen);
                        state = STATE_MOVE_ARM_DOWN; //STATE_MOVE_ARM_DOWN; //STATE_START_LIFT //if picking up block
                        holdup(runtime);
                    } else if ((runtime.milliseconds() - time) > 5000) {
                        telemetry.addData("VuMark", "Assuming Center");
                        posterSeen="Unknown";
                        writeToTeamLog("PosterSeen="+posterSeen);
                        vuMark = RelicRecoveryVuMark.CENTER; //assume center after 5 seconds
                        state = STATE_MOVE_ARM_DOWN;
                        holdup(runtime);
                    }
                    break;

                case STATE_MOVE_ARM_DOWN:
                    ballArmServo.setPosition(BALL_ARM_DOWN);
                    try {
                        Thread.sleep(2000);
                    } catch (InterruptedException e) {
                    }
                    ;
                    state = STATE_SENSE_BALL_COLOR;
                    holdup(runtime);
                    break;

                case STATE_SENSE_BALL_COLOR:
                    telemetry.addData("BallColor", "R=%d G=%d B=%d A=%d", ballColorSensor.red(), ballColorSensor.green(), ballColorSensor.blue(), ballColorSensor.alpha());
                    if ((ballColorSensor.red() != 0 && ballColorSensor.red() != 255)
                            || ballColorSensor.blue() != 0 && ballColorSensor.blue() != 255) {
                        if (ballColorSensor.red() > (ballColorSensor.blue()+minColorThresh)) {
                            ballColor = BALL_RED;
                            telemetry.addData("Ball Color", "Red");
                            ballSeen=String.format("Red red=%d blue=%d",ballColorSensor.red(),ballColorSensor.blue());
                            telemetry.update();
                        } else if (ballColorSensor.blue() > (ballColorSensor.red()+minColorThresh)) {
                            ballColor = BALL_BLUE;
                            telemetry.addData("Ball Color", "Blue");
                            ballSeen=String.format("Blue red=%d blue=%d",ballColorSensor.red(),ballColorSensor.blue());
                            telemetry.update();
                        } else {
                            telemetry.addData("Ball Color", "Unknown");
                            ballSeen="Unknown";
                            telemetry.update();
                        }
                        writeToTeamLog("BallSeen="+ballSeen);
                        time = runtime.milliseconds();
                        recordState("Before Ball");
                        state = STATE_ROTATE_BALL_OFF;
                        holdup(runtime);
                    } else {
                        state = STATE_MOVE_ARM_UP;
                        holdup(runtime);
                    }
                    //blue alliance: (facing toward shelves) if blue then rotate 20, if red then rotate -20
                    //red alliance: (facing away from shelves) if blue then rotate 20, if red then rotate -20
                    //should read the color sensor and record color (using comparison of blue to red)
                    break;

                case STATE_ROTATE_BALL_OFF:
                    if (alliance == ALLIANCE_RED) { //forward to cryptobox
                        if (ballColor == BALL_RED) { //blue in front of us
                            autonomousMove(0,-0.5,0,lf,150,1000,300); //move until lf reads -150 or 500ms
                            //mecanumMoveNoScale(0, -0.5, 0); //move forward
                        } else { //blue in back of us
                            autonomousMove(0,0.5,0,lf,-150,1000,300); //move until lf read 150 or 500ms
                            //mecanumMoveNoScale(0, 0.5, 0); //move backward
                        }
                    } else if (alliance == ALLIANCE_BLUE) { //backward to cryptobox
                        if (ballColor == BALL_RED) { //red in back of us
                            autonomousMove(0,0.5,0,lf,-150,1000,300); //move until lf reads 150 or 500ms
                            //mecanumMoveNoScale(0, 0.5, 0); //move backward
                        } else { //red in front of us
                            autonomousMove(0,-0.5,0,lf,150,1000,300); //move until lf read -150 or 500ms
                            //mecanumMoveNoScale(0, -0.5, 0); //move forward
                        }
                    }
                    //telemetry.addData("Knocking Ball Off");
                    /*
                    if ((runtime.milliseconds() - time) > 300) { //was 300
                        mecanumMoveNoScale(0.0,0.0,0.0); */
                        recordState("After Ball");
                        state = STATE_MOVE_ARM_UP; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                    //}
                    break;


                case STATE_MOVE_ARM_UP:
                    ballArmServo.setPosition(BALL_ARM_UP);
                    try {Thread.sleep(1000);} catch(InterruptedException e) {};
                    time = runtime.milliseconds();
                    state = STATE_MOVING;
                    holdup(runtime);
                    break;

                case STATE_MOVING:
                    //should move by time, encoders, or inertial
                    if (alliance == ALLIANCE_RED) {
                        autonomousMove(0,-0.5,0,lf,1700,3000,1800); //move until lf reads -150 or 500ms
                        //mecanumMoveNoScale(0, -0.5, 0); //move forward
                    } else { //-1500 was near left shelf
                        autonomousMove(0,0.5,0,lf,-1700,3000,1800); //move until lf reads -150 or 500ms
                        //mecanumMoveNoScale(0, 0.5, 0); //move backward
                    }
                    /*int moveTime = 2400;
                    if (alliance == ALLIANCE_BLUE && side == SIDE_RIGHT) {
                        moveTime = 1800;
                    } else if (alliance == ALLIANCE_RED && side == SIDE_LEFT) {
                        moveTime = 1200;
                    }*/
                    int moveTime = 1800;
                    //later remember to compensate for ball knocking move
                    telemetry.addData("Moving", "%s units", distMap.get(vuMark));
                    /*
                    if ((runtime.milliseconds() - time) > moveTime) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                        recordState("After Move");
                    */
                        if (needsExtra()) {
                            state = STATE_EXTRA_MOVE;
                        }else {
                            state = STATE_MOVE_CLOSE; //after a second were at cryptobox?
                        }
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        //time = runtime.milliseconds();
                    //}

                    break;

                case STATE_EXTRA_MOVE:
                    autonomousRelativeMove(-1,0,0,lf,-500,2500,950); //need encoder data on this extra move
                    //mecanumMoveNoScale(-1, 0, 0); //strafe left
                    /*
                    if ((runtime.milliseconds() - time) > 950) { //was 850
                        mecanumMoveNoScale(0.0,0.0,0.0);
                    */
                        recordState("After Extra");
                        state = STATE_EXTRA_ROTATE; //
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    //}
                    break;

                case STATE_EXTRA_ROTATE:
                    if (alliance == ALLIANCE_BLUE) {
                        autonomousRelativeMove(0,0,1,lf,900,2000,550); //need encoder data on this extra move
                        //mecanumMoveNoScale(0, 0, 1); //rotate counter-clockwise
                    } else {
                        autonomousRelativeMove(0,0,-1,lf,-900,2000,550); //need encoder data on this extra move
                        //mecanumMoveNoScale(0, 0, -1); //rotate clockwise
                    }
                    /*
                    if ((runtime.milliseconds() - time) > 550) {
                        mecanumMoveNoScale(0.0,0.0,0.0);
                    */
                        recordState("After Extra Rotate");
                        state = STATE_MOVE_CLOSE; //
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    //}
                    break;

                case STATE_MOVE_CLOSE:
                    //Note: the encoder counts should be relative to current position (due to extra moves)
                    autonomousRelativeMove(1,0,0,lf,600,1700,500); //need encoder data on this extra move
                    //mecanumMoveNoScale(1, 0, 0); //move right
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    /*
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                    */
                        recordState("After Move Close");
                        state = STATE_DUMPING_BLOCK; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                    //}
                    break;

                case STATE_DUMPING_BLOCK:
                    if (blockTiltServo !=null) blockTiltServo.setPosition(blockDumpPosition);
                    state = STATE_MOVE_BACK; //after a second were at cryptobox?
                    holdup(runtime);
                    try { Thread.sleep(500); } catch (InterruptedException e) {};
                    time = runtime.milliseconds();
                    break;

                case STATE_MOVE_BACK:
                    autonomousRelativeMove(-1,0,0,lf,-600,1700,500); //need encoder data on this extra move
                    //mecanumMoveNoScale(-1, 0, 0); //move left
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    /*
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                    */
                        recordState("After Move Back");
                        state = STATE_MOVE_IN; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();
                    //}
                    break;

                case STATE_MOVE_IN:
                    autonomousRelativeMove(1,0,0,lf,600,1700,500); //need encoder data on this extra move
                    //mecanumMoveNoScale(1, 0, 0); //move right
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    /*
                    if ((runtime.milliseconds() - time) > 1000) { //was 500
                        mecanumMoveNoScale(0.0,0.0,0.0);
                    */
                        recordState("After Move In");
                        state = STATE_MOVE_OUT; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                        time = runtime.milliseconds();

                    //}
                    break;

                case STATE_MOVE_OUT:
                    autonomousRelativeMove(-0.75,0,0,lf,-600,1700,500); //need encoder data on this extra move
                    //mecanumMoveNoScale(-0.75, 0, 0); //move more slowly left
                    telemetry.addData("Moving Closer to Cryptobox", "");
                    /*
                    if ((runtime.milliseconds() - time) > 500) { //1600 goes to 2400
                        mecanumMoveNoScale(0.0,0.0,0.0);
                    */
                        recordState("After Move Out");
                        state = STATE_DONE; //after a second were at cryptobox?
                        holdup(runtime);
                        try { Thread.sleep(500); } catch (InterruptedException e) {};
                    //}
                    break;
            }

            // Show the elapsed game time
            for(String s:encoderStates) {
                int pos=s.indexOf(":");
                telemetry.addData(s.substring(0,pos),s.substring(pos+1));
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
