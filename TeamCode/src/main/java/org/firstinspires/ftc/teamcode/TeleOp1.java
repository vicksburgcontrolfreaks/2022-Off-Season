/* Driving with mech wheels
 *
 */

package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*********************************************/

@TeleOp(name="TeleOp1", group="Linear Opmode")

public class TeleOp1 extends LinearOpMode {

    private boolean calibration_complete = false;

   private DigitalChannel clawSwitch;
   private final ElapsedTime runtime = new ElapsedTime();
   private DcMotor arm_lifter = null;
   private DcMotor right_duck = null;
   private DcMotor left_duck = null;
   private Servo LeftG = null; 
   private Servo RightG = null;
   private double powerCmd;
   private float arm_gain;
   private double armPwrCmd;
   private double maxArmPwr;
   private double minArmPwr;

    @Override
    public void runOpMode() {
//        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        // Declare OpMode members.
        byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
        AHRS navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

//        /* Create a PID Controller which uses the Yaw Angle as input. */
//        navXPIDController yawPIDController = new navXPIDController(navx_device,
//                navXPIDController.navXTimestampedDataSource.YAW);
//
//        /* Configure the PID controller */
//        double TARGET_ANGLE_DEGREES = 0.0;
//        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
//        yawPIDController.setContinuous(true);
//        double MIN_MOTOR_OUTPUT_VALUE = -1.0;
//        double MAX_MOTOR_OUTPUT_VALUE = 1.0;
//        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
//        double TOLERANCE_DEGREES = 2.0;
//        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
//        double YAW_PID_P = 0.005;
//        double YAW_PID_I = 0.0;
//        double YAW_PID_D = 0.0;
//        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
//        yawPIDController.enable(true);

        DcMotor left_front = hardwareMap.get(DcMotor.class, "left_front");
        DcMotor right_front = hardwareMap.get(DcMotor.class, "right_front");
        DcMotor left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        DcMotor right_rear = hardwareMap.get(DcMotor.class, "right_rear");
        right_duck = hardwareMap.get(DcMotor.class, "right_duck");
        left_duck = hardwareMap.get(DcMotor.class, "left_duck");
        arm_lifter  = hardwareMap.get(DcMotor.class, "arm_lifter");
        LeftG = hardwareMap.get(Servo.class, "LeftG");
        RightG = hardwareMap.get(Servo.class, "RightG");
        clawSwitch = hardwareMap.get(DigitalChannel.class, "clawSwitch");
        clawSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel armSwitch = hardwareMap.get(DigitalChannel.class, "armSwitch");
        armSwitch.setMode(DigitalChannel.Mode.INPUT);
        DigitalChannel red = hardwareMap.get(DigitalChannel.class, "red");
        DigitalChannel green = hardwareMap.get(DigitalChannel.class, "green");

        
   //  waitForStart();

        int armTgt = 0;
        powerCmd = 0.0;
        arm_gain = (float)(0.002);
        armPwrCmd = 0.0;
        maxArmPwr = 0.9;
        minArmPwr = -0.25;
        double arm_stick_gain = 4.0;

        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
            telemetry.addData("navX-Micro", "Calibration is complete");
            telemetry.update();
        }
        navx_device.zeroYaw();
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.FORWARD);
        right_duck.setDirection(DcMotor.Direction.FORWARD);
        left_duck.setDirection(DcMotor.Direction.REVERSE);
        right_duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_lifter.setDirection(DcMotor.Direction.REVERSE);
        
        arm_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_lifter.setTargetPosition(armTgt);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // arm_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm_lifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        red.setMode(DigitalChannel.Mode.OUTPUT);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

  
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.

            double rcw = gamepad1.right_stick_x;
            double forward = gamepad1.left_stick_y * -1; /* Invert stick Y axis */
            double strafe = (0.75*gamepad1.left_stick_x);

            /* Adjust Joystick X/Y inputs by navX MXP yaw angle */

            float gyro_degrees = (navx_device.getCompassHeading());
            float gyro_radians = (float) (gyro_degrees * Math.PI/180);
            float temp = (float) (forward * Math.cos(gyro_radians) +
                                strafe * Math.sin(gyro_radians));
            strafe = -forward * Math.sin(gyro_radians) +
                    strafe * Math.cos(gyro_radians);
            forward = temp;
            telemetry.addData("Yaw", gyro_degrees);

            /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
            /* rotated by the gyro angle, and can be sent to drive system */

            telemetry.addData("Encoder Left", left_rear.getCurrentPosition());
            telemetry.addData("Encoder Right", right_rear.getCurrentPosition());
            telemetry.update();

            double left_front_power = Range.clip(forward + rcw + strafe, -1.0, 1.0);
            double right_front_power = Range.clip(forward - rcw - strafe, -1.0, 1.0);
            double left_rear_power = Range.clip(forward - rcw + strafe, -1.0, 1.0);
            double right_rear_power = Range.clip(forward + rcw - strafe, -1.0, 1.0);
            
          if (!clawSwitch.getState()){
              
                LeftG.setPosition(0.5);
                RightG.setPosition(0.5);
                red.setState(true);
                green.setState(false);
                hug();
                }
                
            if (gamepad2.a) {
              collectElement();
              red.setState(false);
              green.setState(true);
                // LeftG.setPosition(1);
                // RightG.setPosition(0);
                }
                
                if (gamepad2.b) {
            //   stop collector
                LeftG.setPosition(0.5);
                RightG.setPosition(0.5);
                }
            
            if (gamepad2.y) {
               depositElement();
               red.setState(false);
               green.setState(true);
            }
            
            if (gamepad2.right_bumper) {
               //Move arm up
            }
            
            if (gamepad2.left_bumper) {
               //move arm down
            }
            
            if (gamepad2.x) {
                spinDuckLeft();   
            } else if (gamepad2.y) {
                spinDuckRight();
            } else {
                stopDuck();                
            }
              
            if (gamepad2.dpad_up) {
               armTgt = armTgt + (int)(arm_stick_gain);
               if (armTgt > 900) {
                  armTgt = 900;       
               }
               moveArm(armTgt);    
            } else if (gamepad2.dpad_down) {
               armTgt = armTgt - (int)(arm_stick_gain);
               if (armTgt < 0) {
                  armTgt = 0;
               }
               if (!armSwitch.getState()){
                   armTgt = 0;
               }
               moveArm(armTgt); 
            } 
            
            
            
            armComplete();        
            if ((gamepad1.right_bumper)&&(gamepad1.left_bumper)) {
               arm_lifter.setTargetPosition(-500);
               arm_lifter.setPower(-0.1);
               sleep(1000);
               arm_lifter.setPower(0.0);
               arm_lifter.setTargetPosition(0);
               arm_lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               sleep(100);
               arm_lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            double driveGain;
            if ((gamepad1.right_trigger>0.5)){
          driveGain = 0.9;
        }
        else {
             driveGain = 0.75;
        }
            // Send calculated power to wheels
            // raw 0-1 was difficult to drive, 0.75 as a gain worked
            left_front.setPower(left_front_power * driveGain);
            right_front.setPower(right_front_power * driveGain);
            left_rear.setPower(left_rear_power * driveGain);
            right_rear.setPower(right_rear_power * driveGain);

        
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", left_front_power, right_front_power);
           // telemetry.addData("drive", left_front);
            //telemetry.addData("gamepad", drive);
            //telemetry.addData("gamepad", turn);
           // telemetry.addData("gamepad", strafe);
           //telemetry.addData("arm", arm_lifter.getCurrentPosition());
            telemetry.update();
        }
    }

   public void moveArm(int position) {
      int arm_error;

      arm_error = position - arm_lifter.getCurrentPosition();
      arm_lifter.setTargetPosition(position);

      armPwrCmd = arm_error * arm_gain; //proportional control
      if (armPwrCmd > maxArmPwr) {
         armPwrCmd = maxArmPwr;
      } else if (powerCmd < minArmPwr) {
         armPwrCmd = -minArmPwr;
      }
      arm_lifter.setPower(armPwrCmd);
        // arm_lifter.setPower(0.9);
   }
   
   private boolean armComplete() { 
      double armTolerance;

      armTolerance = arm_lifter.getTargetPosition()- arm_lifter.getCurrentPosition();
    
      armPwrCmd = armTolerance * (double)(arm_gain);
      if (armPwrCmd > maxArmPwr) {
         armPwrCmd = maxArmPwr;
      } else if (powerCmd < minArmPwr) {
         armPwrCmd = -minArmPwr;
      }
      arm_lifter.setPower(armPwrCmd);
    
      if (armTolerance < 0) {
         armTolerance = -armTolerance;
      }
  
      if (armTolerance < 2){
         return true;     
      } else {
         return false;
      }
   }    
   
   private void spinDuckRight() { 
      right_duck.setPower(0.9);
      left_duck.setPower(0.9);
   }

   private void spinDuckLeft() {
      right_duck.setPower(-0.9);
      left_duck.setPower(-0.9);    
   }

   private void stopDuck() {
      right_duck.setPower(0.0);
      left_duck.setPower(0.0);       
   }

   private void collectElement() {
   if(clawSwitch.getState()){
         LeftG.setPosition(1);
         RightG.setPosition(0);
      }
      else{
        LeftG.setPosition(0.5);
        RightG.setPosition(0.5);
   }
}
   private void depositElement() {
       LeftG.setPosition(0);
       RightG.setPosition(1);
      }
      double hugLoop = 0;
    private void hug(){
        
        if (hugLoop > 5) {
                LeftG.setPosition(1);
                RightG.setPosition(0);
                hugLoop = 0;
            }
        else {
                LeftG.setPosition(0.5);
                RightG.setPosition(0.5);
                hugLoop++;
            }
    }
        
   } 
