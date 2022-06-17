/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * feel free to change the name or group of your class to better fit your robot
 */
@TeleOp(name = "DriverRelativeControl", group = "tutorial")
public class DriverRelativeControls extends LinearOpMode {

    /**
     * make sure to change these motors to your team's preference and configuration
     */
    private DcMotor left_rear;
    private DcMotor right_rear;
    private DcMotor left_front;
    private DcMotor right_front;

    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private navXPIDController yawPIDController;
    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private boolean calibration_complete = false;

    @Override
    public void runOpMode() {

        /**
         * you can change the variable names to make more sense
         */
        double driveTurn;
        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot = 0;
        double gamepadRadians = 0;
        double robotRadians = 0;
        double movementRadians = 0;
        double gamepadXControl = 0;
        double gamepadYControl = 0;
        double right_rear_power =0, right_front_power = 0, left_rear_power =0, left_front_power = 0;

        navx_device = AHRS.getInstance(hardwareMap.get(NavxMicroNavigationSensor.class, "navx"),
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
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

        /**
         * make sure to change this to how your robot is configured
         */
        left_front = hardwareMap.dcMotor.get("left_front");
        right_front = hardwareMap.dcMotor.get("right_front");
        right_rear = hardwareMap.dcMotor.get("right_rear");
        left_rear = hardwareMap.dcMotor.get("left_rear");

        //might need to change the motors being reversed
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.FORWARD);

        /**
         * make sure you've configured your imu properly and with the correct device name
         */

        composeTelemetry();

        waitForStart();


        while (opModeIsActive()) {
            telemetry.addData("Yaw", ("%.3f"), robotRadians);
            telemetry.addData("Left X", ("%.3f"), gamepad1.left_stick_x);
            telemetry.addData("Right X", ("%.3f"), gamepad1.right_stick_x);
            telemetry.addData("Right Y", ("%.3f"), gamepad1.right_stick_y);
            telemetry.addData("gamepadHypot", ("%.3f"), gamepadHypot);
            telemetry.addData("gamepadDegree", ("%.3f"), gamepadRadians);
            telemetry.addData("movementDegree", ("%.3f"), movementRadians);
            telemetry.addData("gamepadXControl", ("%.3f"), gamepadXControl);
            telemetry.addData("gamepadYControl", ("%.3f"), gamepadYControl);
            telemetry.addData("RF POWER", ("%.3f"), right_front_power);
            telemetry.addData("RR POWER", ("%.3f"), right_rear_power);
            telemetry.addData("LF POWER", ("%.3f"), left_front_power);
            telemetry.addData("LR POWER", ("%.3f"), left_rear_power);
            
            driveTurn = -gamepad1.left_stick_x;
            gamepadXCoordinate = gamepad1.right_stick_x; //this simply gives our x value relative to the driver
            gamepadYCoordinate = -gamepad1.right_stick_y; //this simply gives our y value relative to the driver
            gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
            //finds just how much power to give the robot based on how much x and y given by gamepad
            //range.clip helps us keep our power within positive 1
            // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)

            gamepadRadians = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);// - Math.PI/2; //the inverse tangent of opposite/adjacent gives us our gamepad degree

            robotRadians = (-navx_device.getYaw() * Math.PI/180); //gives us the angle our robot is at, in radians

            movementRadians = gamepadRadians - robotRadians; //adjust the angle we need to move at by finding needed
                                                             // movement degree based on gamepad and robot angles
            gamepadXControl = Math.cos(movementRadians) * gamepadHypot;
            //by finding the adjacent side, we can get our needed x value to power our motors
            gamepadYControl = Math.sin(movementRadians) * gamepadHypot;
            //by finding the opposite side, we can get our needed y value to power our motors

            //by multiplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will
            // not exceed 1 without any driveTurn
            //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
            //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
            right_front_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            right_rear_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
            left_front_power = (gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            left_rear_power = (gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
            right_front.setPower(right_front_power);
            left_front.setPower(left_front_power);
            right_rear.setPower(right_rear_power);
            left_rear.setPower(left_rear_power);

            telemetry.update();
        }

    }

    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {

            }
        });

    };
}