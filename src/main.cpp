#include "main.h"
#include "Config.hpp"
#include "functions.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-9, 10, -19},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({6, -7, 17}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(14);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(16);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(8);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, 1.125);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0.8);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.4, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(16, // proportional gain (kP) 16 good
                                            0.001, // integral gain (kI)
                                            11, // derivative gain (kD) 11 good
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2.5, // proportional gain (kP)
                                             0.001, // integral gain (kI)
                                             28, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    armrotation.reset();
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}





/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

//!||||||||||||||||||||||||||||||||||
//!Stake Left
//!||||||||||||||||||||||||||||||||||
/*
//Start intake drive forward and grab ring.
intake.move_velocity(160);
chassis.moveToPoint(0, 16, 2000,
{ .maxSpeed = 60});
chassis.waitUntilDone();

//Spin intake until ring is detected then,
//outtake into loader.
while(loader.get_distance()>63 && loader2.get_distance()>63){
intake.move_velocity(140);}
while(loader2.get_distance()<63 || loader.get_distance()<63){
intake.move_velocity(-300);}
pros::delay(200);

//Back up and raise arm while also stopping intake
chassis.moveToPoint(0, 13, 2000,
{.forwards=false, .minSpeed = 20});
chassis.waitUntilDone();
while (armrotation.get_angle() < 9000){
    arm.move_velocity(600);}
arm.brake();
intake.move_velocity(0);

//Turn to face alliance stake and lower arm on it.
chassis.turnToHeading(58, 1000, 
{.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 55});
chassis.waitUntilDone();
while (armrotation.get_angle() > 6700){
    arm.move_velocity(-600);}
arm.brake();

//Back up and drive towards mobile goal and grab it.
chassis.moveToPose(-6, 8, 60, 500, 
{.forwards = false,.minSpeed = 40});
chassis.waitUntilDone();
chassis.moveToPose(-39, -3.5, 93, 6000, 
{.forwards = false, .minSpeed = 35});
chassis.waitUntil(30);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();

//Turn left and grab the bottom ring.
chassis.turnToHeading(225, 1000);
chassis.waitUntilDone();
intake.move_velocity(600);
chassis.moveToPose(-10, -38, 90, 
4000,{.minSpeed = 40});
chassis.waitUntilDone();

//Drive to the corner and grab the bottom ring from stack.
chassis.moveToPose(6, -47, 135, 6000, 
{.minSpeed = 20});
chassis.waitUntilDone();
pros::delay(300);

//Back up, turn to face tower and touch.
chassis.moveToPoint(-8.5,-27.6,2000,
{.forwards=false, .minSpeed = 50});
chassis.waitUntilDone();
chassis.moveToPose(-40, 0,310, 6000, 
{.minSpeed = 70});
arm.move_velocity(600);
chassis.waitUntil(14);
arm.brake();

*/
//?|||||||||||||||||||||||||||||||||||
//? Stake Right
//? ||||||||||||||||||||||||||||||||||

//Start intake drive forward and grab ring.
intake.move_velocity(160);
chassis.moveToPoint(0, 16, 2000,
{ .maxSpeed = 60});
chassis.waitUntilDone();

//Spin intake until ring is dected then,
//outtake into loader.
while(loader.get_distance()>63 && loader2.get_distance()>63){
intake.move_velocity(140);}
while(loader2.get_distance()<63 || loader.get_distance()<63){
intake.move_velocity(-285);}

pros::delay(200);
//Back up and raise arm while also stopping intake
chassis.moveToPoint(0, 13.75, 2000,
{.forwards=false, .minSpeed = 20});
chassis.waitUntilDone();
while (armrotation.get_angle() < 9000){
    arm.move_velocity(600);}
arm.brake();
intake.move_velocity(0);

//Turn to face alliance stake and lower arm on it.
chassis.turnToHeading(297, 1000, 
{.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 55});
chassis.waitUntilDone();
while (armrotation.get_angle() > 7100){
    arm.move_velocity(-600);}
arm.brake();

//Back up and drive towards mobile goal and grab it.
chassis.moveToPose(6, 8, 295, 500, 
{.forwards = false,.minSpeed = 40});
chassis.waitUntilDone();
chassis.moveToPose(37, -5, 272, 5000, 
{.forwards = false, .minSpeed = 35});
chassis.waitUntil(30);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();

//Turn left and grab the bottom ring.
chassis.turnToHeading(135, 1000, {.direction = 
AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
chassis.waitUntilDone();
intake.move_velocity(600);
chassis.moveToPose(10, -36, 270, 4000, {.minSpeed =50});
chassis.waitUntilDone();

//Drive to the corner and grab the bottom ring from stack.
chassis.moveToPose(-6, -45, 220, 2500);
chassis.waitUntilDone();
pros::delay(300);

//Back up, turn to face tower and touch.
chassis.moveToPoint(8.5,-27.6,2000,
{.forwards = false, .maxSpeed = 65});
chassis.waitUntilDone();
chassis.moveToPose(45, 0,45, 6000, 
{.minSpeed = 70});
arm.move_velocity(600);
chassis.waitUntil(8);
arm.brake();

//!||||||||||||||||||||||||||||||||||||||
//!GOAL RIGHT
//!||||||||||||||||||||||||||||||||||||||
/*
//Back up and grab the Mobile goal.
intake.move_velocity(600);
chassis.moveToPoint(0,-25,4000,{.forwards = false, 
.maxSpeed = 50 });
chassis.waitUntil(19);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();
intake.move_velocity(0);
pros::delay(200);

//Raise arm and move towards ring 
//stack and grab bottom ring.
while (armrotation.get_angle() < 11000){
    arm.move_velocity(600);}
arm.brake();
intake.move_velocity(600);
chassis.turnToHeading(225, 1000, {.direction = 
AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60});
chassis.waitUntilDone();
pros::delay(100);

//Go to corner and grab bottom ring and score on mogo.
chassis.moveToPose(-30, -10, 0, 4000, 
{.minSpeed =40});
chassis.waitUntilDone();
chassis.moveToPose(-40, 13,311, 
5000, {.minSpeed= 40});
chassis.waitUntilDone();
pros::delay(500);

//Back away from the corner with ring.
chassis.moveToPoint(-22, -6,1000, 
{.forwards = false,.minSpeed= 50});
chassis.waitUntilDone();

//drive towards the alliance stake and turn
chassis.moveToPose(16, 8,43.5, 4000);
chassis.waitUntilDone();
intake.move_velocity(0);

//lower arm to score ring on stake.
while (armrotation.get_angle() > 7200){
    arm.move_velocity(-600);}
arm.brake();
pros::delay(500);

//Back away from alliance stake and goal towards tower.
chassis.moveToPoint(6, 2,3000, 
{.forwards = false});
chassis.waitUntilDone();
arm.move_velocity(600);
chassis.moveToPose(16, -40,115, 
6000,{.minSpeed=50});
chassis.waitUntil(5);
arm.brake();
*/
//?|||||||||||||||||||||||||||||||||||||||||||||||||
//? GOAL LEFT
//?|||||||||||||||||||||||||||||||||||||||||||||||||
/*
//Back up and grab the Mobile goal.
intake.move_velocity(600);
chassis.moveToPoint(0,-25,4000,{.forwards 
= false, .maxSpeed = 50 });
chassis.waitUntil(19);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();
intake.move_velocity(0);
pros::delay(200);

//Raise arm and move towards ring 
//stack and grab bottom ring.
while (armrotation.get_angle() < 11000){
    arm.move_velocity(600);}
arm.brake();
intake.move_velocity(600);
chassis.turnToHeading(135, 1000, 
{.direction = AngularDirection::CW_CLOCKWISE, .maxSpeed = 60});
chassis.waitUntilDone();
pros::delay(100);

//Go to corner and grab bottom ring and score on mogo.
chassis.moveToPose(30, -10, 0, 4000, 
{.minSpeed =40});
chassis.waitUntilDone();
chassis.moveToPose(40, 13,49, 
5000, {.minSpeed= 40});
chassis.waitUntilDone();
pros::delay(500);

//Back away from the corner with ring.
chassis.moveToPoint(22, -6,1000, {.forwards 
= false,.minSpeed= 50});
chassis.waitUntilDone();

//drive towards the alliance stake and turn
chassis.moveToPose(-14, 7,317.5, 4000);
chassis.waitUntilDone();

//lower arm to score ring on stake.
while (armrotation.get_angle() > 7200){
    arm.move_velocity(-600);}
arm.brake();
pros::delay(500);

//Back away from alliance stake and goal towards tower.
chassis.moveToPoint(-6, 2,3000, 
{.forwards = false});
chassis.waitUntilDone();
intake.move_velocity(0);
arm.move_velocity(600);
chassis.moveToPose(-16, -40,245, 
6000,{.minSpeed=50});
chassis.waitUntil(5);
arm.brake();
*/
/*
//TODO SKILLS
//Raise the arm, move forward and score preload.
intake.move_velocity(600);
pros::delay(200);
while (armrotation.get_angle() < 11000){
    arm.move_velocity(600);}
arm.brake();
intake.move_velocity(0);
chassis.moveToPoint(0, 4, 2000,{.maxSpeed=80});
chassis.waitUntilDone();
pros::delay(200);
while (armrotation.get_angle() > 7000){
    arm.move_velocity(-600);}
arm.brake();
pros::delay(200);
chassis.moveToPoint(0, -5, 2000,
{.forwards= false, .maxSpeed=80});
chassis.waitUntilDone();

//Back up and grab the mobile goal.
chassis.moveToPose(23,-6,236,2800,
{.forwards=false, .maxSpeed=95});
chassis.waitUntil(27);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();
pros::delay(200);

//Turn left and grab two rings and raise arm
chassis.moveToPose(31,-27,90,4000,
{.maxSpeed=90,.minSpeed=40});
chassis.waitUntil(11);
intake.move_velocity(600);
chassis.moveToPoint(48,-27,3000,
{.maxSpeed=65});
chassis.waitUntilDone();
while (armrotation.get_angle() < 9000){
    arm.move_velocity(600);}
arm.brake();

//Grab one ring and back up
chassis.moveToPoint(58,-5,3000,
{.maxSpeed=65});
chassis.waitUntilDone();
pros::delay(300);
chassis.moveToPoint(58,-25,1500,
{.forwards=false,.maxSpeed=60});
chassis.waitUntilDone();

//Grab two more rings and back up.
chassis.moveToPose(49,7,0,4000,
{.maxSpeed=65});
chassis.waitUntilDone();
chassis.moveToPose(49,0,0,2000,
{.forwards=false,.maxSpeed=70});
chassis.waitUntilDone();

//Place Mogo in corner and drop it.
chassis.moveToPose(59,3,225,2100,
{.forwards=false});
chassis.waitUntilDone();
intake.move_velocity(0);
mogo2.set_value(0);
mogo3.set_value(0);
mogo.set_value(0);

//Drive to Mogo #2 and grab it.
chassis.moveToPose(-19,-5,90,6000,
{.forwards=false,.maxSpeed=65});
chassis.waitUntil(77);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();
pros::delay(200);

//Turn right and grab two rings.
chassis.turnToHeading(180, 500);
chassis.waitUntilDone();
chassis.moveToPose(-23,-30,235,2500);
chassis.waitUntil(10);
intake.move_velocity(600);
chassis.moveToPose(-50,-30,270,2500,
{.minSpeed=50});
chassis.waitUntilDone();

//Turn left, then drive grab ring and backup.
chassis.moveToPose(-57,-2,0,4000);
chassis.waitUntilDone();
chassis.moveToPoint(-57,-25,3000,{.forwards=false,
.maxSpeed=60});
chassis.waitUntilDone();
pros::delay(200);

//Grab two more rings and back up
chassis.moveToPose(-46,11.5,.5,4000,
{.minSpeed=50});
chassis.waitUntilDone();
chassis.moveToPoint(-46,0,2000,{.forwards=false});
chassis.waitUntilDone();

//Place goal in corner and drop.
chassis.moveToPose(-60,6,137,1800,{
.forwards=false});
chassis.waitUntilDone();
intake.move_velocity(0);
mogo2.set_value(0);
mogo3.set_value(0);
mogo.set_value(0);

//Drive to Mogo #3 and turn away.
intake.move_velocity(600);
chassis.moveToPose(-30,-80,165,5000);
chassis.waitUntilDone();
chassis.turnToHeading(345, 500);
chassis.waitUntilDone();
intake.move_velocity(0);

//Back up into goal and grab it.
chassis.moveToPose(-26,-102,330,3000,
{.forwards=false,.maxSpeed=65});
chassis.waitUntil(30);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();
pros::delay(200);

//Turn and maneuver the goal into the corner.
chassis.turnToHeading(77, 500);
chassis.waitUntilDone();
chassis.moveToPose(-63.5,-112,65,2200,
{.forwards=false,.minSpeed=30});
chassis.waitUntilDone();
mogo2.set_value(0);
mogo3.set_value(0);
mogo.set_value(0);

//Drive towards Mogo #4.
chassis.moveToPose(-20,-105,90,3000);
chassis.waitUntilDone();

//Turn and grab the goal.
chassis.turnToHeading(225,  500);
chassis.waitUntilDone();
chassis.moveToPose(6,-95,225,3000,
{.forwards=false,.maxSpeed=65});
chassis.waitUntil(25);
mogo2.set_value(1);
mogo3.set_value(1);
pros::c::delay(150);
mogo.set_value(1);
chassis.waitUntilDone();
pros::delay(200);

//Turn and drive towards two rings
intake.move_velocity(600);
chassis.turnToHeading(50, 500);
chassis.waitUntilDone();
chassis.moveToPose(22,-73,47,3000);
chassis.waitUntilDone();
chassis.moveToPose(53,-73,90,3000);
chassis.waitUntilDone();

//Drive towards the last the corner and place goal.
chassis.moveToPose(57, -70, 0,1000);
chassis.waitUntilDone();
intake.move_velocity(0);
chassis.moveToPose(65,-116,330,2600,
{.forwards=false,.minSpeed=45});
chassis.waitUntilDone();
mogo2.set_value(0);
mogo3.set_value(0);
mogo.set_value(0);

//Move away from corner.
chassis.moveToPose(55, -70, 0,3000);
chassis.waitUntilDone();
*/

}
/**
 * Runs in driver control
 */
void opcontrol() {
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // controller
    // loop to continuously update motors
    /*claw.set_value(1);
pros::c::delay(500);
arm.move_velocity(600);
pros::c::delay(200);
arm.brake();
chassis.moveToPoint(0, -3, 1000,
{.forwards = false});
pros::c::delay(200);
while (armrotation.get_angle() < 11500 || 
armrotation.get_angle() > 30000  ){
    arm.move_velocity(600);}
arm.brake();

//Move towards red stake and score ring.
pros::c::delay(250);
chassis.moveToPoint(0, 2.7, 1000,
{.maxSpeed = 15});
chassis.waitUntilDone();
pros::c::delay(250);
while (armrotation.get_angle() > 8000 ){
    arm.move_velocity(-600);}
arm.brake();
chassis.waitUntilDone();*/

    while (true) {
        
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);


    //!intake
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            intake.move_velocity(-100);
        }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
    {
        intake.move_velocity(600);
    }

    else {
        intake.move_velocity(0);
    }

    //!lOADER
       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)&& controller.get_digital
    (pros::E_CONTROLLER_DIGITAL_R2)&& limit.get_value()==1)
        {
                if(loader.get_distance()>63 && loader2.get_distance()>63)
                {
                    intake.move_velocity(130);
                }

                else if(loader.get_distance() <63 || loader2.get_distance() <63)
                {
                    intake.move_velocity(-140);   
                }
        }

    //!Arm
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)&& limit.get_value()==0)
        {
            arm.move_velocity(-600);
            
        }

    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)&& armrotation.get_angle() < 15300 || 
    controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && armrotation.get_angle() > 30000)
    {
        arm.move_velocity(600);
    }
    else {
        arm.brake();
    }


    //!Mogo Mech
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        if (ogom==true){
            mogo.set_value(0);
            mogo2.set_value(0);
            mogo3.set_value(0);

            ogom=false;
        }
        else if (ogom==false){
            mogo2.set_value(1);
            mogo3.set_value(1);
            pros::c::delay(150);
            mogo.set_value(1);
            ogom=true;
        }

    }
    

    }

    
}
