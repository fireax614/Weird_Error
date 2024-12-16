#include "Config.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"

pros::Motor intake (20);

pros::Motor arm (-2);

pros::Rotation armrotation (4);

pros::Distance loader (12);

pros::Distance loader2 (13);

pros::adi::DigitalOut mogo ('D');

pros::adi::DigitalOut mogo2 ('F');

pros::adi::DigitalOut mogo3 ('C');


pros::adi::DigitalIn limit ('G');


bool walc = false;
bool ogom = false;
bool loading = false;