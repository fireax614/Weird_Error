#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/rotation.hpp"
#pragma once

extern pros::Motor intake;

extern pros::Motor arm;

extern pros::Rotation armrotation;

extern pros::Distance loader;

extern pros::Distance loader2;

extern pros::adi::DigitalOut mogo;

extern pros::adi::DigitalOut mogo2;

extern pros::adi::DigitalOut mogo3;

extern pros::adi::DigitalIn limit;

extern bool ogom;

extern bool walc;

extern bool loading;