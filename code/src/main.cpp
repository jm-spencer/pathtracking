#include "main.h"
#include "odometry/odom4Enc.hpp"
#include "odometry/odom4EncImu.hpp"
#include "odometry/odom4EncImu2.hpp"
#include "odometry/odom4EncImuSimp.hpp"

#define ACTIVE_ODOM odom1

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {

	okapi::Controller controller;

	auto chassis = std::make_shared<kappa::ArrayDistributor<double,2>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
		std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({ 1, 2}))),
		std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({-8,-9})))
	});

	auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -M_PI * 2.75 / 360.0);
	auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), -M_PI * 2.75 / 360.0);
	auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6),  M_PI * 2.75 / 360.0);
	auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2),  M_PI * 2.75 / 360.0);
	auto imu  = std::make_shared<kappa::ImuInput>(15);

	auto odom1 = std::make_shared<Odom4Enc>(Odom4Enc::OdomVals{13.3125, 5.4375},
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc
		})
	);

	auto odom2 = std::make_shared<Odom4EncImu>(Odom4EncImu::OdomVals{13.3125, 5.4375},
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc, imu
		})
	);

	auto odom3 = std::make_shared<Odom4EncImu2>(
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc, imu
		})
	);

	auto odom4 = std::make_shared<Odom4EncImuSimp>(Odom4EncImuSimp::OdomVals{13.3125, 5.4375},
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc, imu
		})
	);

	auto odomLog = std::make_shared<kappa::ArrayInputLogger<double,6>>(
		ACTIVE_ODOM
	);

	auto t = pros::millis();

	pros::Task([&]{
		ACTIVE_ODOM->step();

		pros::Task::delay_until(&t, 2);
	}, "Odom");

	while(true){
		chassis->set({12000 * controller.getAnalog(okapi::ControllerAnalog::leftY),
									12000 * controller.getAnalog(okapi::ControllerAnalog::rightY)});

		odomLog->get();

		pros::delay(10);
	}
}
