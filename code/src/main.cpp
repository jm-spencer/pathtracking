#include "main.h"
#include "odometry/odom4Enc.hpp"
#include "odometry/odom4EncImu.hpp"
#include "odometry/odom4EncImu2.hpp"
#include "odometry/odom4EncImuSimp.hpp"

void initialize() {
		std::cout.setf(std::ios::fixed);
}

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

	imu->calibrate();
	pros::delay(2100);

	auto odom1 = std::make_shared<Odom4Enc>(Odom4Enc::OdomVals{13.3125, 10.875},
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,4>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc
		})
	);

	auto odom2 = std::make_shared<Odom4EncImu>(Odom4EncImu::OdomVals{13.3125, 10.875},
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

	auto odom4 = std::make_shared<Odom4EncImuSimp>(Odom4EncImuSimp::OdomVals{13.3125, 10.875},
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc, imu
		})
	);
/*
	auto sensorLog = std::make_shared<kappa::ArrayInputLogger<double,5>>(6, ", ", ", ", ", ",
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
				lEnc, bEnc, rEnc, fEnc, imu
			})
	);
*/
/*
	auto odomLog = std::make_shared<kappa::ArrayInputLogger<double,6>>(
		odom2
	);
*/

	int tcount1 = 0;
	int tcount2 = 0;
	int tcount3 = 0;

	pros::Task odomTask1([&]{
		auto t = pros::millis();

		while(true){
			odom2->step();
			tcount1++;
			pros::Task::delay_until(&t, 2);
		}
	}, "OdomTask1");


	pros::Task odomTask2([&]{
		auto t = pros::millis();

		while(true){
			odom3->step();
			tcount2++;
			pros::Task::delay_until(&t, 2);
		}
	}, "OdomTask2");

	pros::Task odomTask3([&]{
		auto t = pros::millis();

		while(true){
			odom4->step();
			tcount3++;
			pros::Task::delay_until(&t, 2);
		}
	}, "OdomTask3");

	pros::Task logTask([&]{
		while(true){
			//sensorLog->get();
			//odomLog->get();

			const auto &p1 = odom2->get();
			const auto &p2 = odom3->get();
			const auto &p3 = odom4->get();
			std::cout << p1[0] << ',' << p1[1] << '\t' << p2[0] << ',' << p2[1] << '\t' << p3[0] << ',' << p3[1] << '\n';
			pros::delay(500);
		}
	}, "Log");

	while(true){
		chassis->set({12000 * controller.getAnalog(okapi::ControllerAnalog::leftY),
									12000 * controller.getAnalog(okapi::ControllerAnalog::rightY)});

		pros::delay(10);
	}
}
