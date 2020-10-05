#include "main.h"
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

	auto chart1 = lv_chart_create(lv_scr_act(), NULL);
	lv_obj_set_pos(chart1, 0, 0);
	lv_obj_set_size(chart1, 480, 120);
	lv_chart_set_type(chart1, LV_CHART_TYPE_LINE);
	lv_chart_set_range(chart1, -230, 230);
	lv_chart_set_point_count(chart1, 240);

	auto chart2 = lv_chart_create(lv_scr_act(), NULL);
	lv_obj_set_pos(chart2, 0, 120);
	lv_obj_set_size(chart2, 480, 120);
	lv_chart_set_type(chart2, LV_CHART_TYPE_LINE);
	lv_chart_set_range(chart2, -230, 230);
	lv_chart_set_point_count(chart2, 240);

	auto targ1 = lv_chart_add_series(chart1, LV_COLOR_BLACK);
	auto read1 = lv_chart_add_series(chart1, LV_COLOR_RED);
	auto targ2 = lv_chart_add_series(chart2, LV_COLOR_BLACK);
	auto read2 = lv_chart_add_series(chart2, LV_COLOR_RED);

	okapi::Controller controller;

	auto chassis = //std::make_shared<kappa::TupleOutputLogger<double,double>>(6, " Chassis Commands ", " | ", "\n",
		std::make_shared<kappa::TwoAxisChassis>(4.125, 14.8125,
			std::make_shared<kappa::ArrayOutputClamp<double,2>>(-220, 220,
				//std::make_shared<kappa::ArrayOutputLogger<double,2>>(6, " Motor Commands ", " | ", "\n",
					std::make_shared<kappa::ArrayDistributor<double,2>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
						std::make_shared<kappa::OutputChartLogger<double>>(chart1, targ1,
							std::make_shared<kappa::VPidSubController>(
								kappa::VPidSubController::Gains{90,25,49,620}, -12000, 12000,
								std::make_shared<kappa::InputChartLogger<double>>(chart1, read1,
									std::make_shared<kappa::InputDifferentiator<double>>(6000.0/900.0, std::make_unique<okapi::EmaFilter>(.65),
										std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(1))
									)
								),
								std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({ 1, 2})))
							)
						),
						std::make_shared<kappa::OutputChartLogger<double>>(chart2, targ2,
							std::make_shared<kappa::VPidSubController>(
								kappa::VPidSubController::Gains{90,25,49,620}, -12000, 12000,
								std::make_shared<kappa::InputChartLogger<double>>(chart2, read2,
									std::make_shared<kappa::InputDifferentiator<double>>(6000.0/900.0, std::make_unique<okapi::EmaFilter>(.65),
										std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(8, true))
									)
								),
								std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({-8,-9})))
							)
						)
					})
				//)
			)
		)
	;//);

	auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -M_PI * 2.75 / 360.0);
	auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), -M_PI * 2.75 / 360.0);
	auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6),  M_PI * 2.75 / 360.0);
	auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2),  M_PI * 2.75 / 360.0);
	auto imu  = std::make_shared<kappa::ImuInput>(15);

	imu->calibrate();
	pros::delay(2100);

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
			std::cout << tcount1 << '\t' << tcount2 << '\t' << tcount3 << '\t';
			std::cout << p1[0] << ',' << p1[1] 	<< '\t' << p2[0] << ',' << p2[1] << '\t' << p3[0] << ',' << p3[1] << '\n';
			pros::delay(500);
		}
	}, "Log");

	auto t = pros::millis();

	while(true){

		chassis->set({40   * controller.getAnalog(okapi::ControllerAnalog::rightY),
									-5.5 * controller.getAnalog(okapi::ControllerAnalog::rightX)});

		pros::Task::delay_until(&t,10);
	}
}
