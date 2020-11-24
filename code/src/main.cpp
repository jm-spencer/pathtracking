#include "main.h"
#include "odometry/odom4EncImu.hpp"
#include "odometry/odom4EncImuSimp.hpp"
#include "tracking/followTheCarrot.hpp"
#include "tracking/purePursuit.hpp"

std::string createNumberedFilename(std::string &&root, std::string &&extension){
	int i = 0;
	while(true){
		std::ifstream f(root + std::to_string(i) + extension);
		if(!f.good()){
			break;
		}
		i++;
	}
	return root + std::to_string(i) + extension;
}


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

	auto chassis =
		std::make_shared<kappa::TwoAxisChassis>(10.4775, 37.62375,
			std::make_shared<kappa::ArrayOutputClamp<double,2>>(-220, 220,
				std::make_shared<kappa::ArrayDistributor<double,2>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
					std::make_shared<kappa::OutputChartLogger<double>>(chart1, targ1,
					std::make_shared<kappa::OutputLogger<double>>("LTV ","\t",
						std::make_shared<kappa::VPidSubController>(
							kappa::VPidSubController::Gains{60,50,50,620}, -12000, 12000,
							std::make_shared<kappa::InputChartLogger<double>>(chart1, read1,
							std::make_shared<kappa::InputLogger<double>>("LRV ","\t",
								std::make_shared<kappa::InputDifferentiator<double>>(60000.0/900.0, std::make_unique<okapi::EmaFilter>(.65),
									std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(1))
								)
							)),
							std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({ 1, 2})))
						))
					),
					std::make_shared<kappa::OutputChartLogger<double>>(chart2, targ2,
					std::make_shared<kappa::OutputLogger<double>>("RTV ","\t",
						std::make_shared<kappa::VPidSubController>(
							kappa::VPidSubController::Gains{60,50,50,620}, -12000, 12000,
							std::make_shared<kappa::InputChartLogger<double>>(chart2, read2,
							std::make_shared<kappa::InputLogger<double>>("RRV ","\t",
								std::make_shared<kappa::InputDifferentiator<double>>(60000.0/900.0, std::make_unique<okapi::EmaFilter>(.65),
									std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(8, true))
								)
							)),
							std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({-8,-9})))
						))
					)
				})
			)
		)
	;

	auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -M_PI * 6.985 / 360.0);
	auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), -M_PI * 6.985 / 360.0);
	auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6),  M_PI * 6.985 / 360.0);
	auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2),  M_PI * 6.985 / 360.0);
	auto imu  = std::make_shared<kappa::ImuInput>(15);

	//kP value of 2, desired speed of 100 cm/s, lookahead distance of 15 cm
	FollowTheCarrotTracker ftcTracker(2, 100, 15);

	// desired speed of 50 cm/s, lookahead distance of 50 cm
	PurePursuitTracker ppTracker(50, 50);

	imu->calibrate();
	pros::delay(2100);

	//std::ofstream positionTelemFile(createNumberedFilename("/usd/telem/path1.", ".csv"));

/*
	auto odom2 = std::make_shared<Odom4EncImu>(//OdomVals{33.81375, 27.6225},
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_unique<okapi::PassthroughFilter>(),
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc, imu
		})
	);
*/
	auto odom = std::make_shared<Odom4EncImuSimp>(//OdomVals{33.81375, 27.6225},
		std::make_unique<okapi::EmaFilter>(.65),
		std::make_unique<okapi::EmaFilter>(.65),
		std::make_unique<okapi::EmaFilter>(.65),
		std::make_shared<kappa::ArrayConsolidator<double,5>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
			lEnc, bEnc, rEnc, fEnc, imu
		})
	);
/*
	auto sensorLog = std::make_shared<kappa::ArrayInputLogger<double,7>>(", ", ", ", ", ",
		std::make_shared<kappa::ArrayConsolidator<double,7>>(std::initializer_list<std::shared_ptr<kappa::AbstractInput<double>>>{
				lEnc, bEnc, rEnc, fEnc, imu,
				std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(1)),
				std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(8,true))
			})
	);
*/

  auto pathFile = std::make_shared<kappa::BinFileInput<double,3>>("/usd/paths/path1");

/*
	pros::Task odomTask2([&]{
		auto t = pros::millis();

		while(true){
			odom2->step();
			pros::Task::delay_until(&t, 10);
		}
	}, "OdomTask1");
*/
/*
	pros::Task odomTask([&]{
		auto t = pros::millis();

		while(true){
			auto pos = odom->step();

			positionTelemFile << pros::millis();
	    for(std::size_t i = 0; i < 6; i++){
	      positionTelemFile << ", " << pos[i];
	    }
	    positionTelemFile << "\n";

			//std::cout << pos[3] << ", " << pos[5] << "\n";

			pros::Task::delay_until(&t, 10);
		}
	}, "Odom Task");
*/
/*
	pros::Task logTask([&]{
		while(true){
			//sensorLog->get();
			//odomLog->get();

			const auto &p1 = odom->get();
			const auto &p2 = odom2->get();
			std::cout << p1[0] << ',' << p1[1] 	<< '\t' << p2[0] << ',' << p2[1] << '\n';
			pros::delay(500);
		}
	}, "Log");
*/
	auto t = pros::millis();

	ppTracker.setTarget(pathFile);

	while(!ppTracker.isSettled()){
		//chassis->set(ppTracker.step(odom->get()));
		//chassis->set({75,2});

		chassis->set({100  * controller.getAnalog(okapi::ControllerAnalog::rightY), // Maximum linear speed, 100 cm/s
									-5.5 * controller.getAnalog(okapi::ControllerAnalog::rightX)}); // Maximum angular velocity, 5.5 rad/s


		std::cout << "\n";

		pros::Task::delay_until(&t, 50);
	}
	chassis->set({0,0});


	//odomTask.remove();

	//positionTelemFile.close();
}
