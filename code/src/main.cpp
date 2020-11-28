#include "main.h"
#include "odometry/odom4EncImu.hpp"
#include "odometry/odom4EncImuSimp.hpp"

#include "tracking/followTheCarrot.hpp"
#include "tracking/followThePast.hpp"
#include "tracking/purePursuit.hpp"
#include "tracking/purePursuitAdaptive1.hpp"
#include "tracking/ramsete.hpp"
#include "tracking/stanley.hpp"
#include "tracking/vectorPursuit.hpp"

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
		std::make_shared<kappa::TwoAxisChassis>(10.381, 38.754,
			std::make_shared<kappa::ArrayOutputClamp<double,2>>(-220, 220,
				std::make_shared<kappa::ArrayDistributor<double,2>>(std::initializer_list<std::shared_ptr<kappa::AbstractOutput<double>>>{
					std::make_shared<kappa::OutputChartLogger<double>>(chart1, targ1,
						std::make_shared<kappa::VPidSubController>(
							kappa::VPidSubController::Gains{60,50,50,620}, -12000, 12000,
							std::make_shared<kappa::InputChartLogger<double>>(chart1, read1,
								std::make_shared<kappa::InputDifferentiator<double>>(60000.0/900.0, std::make_unique<okapi::EmaFilter>(.65),
									std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(1))
								)
							),
							std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({ 1, 2})))
						)
					),
					std::make_shared<kappa::OutputChartLogger<double>>(chart2, targ2,
						std::make_shared<kappa::VPidSubController>(
							kappa::VPidSubController::Gains{60,50,50,620}, -12000, 12000,
							std::make_shared<kappa::InputChartLogger<double>>(chart2, read2,
								std::make_shared<kappa::InputDifferentiator<double>>(60000.0/900.0, std::make_unique<okapi::EmaFilter>(.65),
									std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::IntegratedEncoder>(8, true))
								)
							),
							std::make_shared<kappa::VoltageMotor>(std::make_shared<okapi::MotorGroup>(std::initializer_list<okapi::Motor>({-8,-9})))
						)
					)
				})
			)
		)
	;

	auto lEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(3,4), -0.06049);
	auto bEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(7,8), -0.06049);
	auto rEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(5,6),  0.06049);
	auto fEnc = std::make_shared<kappa::OkapiInput>(std::make_shared<okapi::ADIEncoder>(1,2),  0.06049);
	auto imu  = std::make_shared<kappa::ImuInput>(15);

	// kP value (hz), desired speed (cm/s), lookahead distance (cm)
	FollowTheCarrotTracker ftcTracker(10, 100, 40);

	// desired speed (cm/s), lookahead distance (cm)
	PurePursuitTracker ppTracker(100, 40);

	// desired speed (cm/s), lookahead distance (cm)
	PurePursuitAdaptive1Tracker pp1Tracker(100, 40);

	// zeta (unitless), b (1/cm^2), desired speed (cm/s), lookahead distance (cm)
	RamseteTracker ramseteTracker(0.5, 0.002, 100, 10);

	// k gain (hz), emulated vehicle length (cm), desired speed (cm/s)
	StanleyTracker stanleyTracker(10, 20, 75);

	// k gain (unitless), desired speed (cm/s), lookahead distance (cm)
	VectorPursuitTracker vpTracker(1, 75, 35);

	// emulated vehicle length (cm), lookahead distance (cm), desired speed (cm/s)
	FollowThePastTracker ftpTracker(20, 30, 100);


	imu->calibrate();
	pros::delay(2100);

	std::ofstream positionTelemFile(createNumberedFilename("/usd/telem/posData.", ".csv"));

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

  auto pathFile = std::make_shared<kappa::BinFileInput<double,4>>("/usd/paths/path1.4");

/*
	pros::Task odomTask2([&]{
		auto t = pros::millis();

		while(true){
			odom2->step();
			pros::Task::delay_until(&t, 10);
		}
	}, "OdomTask1");
*/

	pros::Task odomTask([&]{
		auto t = pros::millis();

		while(true){
			odom->step();

			pros::Task::delay_until(&t, 10);
		}
	}, "Odom Task");

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

	ramseteTracker.setTarget(pathFile);

	while(!ramseteTracker.isSettled()){
		auto pos = odom->get();

		chassis->set(ramseteTracker.step(pos));
		//chassis->set({50,-2});

		positionTelemFile << pros::millis();
		for(std::size_t i = 0; i < 6; i++){
			positionTelemFile << ", " << pos[i];
		}
		positionTelemFile << "\n";

		positionTelemFile.flush();

		std::cout << "\n";

		pros::Task::delay_until(&t, 50);
	}

	for(uint i = 0; i < 20; i++){
		chassis->set({0,0});
		pros::Task::delay_until(&t, 50);
	}

	odomTask.remove();

	positionTelemFile.close();

	auto end1 = lv_chart_add_series(chart1, LV_COLOR_YELLOW);
	auto end2 = lv_chart_add_series(chart2, LV_COLOR_YELLOW);

	while(true){
		lv_chart_set_next(chart1, end1, 0);
		lv_chart_set_next(chart2, end2, 0);
		pros::delay(100);
	}

}
