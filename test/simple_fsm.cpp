// Simple FSM unit test
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <nimbro_fsm2/fsm.h>

#include <catch_ros/catch.hpp>

#include <ros/init.h>

class TestDriver
{
public:
	bool flag = false;
};

using FSM = nimbro_fsm2::FSM<TestDriver>;
using Transition = FSM::Transition;

class StateA;
class StateB;

class StateA : public FSM::State<StateA, FSM::Transitions<StateB>>
{
public:
	Transition execute() override
	{
		driver().flag = true;
		return transit<StateB>();
	}
};

class StateB : public FSM::State<StateB, FSM::Transitions<StateA, StateB>>
{
public:
	explicit StateB(bool transit = false)
	 : m_transit{transit}
	{}

	Transition execute() override
	{
		if(m_transit)
			return transit<StateA>();

		switch(m_count++)
		{
			case 0:
				return stay();
			case 1:
				return transit<StateB>(true);
		}

		throw std::logic_error("Should never arrive here");
	};
private:
	int m_count = 0;
	bool m_transit = false;
};

TEST_CASE("simple FSM")
{
	int argc = 1;
	std::vector<char*> argv{strdup("dummy")};
	ros::init(argc, argv.data(), "dummy");
	free(argv[0]);

	TestDriver driver;
	FSM fsm(driver);

	CHECK(fsm.currentStateName() == std::string());

	fsm.initialize<StateA>();
	{
		auto info = fsm.stateInfo();
		CHECK(info.states.size() == 2);

		auto infoA = std::find_if(info.states.begin(), info.states.end(), [](const auto& info){
			return info.name == "StateA";
		});
		REQUIRE(infoA != info.states.end());
		CHECK(infoA->successors.size() == 1);

		auto infoB = std::find_if(info.states.begin(), info.states.end(), [](const auto& info){
			return info.name == "StateB";
		});
		REQUIRE(infoB != info.states.end());
		CHECK(infoB->successors.size() == 2);
	}

	fsm.setState<StateA>();
	CHECK(fsm.currentStateName() == "StateA");
	CHECK(driver.flag == false);

	fsm.step();
	CHECK(driver.flag == true);
	CHECK(fsm.currentStateName() == "StateB");

	fsm.step();
	CHECK(fsm.currentStateName() == "StateB");

	fsm.step();
	CHECK(fsm.currentStateName() == "StateB");

	fsm.step();
	CHECK(fsm.currentStateName() == "StateA");
}
