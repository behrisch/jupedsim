# Copyright © 2012-2023 Forschungszentrum Jülich GmbH
# SPDX-License-Identifier: LGPL-3.0-or-later
import pytest

import jupedsim as jps
from jupedsim.native.journey import Transition


def test_can_query_agents_in_range():
    messages = []

    def log_msg_handler(msg):
        messages.append(msg)

    jps.set_info_callback(log_msg_handler)
    jps.set_warning_callback(log_msg_handler)
    jps.set_error_callback(log_msg_handler)

    geo_builder = jps.GeometryBuilder()
    geo_builder.add_accessible_area([(0, 0), (100, 0), (100, 100), (0, 100)])
    geometry = geo_builder.build()

    simulation = jps.Simulation(
        model=jps.VelocityModelParameters(), geometry=geometry, dt=0.01
    )
    exit = simulation.add_exit_stage(
        [(99, 45), (99, 55), (100, 55), (100, 45)]
    )

    journey = jps.JourneyDescription([exit])

    journey_id = simulation.add_journey(journey)

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journey_id
    agent_parameters.stage_id = exit
    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.15

    initial_agent_positions = [
        (10, 10),
        (20, 10),
        (30, 10),
        (40, 10),
        (50, 10),
    ]

    expected_agent_ids = set()

    for new_pos, id in zip(
        initial_agent_positions,
        range(10, 10 + len(initial_agent_positions)),
    ):
        print(id)
        agent_parameters.position = new_pos
        agent_parameters.id = id
        expected_agent_ids.add(simulation.add_agent(agent_parameters))

    actual_ids_in_range = list(
        simulation.agents_in_range(initial_agent_positions[2], 10)
    )

    actual_ids_in_polygon = list(
        simulation.agents_in_polygon([(39, 11), (39, 9), (51, 9), (51, 11)])
    )

    assert actual_ids_in_range == [11, 12, 13]
    assert actual_ids_in_polygon == [13, 14]


def test_can_run_simulation():
    messages = []

    def log_msg_handler(msg):
        messages.append(msg)

    # jps.set_debug_callback(log_msg_handler)
    jps.set_info_callback(log_msg_handler)
    jps.set_warning_callback(log_msg_handler)
    jps.set_error_callback(log_msg_handler)

    geo_builder = jps.GeometryBuilder()
    geo_builder.add_accessible_area([(0, 0), (10, 0), (10, 10), (0, 10)])
    geo_builder.add_accessible_area([(10, 4), (20, 4), (20, 6), (10, 6)])
    geometry = geo_builder.build()

    simulation = jps.Simulation(
        model=jps.VelocityModelParameters(), geometry=geometry, dt=0.01
    )
    exit_stage_id = simulation.add_exit_stage(
        [(18, 4), (20, 4), (20, 6), (18, 6)]
    )

    journey = jps.JourneyDescription([exit_stage_id])

    journey_id = simulation.add_journey(journey)

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journey_id
    agent_parameters.stage_id = exit_stage_id
    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.15

    initial_agent_positions = [(7, 7), (1, 3), (1, 5), (1, 7), (2, 7)]

    expected_agent_ids = set()

    for new_pos in initial_agent_positions:
        agent_parameters.position = new_pos
        expected_agent_ids.add(simulation.add_agent(agent_parameters))

    actual_agent_ids = {agent.id for agent in simulation.agents()}

    assert actual_agent_ids == expected_agent_ids

    agent_parameters.position = (6, 6)
    agent_id = simulation.add_agent(agent_parameters)
    assert simulation.remove_agent(agent_id)
    with pytest.raises(RuntimeError, match=r"Unknown agent id \d+"):
        assert simulation.remove_agent(agent_id)

    for actual, expected in zip(simulation.agents(), initial_agent_positions):
        assert actual.position == expected

    while simulation.agent_count() > 0:
        simulation.iterate()
        assert simulation.iteration_count() < 2000


def test_can_wait():
    messages = []

    def log_msg_handler(msg):
        messages.append(msg)

    jps.set_info_callback(log_msg_handler)
    jps.set_warning_callback(log_msg_handler)
    jps.set_error_callback(log_msg_handler)

    geo_builder = jps.GeometryBuilder()
    geo_builder.add_accessible_area([(0, 0), (100, 0), (100, 100), (0, 100)])
    geometry = geo_builder.build()

    simulation = jps.Simulation(
        model=jps.VelocityModelParameters(), geometry=geometry, dt=0.01
    )
    wp = simulation.add_waypoint_stage((50, 50), 1)
    waiting_set_id = simulation.add_waiting_set_stage(
        [
            (70, 50),
            (69, 50),
            (68, 50),
            (67, 50),
            (66, 50),
            (65, 50),
            (64, 50),
        ]
    )
    waiting_set = simulation.get_stage_proxy(waiting_set_id)
    exit = simulation.add_exit_stage(
        [(99, 40), (99, 60), (100, 60), (100, 40)]
    )
    journey = jps.JourneyDescription([wp, waiting_set_id, exit])
    journey.set_transition_for_stage(
        wp, Transition.create_fixed_transition(waiting_set_id)
    )
    journey.set_transition_for_stage(
        waiting_set_id, Transition.create_fixed_transition(exit)
    )

    journey_id = simulation.add_journey(journey)

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journey_id
    agent_parameters.stage_id = wp
    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.15

    initial_agent_positions = [
        (1, 1),
        (1, 2),
        (2, 1),
        (2, 2),
        (3, 1),
        (3, 2),
        (3, 3),
        (2, 3),
        (1, 3),
    ]

    expected_agent_ids = set()

    for new_pos in initial_agent_positions:
        agent_parameters.position = new_pos
        expected_agent_ids.add(simulation.add_agent(agent_parameters))

    actual_agent_ids = {agent.id for agent in simulation.agents()}

    assert actual_agent_ids == expected_agent_ids

    agent_parameters.position = (30, 30)
    agent_id = simulation.add_agent(agent_parameters)
    assert simulation.remove_agent(agent_id)
    with pytest.raises(RuntimeError, match=r"Unknown agent id \d+"):
        assert simulation.remove_agent(agent_id)

    for actual, expected in zip(simulation.agents(), initial_agent_positions):
        assert actual.position == expected

    while simulation.agent_count() > 0:
        simulation.iterate()
        if simulation.iteration_count() == 1000:
            waiting_set.state = jps.WaitingSetState.INACTIVE


def test_can_change_journey_while_waiting():
    messages = []

    def log_msg_handler(msg):
        messages.append(msg)

    jps.set_info_callback(log_msg_handler)
    jps.set_warning_callback(log_msg_handler)
    jps.set_error_callback(log_msg_handler)

    geo_builder = jps.GeometryBuilder()
    geo_builder.add_accessible_area([(0, 0), (100, 0), (100, 100), (0, 100)])
    geometry = geo_builder.build()

    simulation = jps.Simulation(
        model=jps.VelocityModelParameters(), geometry=geometry, dt=0.01
    )
    wp = simulation.add_waypoint_stage((50, 50), 1)
    stage_id = simulation.add_waiting_set_stage(
        [
            (60, 50),
            (59, 50),
            (58, 50),
        ]
    )
    stage = simulation.get_stage_proxy(stage_id)
    exit1 = simulation.add_exit_stage(
        [(99, 40), (99, 60), (100, 60), (100, 40)]
    )

    journey1 = jps.JourneyDescription([wp, stage_id, exit1])
    journey1.set_transition_for_stage(
        wp, Transition.create_fixed_transition(stage_id)
    )
    journey1.set_transition_for_stage(
        stage_id, Transition.create_fixed_transition(exit1)
    )
    journey2_stages = [
        simulation.add_waypoint_stage((60, 40), 1),
        simulation.add_waypoint_stage((40, 40), 1),
        simulation.add_waypoint_stage((40, 60), 1),
        simulation.add_waypoint_stage((60, 60), 1),
        simulation.add_exit_stage([(99, 50), (99, 70), (100, 70), (100, 50)]),
    ]
    journey2 = jps.JourneyDescription(journey2_stages)
    for src, dst in zip(journey2_stages[:-1], journey2_stages[1:]):
        journey2.set_transition_for_stage(
            src, Transition.create_fixed_transition(dst)
        )

    journeys = []
    journeys.append(simulation.add_journey(journey1))
    journeys.append(simulation.add_journey(journey2))

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journeys[0]
    agent_parameters.stage_id = stage_id
    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.15

    agent_parameters.position = (10, 50)
    simulation.add_agent(agent_parameters)

    agent_parameters.position = (8, 50)
    simulation.add_agent(agent_parameters)

    agent_parameters.position = (6, 50)
    simulation.add_agent(agent_parameters)

    redirect_once = True
    signal_once = True
    while simulation.agent_count() > 0:
        agents_at_head_of_waiting = list(
            simulation.agents_in_range((60, 50), 1)
        )
        if redirect_once and agents_at_head_of_waiting:
            simulation.switch_agent_journey(
                agent_id=agents_at_head_of_waiting[0],
                journey_id=journeys[1],
                stage_id=journey2_stages[0],
            )
            redirect_once = False

        if signal_once and simulation.agents_in_range((60, 60), 1):
            stage.state = jps.WaitingSetState.INACTIVE
            signal_once = False
        simulation.iterate()


def test_get_single_agent_from_simulation():
    messages = []

    def log_msg_handler(msg):
        messages.append(msg)

    # jps.set_debug_callback(log_msg_handler)
    jps.set_info_callback(log_msg_handler)
    jps.set_warning_callback(log_msg_handler)
    jps.set_error_callback(log_msg_handler)

    geo_builder = jps.GeometryBuilder()
    geo_builder.add_accessible_area([(0, 0), (10, 0), (10, 10), (0, 10)])
    geo_builder.add_accessible_area([(10, 4), (20, 4), (20, 6), (10, 6)])
    geometry = geo_builder.build()

    simulation = jps.Simulation(
        model=jps.VelocityModelParameters(), geometry=geometry, dt=0.01
    )
    exit_id = simulation.add_exit_stage([(18, 4), (20, 4), (20, 6), (18, 6)])

    journey = jps.JourneyDescription([exit_id])

    journey_id = simulation.add_journey(journey)

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journey_id
    agent_parameters.stage_id = exit_id
    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.15

    initial_agent_positions = [(7, 7), (1, 3), (1, 5), (1, 7), (2, 7)]

    agent_ids = set()

    for new_pos in initial_agent_positions:
        agent_parameters.position = new_pos
        agent_ids.add(simulation.add_agent(agent_parameters))

    for agent_id in agent_ids:
        assert simulation.agent(agent_id).id == agent_id


def test_get_agent_non_existing_agent_from_simulation():
    messages = []

    def log_msg_handler(msg):
        messages.append(msg)

    # jps.set_debug_callback(log_msg_handler)
    jps.set_info_callback(log_msg_handler)
    jps.set_warning_callback(log_msg_handler)
    jps.set_error_callback(log_msg_handler)

    geo_builder = jps.GeometryBuilder()
    geo_builder.add_accessible_area([(0, 0), (10, 0), (10, 10), (0, 10)])
    geo_builder.add_accessible_area([(10, 4), (20, 4), (20, 6), (10, 6)])
    geometry = geo_builder.build()

    simulation = jps.Simulation(
        model=jps.VelocityModelParameters(), geometry=geometry, dt=0.01
    )

    exit_id = simulation.add_exit_stage([(18, 4), (20, 4), (20, 6), (18, 6)])
    journey = jps.JourneyDescription([exit_id])

    journey_id = simulation.add_journey(journey)

    agent_parameters = jps.VelocityModelAgentParameters()
    agent_parameters.journey_id = journey_id
    agent_parameters.stage_id = exit_id
    agent_parameters.orientation = (1.0, 0.0)
    agent_parameters.position = (0.0, 0.0)
    agent_parameters.time_gap = 1
    agent_parameters.tau = 0.5
    agent_parameters.v0 = 1.2
    agent_parameters.radius = 0.15

    initial_agent_position = (7, 7)
    agent_parameters.position = initial_agent_position

    agent_id = simulation.add_agent(agent_parameters)

    assert simulation.agent(agent_id).id == agent_id

    with pytest.raises(
        RuntimeError, match=".*Trying to access unknown Agent.*"
    ):
        simulation.agent(1000)