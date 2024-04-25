#! /usr/bin/env python3

# Copyright © 2012-2023 Forschungszentrum Jülich GmbH
# SPDX-License-Identifier: LGPL-3.0-or-later
import logging
import pathlib
import sys
import time

import jupedsim as jps
import jupedsim.distributions
import shapely
from numpy.random import normal
from shapely import affinity


def log_debug(msg):
    logging.debug(msg)


def log_info(msg):
    logging.info(msg)


def log_warn(msg):
    logging.warning(msg)


def log_error(msg):
    logging.error(msg)


def main():
    logging.basicConfig(level=logging.DEBUG, format="%(levelname)s : %(message)s")
    jps.set_debug_callback(log_debug)
    jps.set_info_callback(log_info)
    jps.set_warning_callback(log_warn)
    jps.set_error_callback(log_error)

    geo = shapely.from_wkt(
        pathlib.Path("JuPedSim_EURO_Freiligrathplatz_v3.wkt").read_text()
    )
    # shift to center to make visualizer work
    cp = geo.centroid
    offset = (-cp.x, -cp.y)
    geo = affinity.translate(geo, offset[0], offset[1])

    simulation = jps.Simulation(
        model=jps.CollisionFreeSpeedModel(
            range_neighbor_repulsion=0.2,
            strength_geometry_repulsion=35,
            range_geometry_repulsion=0.019,
        ),
        dt=0.02,
        geometry=geo,
        trajectory_writer=jps.SqliteTrajectoryWriter(
            output_file=pathlib.Path("fanwalk_freiligrathplatz_5k_5min.sqlite"),
            every_nth_frame=10,
        ),
    )
    simulation.set_tracing(True)

    # exit area
    exit_area = [
        (x + offset[0], y + offset[1])
        for (x, y) in [
            (32343106.6, 5681829.77),
            (32343106.2, 5681830.69),
            (32343100.3, 5681828.30),
            (32343100.6, 5681827.37),
        ]
    ]

    exit_id = simulation.add_exit_stage(exit_area)

    WPs = [
        [x + offset[0], y + offset[1]]
        for [x, y] in [
            [32343212.0, 5681654.00],  # shared WP
        ]
    ]

    radius = [5.0]

    WP_ids = [
        simulation.add_waypoint_stage(wp_info[0], wp_info[1])
        for wp_info in zip(WPs, radius)
    ]

    journey = jps.JourneyDescription([exit_id] + WP_ids)

    journey.set_transition_for_stage(
        WP_ids[0],
        jps.Transition.create_fixed_transition(exit_id),
    )

    journey_id = simulation.add_journey(journey)

    # used as frequent spawning area of 1x8.5m -> 8 agents per spawn
    spawning_area = shapely.polygons(
        [
            (32343231.8, 5681604.78),
            (32343231.6, 5681605.77),
            (32343223.3, 5681604.31),
            (32343223.5, 5681603.32),
        ]
    )
    spawning_area = affinity.translate(spawning_area, offset[0], offset[1])

    stopping_area = shapely.polygons(
        [
            (32343219.0, 5681654.59),
            (32343209.4, 5681649.67),
            (32343220.0, 5681600.00),
            (32343238.7, 5681604.50),
        ]
    )
    stopping_area = affinity.translate(stopping_area, offset[0], offset[1])

    agent_parameters = jps.CollisionFreeSpeedModelAgentParameters()
    agent_parameters.radius = 0.15

    # calc positions for fanwalk
    num_agents = 8
    pos_fanwalk = jupedsim.distributions.distribute_by_number(
        polygon=spawning_area,
        number_of_agents=num_agents,
        distance_to_agents=0.3,
        distance_to_polygon=0.3,
        seed=234567,
    )
    # slow movement with small variance
    v_distribution = normal(0.83, 0.01, num_agents)

    start_time = time.perf_counter_ns()

    for pos, v0 in zip(pos_fanwalk, v_distribution):
        simulation.add_agent(
            jps.CollisionFreeSpeedModelAgentParameters(
                journey_id=journey_id,
                stage_id=WP_ids[0],
                position=pos,
                v0=v0,
                radius=0.15,
            )
        )

    num_required_for_fanwalk = 5000 - num_agents

    steps_per_second = 1 / 0.02

    time_stopped = -1

    offset_timer = 3100  # frist time agents reaching stopping line

    dict_id_orig_v0 = {}

    f = open("times.csv", "w+")

    while simulation.agent_count() > 0:
        try:
            loop_begin_time = time.perf_counter_ns()
            # spawn agents if spawning area is empty
            if (
                time_stopped == -1
                and num_required_for_fanwalk > 0
                and len(list(simulation.agents_in_polygon(spawning_area))) == 0
            ):
                num_required_for_fanwalk -= num_agents
                # spawning area is empty, spawn new
                for pos, v0 in zip(pos_fanwalk, v_distribution):
                    simulation.add_agent(
                        jps.CollisionFreeSpeedModelAgentParameters(
                            journey_id=journey_id,
                            stage_id=WP_ids[0],
                            position=pos,
                            v0=v0,
                            radius=0.15,
                        )
                    )

            if time_stopped > 0 and (simulation.iteration_count() - time_stopped) == (
                5 * 60 * steps_per_second
            ):
                # let agents move forward after 5 minutes of waiting
                for agent_id in list(dict_id_orig_v0.keys()):
                    simulation.agent(agent_id).model.v0 = dict_id_orig_v0.pop(agent_id)
                time_stopped = -1

            elif (
                simulation.iteration_count() > offset_timer
                and (simulation.iteration_count() - offset_timer)
                % (5 * 60 * steps_per_second)
                == 0
            ):
                # stop agents in stopping area every 5 minutes
                agents_to_be_stopped = list(simulation.agents_in_polygon(stopping_area))
                for agent_id in agents_to_be_stopped:
                    dict_id_orig_v0[agent_id] = simulation.agent(agent_id).model.v0
                    simulation.agent(agent_id).model.v0 = 0

                time_stopped = simulation.iteration_count()

            simulation.iterate()
            loop_end_time = time.perf_counter_ns()
            dt_s = (loop_end_time - start_time) / 1e9
            internal_iteration_duration_ms = (
                simulation.get_last_trace().iteration_duration / 1e3
            )
            iteration = simulation.iteration_count()
            loop_time_ms = (loop_end_time - loop_begin_time) / 1e6
            print(
                f"WC-Time: {dt_s:6.2f}s "
                f"S-Time: {iteration / steps_per_second:6.2f}s "
                f"I: {iteration:6d} "
                f"Agents: {simulation.agent_count():4d} "
                f"Time spend in C++: {internal_iteration_duration_ms:6.2f}ms ",
                f"Time spend in Python: {(loop_time_ms - internal_iteration_duration_ms):6.2f}ms ",
                f"Total Iteration Time: {loop_time_ms:6.2f}ms ",
                end="\r",
            )
            print(
                f"{simulation.agent_count()},{loop_time_ms},{internal_iteration_duration_ms},{(loop_time_ms - internal_iteration_duration_ms)}",
                file=f,
            )

        except KeyboardInterrupt:
            print("CTRL-C Received! Shutting down")
            sys.exit(1)
    print("\n")


if __name__ == "__main__":
    main()
