// Copyright © 2012-2024 Forschungszentrum Jülich GmbH
// SPDX-License-Identifier: LGPL-3.0-or-later
#pragma once

#include "RoutingEngine.hpp"

#include <vector>

class TacticalDecisionSystem
{
public:
    TacticalDecisionSystem() = default;
    ~TacticalDecisionSystem() = default;
    TacticalDecisionSystem(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem& operator=(const TacticalDecisionSystem& other) = delete;
    TacticalDecisionSystem(TacticalDecisionSystem&& other) = delete;
    TacticalDecisionSystem& operator=(TacticalDecisionSystem&& other) = delete;

    void Run(RoutingEngine& routingEngine, auto&& agents) const
    {
        for(auto& agent : agents) {
            const auto dest = agent.target;
            if(agent.iterationsUntilRouting == 0) {
                agent.destination = routingEngine.ComputeWaypoint(agent.pos, dest);
                const auto sqared_distance = DistanceSquared(agent.pos, agent.destination);
                if(sqared_distance >= 0.09) {
                    agent.iterationsUntilRouting = 20;
                }
            } else {
                --agent.iterationsUntilRouting;
            }
        }
    }
};
