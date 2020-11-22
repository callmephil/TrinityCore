/*
 * This file is part of the TrinityCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "WaypointMovementGenerator.h"
#include "Creature.h"
#include "CreatureAI.h"
#include "Log.h"
#include "Map.h"
#include "MoveSpline.h"
#include "MoveSplineInit.h"
#include "ObjectMgr.h"
#include "Transport.h"
#include "WaypointManager.h"

WaypointMovementGenerator<Creature>::WaypointMovementGenerator(uint32 pathId, bool repeating) : _nextMoveTimer(0), _movementInformTimer(0), _lastSplineId(0), _pathId(pathId),
_waypointReached(true), _recalculateSpeed(false), _repeating(repeating), _loadedFromDB(true), _stalled(false), _done(false)
{
}

WaypointMovementGenerator<Creature>::WaypointMovementGenerator(WaypointPath& path, bool repeating) : _nextMoveTimer(0), _movementInformTimer(0), _lastSplineId(0), _pathId(0),
_waypointReached(true), _recalculateSpeed(false), _repeating(repeating), _loadedFromDB(false), _stalled(false), _done(false)
{
    _path = &path;
}

void WaypointMovementGenerator<Creature>::DoInitialize(Creature* creature)
{
    _done = false;

    if (_loadedFromDB)
    {
        if (!_pathId)
            _pathId = creature->GetWaypointPath();

        _path = sWaypointMgr->GetPath(_pathId);
    }

    if (!_path)
    {
        // No path id found for entry
        TC_LOG_ERROR("sql.sql", "WaypointMovementGenerator::DoInitialize: creature %s (Entry: %u GUID: %u DB GUID: %u) doesn't have waypoint path id: %u", creature->GetName().c_str(), creature->GetEntry(), creature->GetGUID().GetCounter(), creature->GetSpawnId(), _pathId);
        return;
    }

    _nextMoveTimer.Reset(1000);

    // inform AI
    if (creature->IsAIEnabled)
        creature->AI()->WaypointPathStarted(_path->Id);
}

void WaypointMovementGenerator<Creature>::DoFinalize(Creature* creature)
{
    creature->ClearUnitState(UNIT_STATE_ROAMING | UNIT_STATE_ROAMING_MOVE);
    creature->SetWalk(false);
}

void WaypointMovementGenerator<Creature>::DoReset(Creature* creature)
{
    if (!_done && _nextMoveTimer.Passed() && CanMove(creature))
        StartMove(creature);
    else if (_done)
    {
        // mimic IdleMovementGenerator
        if (!creature->IsStopped())
            creature->StopMoving();
    }
}

void WaypointMovementGenerator<Creature>::OnArrived(Creature* creature)
{
    if (!_path || _path->Nodes.empty())
        return;

    WaypointNode const& waypoint = _path->Nodes.at(_currentNode);
    if (waypoint.Delay > 0)
        creature->ClearUnitState(UNIT_STATE_ROAMING_MOVE);

    if (waypoint.EventId && urand(0, 99) < waypoint.EventChance)
    {
        TC_LOG_DEBUG("maps.script", "Creature movement start script %u at point %u for %s.", waypoint.EventId, _currentNode, creature->GetGUID().ToString().c_str());
        creature->ClearUnitState(UNIT_STATE_ROAMING_MOVE);
        creature->GetMap()->ScriptsStart(sWaypointScripts, waypoint.EventId, creature, nullptr);
    }

    // inform AI
    if (creature->IsAIEnabled)
    {
        creature->AI()->MovementInform(WAYPOINT_MOTION_TYPE, _currentNode);
        creature->AI()->WaypointReached(waypoint.Id, _path->Id);
    }

    creature->UpdateCurrentWaypointInfo(waypoint.Id, _path->Id);

    _waypointReached = true;
}

void WaypointMovementGenerator<Creature>::StartMove(Creature* creature, bool relaunch /*= false*/)
{
    // sanity checks
    if (!creature || !creature->IsAlive() || _done || !_path || _path->Nodes.empty() || (relaunch && _waypointReached))
        return;

    if (!relaunch)  // on relaunch, can avoid this since its only called on valid movement
    {
        if (!CanMove(creature) || (creature->IsFormationLeader() && !creature->IsFormationLeaderMoveAllowed())) // if cannot move OR cannot move because of formation
        {
            _nextMoveTimer.Reset(1000); // delay 1s
            return;
        }
    }

    // Step one: select next waypoint node. Skip waypoint launch when we have reached the end of the path and shall not repeat.
    if ((_currentNode == _path->Nodes.size() - 1) && !_repeating)
    {
        _done = true;
        creature->UpdateCurrentWaypointInfo(0, 0);

        // Inform the AI that the path has ended.
        if (creature->IsAIEnabled)
            creature->AI()->WaypointPathEnded(_path->Nodes.at(_currentNode).Id, _path->Id);
        return;
    }

    // Step two: select next waypoint node
    _currentNode = (_currentNode + 1) % _path->Nodes.size();

    // Step three: launch next spline
    creature->AddUnitState(UNIT_STATE_ROAMING_MOVE);
    bool const useTransportPath = !creature->GetTransGUID().IsEmpty();
    WaypointNode const& waypoint = _path->Nodes.at(_currentNode);

    Movement::MoveSplineInit init(creature);

    //! If the creature is on transport, we assume waypoints set in DB are already transport offsets
    if (useTransportPath)
        init.DisableTransportPathTransformations();

    if (creature->movespline->Finalized() || _recalculateSpeed)
        init.MoveTo(waypoint.X, waypoint.Y, waypoint.Z);
    else if (_lastSplineId == creature->movespline->GetId())
    {
        init.MoveTo(creature->movespline->FinalDestination(), G3D::Vector3(waypoint.X, waypoint.Y, waypoint.Z));
        if (!init.Path().empty())
            init.Path().insert(init.Path().begin(), G3D::Vector3(creature->GetPositionX(), creature->GetPositionY(), creature->GetPositionZ()));
    }

    //! Accepts angles such as 0.00001 and -0.00001, 0 must be ignored, default value in waypoint table
    if (waypoint.Orientation && waypoint.Delay > 0)
        init.SetFacing(waypoint.Orientation);

    switch (waypoint.MoveType)
    {
        case WAYPOINT_MOVE_TYPE_LAND:
            init.SetAnimation(AnimationTier::Ground);
            break;
        case WAYPOINT_MOVE_TYPE_TAKEOFF:
            init.SetAnimation(AnimationTier::Hover);
            break;
        case WAYPOINT_MOVE_TYPE_RUN:
            init.SetWalk(false);
            break;
        case WAYPOINT_MOVE_TYPE_WALK:
            init.SetWalk(true);
            break;
        default:
            break;
    }

    if (waypoint.Velocity > 0.f)
        init.SetVelocity(waypoint.Velocity);


    int32 moveTime = init.Launch();
    int32 estimatedArrivalTime = std::max<int32>(0, moveTime + waypoint.Delay);
    _movementInformTimer.Reset(std::min<int32>(moveTime, estimatedArrivalTime));
    _nextMoveTimer.Reset(estimatedArrivalTime);

    if (!creature->movespline->Finalized())
        _lastSplineId = creature->movespline->GetId();

    // Inform formation
    creature->SignalFormationMovement();

    // Inform AI
    if (creature->IsAIEnabled)
        creature->AI()->WaypointStarted(waypoint.Id, _path->Id);

    _recalculateSpeed = false;
    _waypointReached = false;
}

bool WaypointMovementGenerator<Creature>::DoUpdate(Creature* creature, uint32 diff)
{
    if (!creature || !creature->IsAlive())
        return true;

    if (_done || !_path || _path->Nodes.empty())
        return true;

    if (_stalled || creature->HasUnitState(UNIT_STATE_NOT_MOVE) || creature->IsMovementPreventedByCasting())
    {
        creature->StopMoving();
        return true;
    }

    if (_recalculateSpeed)
        StartMove(creature, true);

    // Update movement inform timer to notify AI that we have reached our waypoint transition point
    _movementInformTimer.Update(diff);
    if (!_waypointReached && _movementInformTimer.Passed())
        OnArrived(creature);

    // Update next movement timer. Launch new spline when timer has passed.
    _nextMoveTimer.Update(diff);
    if (_nextMoveTimer.Passed())
        StartMove(creature);

    // Creature is moving. Update home position or re-launch spline if the creature's speed has changed while moving.
    if (!creature->movespline->Finalized())
    {
        // set home position at place (every MotionMaster::UpdateMotion)
        if (creature->GetTransGUID().IsEmpty())
            creature->SetHomePosition(creature->GetPosition());

        // relaunch movement if its speed has changed
        if (_recalculateSpeed)
            StartMove(creature, true);
    }

    return true;
}

void WaypointMovementGenerator<Creature>::MovementInform(Creature* creature)
{
    if (creature->AI())
        creature->AI()->MovementInform(WAYPOINT_MOTION_TYPE, _currentNode);
}

bool WaypointMovementGenerator<Creature>::GetResetPos(Creature*, float& x, float& y, float& z)
{
    // prevent a crash at empty waypoint path.
    if (!_path || _path->Nodes.empty())
        return false;

    ASSERT(_currentNode < _path->Nodes.size(), "WaypointMovementGenerator::GetResetPos: tried to reference a node id (%u) which is not included in path (%u)", _currentNode, _path->Id);
    WaypointNode const &waypoint = _path->Nodes.at(_currentNode);

    x = waypoint.X;
    y = waypoint.Y;
    z = waypoint.Z;
    return true;
}

void WaypointMovementGenerator<Creature>::Pause(uint32 timer/* = 0*/)
{
    _stalled = timer ? false : true;
    _nextMoveTimer.Reset(timer ? timer : 1);
}

void WaypointMovementGenerator<Creature>::Resume(uint32 overrideTimer/* = 0*/)
{
    _stalled = false;
    if (overrideTimer)
        _nextMoveTimer.Reset(overrideTimer);
}

/*static*/ bool WaypointMovementGenerator<Creature>::CanMove(Creature* creature)
{
    return !creature->HasUnitState(UNIT_STATE_NOT_MOVE) && !creature->IsMovementPreventedByCasting();
}
