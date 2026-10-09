"""Tests for the plant's stand-in autopilot: orders from Node A, battery, flight modes."""

import pytest

from sim.plant.autopilot import Autopilot, Battery, Command, parse_command, sentence
from sim.plant.plant import STEP_S, Plant, Scenario
from sim.plant.vehicle import FlightPlan


def run(plant: Plant, seconds: float) -> list:
    return [plant.step() for _ in range(round(seconds / STEP_S))]


def order(plant: Plant, mode: str, avoid: bool = False, min_dm: int = 0) -> None:
    plant.receive(sentence(f"VDCMD,{mode},{int(avoid)},{min_dm}"))


def test_parses_node_a_commands() -> None:
    # Checksums as in the firmware tests (tests/unit/test_ap_msg.c).
    assert parse_command("$VDCMD,MISSION,1,150*3D") == Command("MISSION", True, 15.0)
    assert parse_command("$VDCMD,RTH,0,0*3A\r") == Command("RTH", False, 0.0)
    assert parse_command("$VDCMD,MISSION,1,150*3E") is None      # checksum
    assert parse_command(sentence("VDCMD,FLY,0,0")) is None       # mode
    assert parse_command(sentence("VDCMD,HOLD,2,0")) is None      # avoid flag
    assert parse_command(sentence("VDCMD,HOLD,0")) is None        # field count


def test_reassembles_commands_split_across_reads() -> None:
    autopilot = Autopilot(Battery())
    assert autopilot.receive("$VDCMD,RT") == []
    assert autopilot.receive("H,0,0*3A\r\n$VDCMD,bad*00\r\n") == [Command("RTH", False, 0.0)]
    assert (autopilot.mode, autopilot.commands, autopilot.rejected) == ("RTH", 1, 1)


def test_reports_battery_and_mode_at_5_hz() -> None:
    plant = Plant(Scenario(battery_pct=50.0, battery_drain_pct_per_s=1.0))
    status = [s.autopilot_tx for s in run(plant, 1.95) if s.autopilot_tx]
    assert len(status) == 10
    assert status[0] == sentence("VDAPS,49,MISSION")
    assert status[-1] == sentence("VDAPS,48,MISSION")


def test_keep_out_holds_the_vehicle_off_the_line_through_the_close_pass() -> None:
    plant = Plant(Scenario(plan=FlightPlan(start_s_m=180.0)))
    run(plant, 3.0)
    order(plant, "MISSION", avoid=True, min_dm=150)
    run(plant, 4.0)                                        # time to move out
    close = run(plant, 15.0)                               # along the close part of the plan
    assert plant.vehicle.plan.offset_at(close[-1].state.s_m) == pytest.approx(10.0)
    assert min(s.state.distance_to_line_m for s in close) > 15.0

    # Avoidance withdrawn: the plan still comes too close, so the keep-out stays.
    order(plant, "MISSION")
    later = run(plant, 3.0)
    assert plant.autopilot.keep_out_m == pytest.approx(16.0)
    assert min(s.state.distance_to_line_m for s in later) > 15.0


def test_ignores_avoidance_when_faulty() -> None:
    plant = Plant(Scenario(obey_avoidance=False))
    order(plant, "MISSION", avoid=True, min_dm=150)
    assert plant.autopilot.keep_out_m is None


def test_hold_stops_and_mission_resumes() -> None:
    plant = Plant()
    run(plant, 10.0)
    order(plant, "HOLD")
    run(plant, 1.0)
    s_held = plant.vehicle.s
    held = run(plant, 5.0)
    assert plant.vehicle.s == s_held
    assert held[-1].state.velocity.norm() < 1.0
    order(plant, "MISSION")
    run(plant, 2.0)
    assert plant.vehicle.s > s_held + 5.0


def test_return_to_home_flies_back_to_the_start() -> None:
    plant = Plant()
    run(plant, 15.0)
    order(plant, "RTH")
    back = run(plant, 30.0)
    assert back[-1].state.s_m == 0.0
    assert back[-1].state.mode == "RTH"
    assert back[-1].state.position.norm() < 25.0 + 20.0 + 2.0     # near the start point


def test_land_descends_at_1_mps_and_stays_down() -> None:
    plant = Plant(Scenario(battery_pct=50.0, battery_drain_pct_per_s=1.0))
    run(plant, 5.0)
    order(plant, "LAND")
    descent = run(plant, 10.0)
    rate = (descent[300].state.position.z - descent[800].state.position.z) / 5.0
    assert rate == pytest.approx(1.0, abs=0.1)
    landed = run(plant, 20.0)
    assert not landed[-1].state.airborne and landed[-1].state.position.z == 0.0
    battery = plant.battery.pct
    order(plant, "MISSION")                                # LAND is final
    run(plant, 2.0)
    assert plant.vehicle.mode == "LAND" and plant.battery.pct == battery
