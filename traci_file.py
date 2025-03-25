
#This code is developed by AI, Claude 3.5 Haiku.

import traci
import time
import xml.etree.ElementTree as ET
import csv
from datetime import datetime


def parse_signal_logics(file_path):
    with open(file_path, 'r') as file:
        content = file.read()
    signal_logics = content.split("Signal ID: ")[1:]
    parsed_logics = {}
    for signal in signal_logics:
        lines = signal.splitlines()
        signal_id = lines[0].strip()
        logic_xml = "\n".join(lines[2:]).strip()
        parsed_logics[signal_id] = logic_xml
    return parsed_logics


def parse_pn_lanes(file_path):
    with open(file_path, 'r') as file:
        lanes = [lane.strip().replace('lane:', '') for lane in file.readlines() if lane.strip()]
    return lanes


def xml_to_traci_logic(logic_xml):
    root = ET.fromstring(logic_xml)
    phases = []
    for phase in root.findall("phase"):
        duration = int(phase.get("duration"))
        state = phase.get("state")
        phases.append(traci.trafficlight.Phase(duration=duration, state=state))

    type_mapping = {"static": 0, "actuated": 1}
    logic_type = type_mapping.get(root.get("type"), 0)

    return traci.trafficlight.Logic(
        programID=root.get("programID"),
        type=logic_type,
        currentPhaseIndex=0,
        phases=phases
    )


def calculate_instantaneous_metrics(pn_lanes, step_length=1.0):
    total_vehicles = 0
    total_distance_travelled = 0

    for lane_id in pn_lanes:
        try:
            vehicles = traci.lane.getLastStepVehicleNumber(lane_id)
            total_vehicles += vehicles

            vehicles_on_lane = traci.lane.getLastStepVehicleIDs(lane_id)
            for vehicle_id in vehicles_on_lane:
                vehicle_speed = traci.vehicle.getSpeed(vehicle_id)  # m/s
                # Adjust for step length - distance traveled in this step
                total_distance_travelled += vehicle_speed * step_length  # meters
        except traci.exceptions.TraCIException as e:
            print(f"Warning: Error processing lane {lane_id}: {e}")
            continue

    return total_vehicles, total_distance_travelled


def store_initial_signal_plans():
    normal_logics = {}
    for signal_id in traci.trafficlight.getIDList():
        try:
            current_logic = traci.trafficlight.getCompleteRedYellowGreenDefinition(signal_id)[0]
            normal_logics[signal_id] = current_logic
        except Exception as e:
            print(f"Error storing normal logic for {signal_id}: {e}")
    return normal_logics


def apply_signal_changes(signal_logics, message=""):
    for signal_id, logic in signal_logics.items():
        if signal_id in traci.trafficlight.getIDList():
            try:
                traci.trafficlight.setProgramLogic(signal_id, logic)
                traci.trafficlight.setProgram(signal_id, logic.programID)
                traci.trafficlight.setPhase(signal_id, 0)
                print(f"{message} Signal plan changed for {signal_id}")
            except Exception as e:
                print(f"Error changing signal logic for {signal_id}: {e}")
        else:
            print(f"Error: Traffic light {signal_id} not found")


def main():
    # Define density thresholds
    NORMAL_THRESHOLD = 20.0
    SOFT_GATING_THRESHOLD = 22.5
    HARD_GATING_THRESHOLD = 25
    CHECK_INTERVAL = 60  # seconds
    SAMPLING_INTERVAL = 1.0  # Fixed 1-second sampling rate

    sumo_binary = "sumo-gui"
    sumo_config = r" Path to your 24h_sim.sumocfg"
    traci.start([sumo_binary, "-c", sumo_config])

    # Get the simulation step length
    step_length = traci.simulation.getDeltaT()
    print(f"Simulation step length: {step_length} seconds")

    # Signal logic file paths
    hard_gating_file = r"File path to Signal_Logics_gating_2.txt"
    soft_gating_file = r"File path to Signal_Logics_gating_1.txt"
    pn_lanes_file = r"File path to All_PN_Vehicular_Lanes.txt"

    # Parse all signal logics
    hard_gating_logics = parse_signal_logics(hard_gating_file)
    soft_gating_logics = parse_signal_logics(soft_gating_file)
    pn_lanes = parse_pn_lanes(pn_lanes_file)

    print(f"Loaded {len(pn_lanes)} protected network lanes")
    print("First few lane IDs:", pn_lanes[:5])

    # Store all signal plan types
    hard_gating_plans = {signal_id: xml_to_traci_logic(logic_xml)
                         for signal_id, logic_xml in hard_gating_logics.items()}
    soft_gating_plans = {signal_id: xml_to_traci_logic(logic_xml)
                         for signal_id, logic_xml in soft_gating_logics.items()}
    normal_plans = store_initial_signal_plans()

    current_mode = "normal"  # Can be "normal", "soft", or "hard"
    last_check_time = 0
    last_sample_time = 0
    cumulative_distance = 0
    metrics_data = []

    # Variables for average counting
    sample_count = 0
    accumulated_vehicles = 0
    interval_distance = 0

    # Calculate total network length once (in km)
    total_length_km = 0
    for lane_id in pn_lanes:
        try:
            total_length_km += traci.lane.getLength(lane_id) / 1000  # Convert m to km
        except traci.exceptions.TraCIException as e:
            print(f"Warning: Error getting length for lane {lane_id}: {e}")

    try:
        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            current_time = traci.simulation.getTime()

            # Sample metrics at 1-second intervals regardless of step length
            if current_time - last_sample_time >= SAMPLING_INTERVAL:
                # Count vehicles and distance at 1-second rate
                vehicles, distance = calculate_instantaneous_metrics(pn_lanes, SAMPLING_INTERVAL)
                accumulated_vehicles += vehicles
                interval_distance += distance  # in meters
                sample_count += 1
                last_sample_time = current_time

            if current_time - last_check_time >= CHECK_INTERVAL:
                # Calculate the average number of vehicles over the interval
                avg_vehicles = accumulated_vehicles / sample_count if sample_count > 0 else 0

                # Calculate the average density (vehicles per km)
                avg_density = avg_vehicles / total_length_km if total_length_km > 0 else 0

                # Calculate average speed (m/s) - total distance divided by (vehicles * time)
                avg_speed_mps = 0
                if avg_vehicles > 0 and CHECK_INTERVAL > 0:
                    # Distance in meters, time in seconds
                    avg_speed_mps = interval_distance / (avg_vehicles * CHECK_INTERVAL)

                # Convert to km/h for reporting
                avg_speed_kmh = avg_speed_mps * 3.6  # 3.6 converts m/s to km/h

                # Calculate flow using q = k * v formula
                # density (k) is in veh/km, speed (v) is in km/h, so flow (q) will be in veh/h
                flow = avg_density * avg_speed_kmh
                # Convert to veh/lane/km (since density is already in veh/lane-km)
                flow_per_lane_km = flow

                # Accumulate total distance travelled
                cumulative_distance += interval_distance

                metrics_data.append({
                    'Time (s)': current_time,
                    'Avg Vehicles': round(avg_vehicles, 2),
                    'Network Length (km)': round(total_length_km, 2),
                    'Network Density (veh/lane-km)': round(avg_density, 2),
                    'Flow (veh/lane/hr)': round(flow_per_lane_km, 2),
                    'Interval Distance (m)': round(interval_distance, 2),
                    'Avg Speed (m/s)': round(avg_speed_mps, 2),
                    'Avg Speed (km/h)': round(avg_speed_kmh, 2),
                    'Cumulative Distance (m)': round(cumulative_distance, 2),
                    'Current Mode': current_mode,
                    'Sample Count': sample_count,
                    'Step Length (s)': step_length
                })

                print(f"\nTime: {current_time} seconds")
                print(f"Average vehicles: {avg_vehicles:.2f}")
                print(f"Total network length: {total_length_km:.2f} km")
                print(f"Average network density: {avg_density:.2f} veh/lane-km")
                print(f"Flow: {flow_per_lane_km:.2f} veh/lane/hr")
                print(f"Distance travelled in last interval: {interval_distance:.2f} meters")
                print(f"Average speed: {avg_speed_mps:.2f} m/s ({avg_speed_kmh:.2f} km/hr)")
                print(f"Cumulative distance travelled: {cumulative_distance:.2f} meters")
                print(f"Upper density threshold: {HARD_GATING_THRESHOLD} vehicles/km")
                print(f"Lower density threshold: {NORMAL_THRESHOLD} vehicles/km")
                print(f"Current mode: {current_mode}")
                print(f"Samples taken in this interval: {sample_count}")
                print(f"Simulation step length: {step_length} seconds")

                # Handle mode transitions based on density
                if current_mode == "normal":
                    if avg_density >= HARD_GATING_THRESHOLD:
                        apply_signal_changes(hard_gating_plans, "Switching to hard gating:")
                        current_mode = "hard"
                    elif avg_density >= SOFT_GATING_THRESHOLD:
                        apply_signal_changes(soft_gating_plans, "Switching to soft gating:")
                        current_mode = "soft"
                elif current_mode == "soft":
                    if avg_density >= HARD_GATING_THRESHOLD:
                        apply_signal_changes(hard_gating_plans, "Switching to hard gating:")
                        current_mode = "hard"
                    elif avg_density < NORMAL_THRESHOLD:
                        apply_signal_changes(normal_plans, "Restoring normal mode:")
                        current_mode = "normal"
                elif current_mode == "hard":
                    if avg_density < SOFT_GATING_THRESHOLD:
                        apply_signal_changes(soft_gating_plans, "Switching to soft gating:")
                        current_mode = "soft"
                    # Will transition to normal mode in next interval if density < NORMAL_THRESHOLD

                # Reset counters for next interval
                last_check_time = current_time
                accumulated_vehicles = 0
                interval_distance = 0
                sample_count = 0

            time.sleep(0)

    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"density_vs_speed_{timestamp}.csv"

        with open(csv_filename, 'w', newline='') as csvfile:
            if metrics_data:
                fieldnames = metrics_data[0].keys()
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(metrics_data)
                print(f"\nMetrics saved to {csv_filename}")

        traci.close()
        print("Simulation ended.")


if __name__ == "__main__":
    main()
