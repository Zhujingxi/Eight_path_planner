import numpy as np


def generate_figure_eight_waypoints(lat0, lon0, size_meters, altitude, num_points_per_loop, rotation_degrees=0,
                                    loop_count=1):
    """
    Generates waypoints for a figure-eight flight path with rotation and loop count.

    Parameters:
        lat0 (float): Latitude of the center point in decimal degrees.
        lon0 (float): Longitude of the center point in decimal degrees.
        size_meters (float): Size of the figure-eight pattern in meters.
        altitude (float): Altitude in meters for all waypoints.
        num_points_per_loop (int): Number of waypoints to generate per loop.
        rotation_degrees (float): Rotation of the figure-eight pattern in degrees.
        loop_count (int): Number of times to repeat the figure-eight pattern.

    Returns:
        list of tuples: A list containing (latitude, longitude, altitude) for each waypoint.
    """
    # Earth's radius in meters
    R = 6378137.0  # Mean Earth radius

    # Convert degrees to radians for latitude
    lat0_rad = np.deg2rad(lat0)

    # Meters per degree latitude and longitude
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = 111320.0 * np.cos(lat0_rad)

    # Rotation in radians
    rotation_rad = np.deg2rad(rotation_degrees)

    # Semi-major axis of the figure eight
    a = size_meters / 2.0

    # Total number of points
    total_num_points = num_points_per_loop * loop_count

    # Parameter t for the parametric equations
    t = np.linspace(0, 2 * np.pi * loop_count, total_num_points, endpoint=False)

    # Parametric equations for the figure-eight path
    x = a * np.sin(t)
    y = a * np.sin(t) * np.cos(t)

    # Apply rotation
    x_rot = x * np.cos(rotation_rad) - y * np.sin(rotation_rad)
    y_rot = x * np.sin(rotation_rad) + y * np.cos(rotation_rad)

    # Convert x and y from meters to degrees
    delta_lat = y_rot / meters_per_deg_lat
    delta_lon = x_rot / meters_per_deg_lon

    # Calculate waypoints
    latitudes = lat0 + delta_lat
    longitudes = lon0 + delta_lon
    altitudes = np.full(total_num_points, altitude)

    # Combine into a list of tuples
    waypoints = list(zip(latitudes, longitudes, altitudes))

    return waypoints


def write_mission_file(waypoints, filename, takeoff_altitude):
    """
    Writes the waypoints to a mission file compatible with Ardupilot, including takeoff and landing.

    Parameters:
        waypoints (list of tuples): List containing (latitude, longitude, altitude) for each waypoint.
        filename (str): Name of the file to write the mission to.
        takeoff_altitude (float): Altitude to climb to during takeoff.
    """
    with open(filename, 'w') as f:
        f.write('QGC WPL 110\n')
        # Home location (index 0)
        home_lat, home_lon, home_alt = waypoints[0]
        index = 0
        current_wp = 1  # Indicates this is the current waypoint
        frame = 0  # MAV_FRAME_GLOBAL
        command = 16  # MAV_CMD_NAV_WAYPOINT
        params = [0, 0, 0, 0, home_lat, home_lon, home_alt]
        autocontinue = 1
        line = f"{index}\t{current_wp}\t{frame}\t{command}\t{params[0]}\t{params[1]}\t{params[2]}\t{params[3]}\t{params[4]}\t{params[5]}\t{params[6]}\t{autocontinue}\n"
        f.write(line)

        # Takeoff command (index 1)
        index = 1
        current_wp = 0  # Not current waypoint
        frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = 22  # MAV_CMD_NAV_TAKEOFF
        param1 = 0  # Minimum pitch (ignored by copter)
        param2 = 0  # Empty
        param3 = 0  # Empty
        param4 = 0  # Yaw angle
        takeoff_lat = home_lat
        takeoff_lon = home_lon
        takeoff_alt = takeoff_altitude
        autocontinue = 1
        line = f"{index}\t{current_wp}\t{frame}\t{command}\t{param1}\t{param2}\t{param3}\t{param4}\t{takeoff_lat}\t{takeoff_lon}\t{takeoff_alt}\t{autocontinue}\n"
        f.write(line)

        # Waypoints starting from index 2
        for i, (lat, lon, alt) in enumerate(waypoints):
            index = i + 2  # Adjust index because of home and takeoff commands
            current_wp = 0
            frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
            command = 16  # MAV_CMD_NAV_WAYPOINT
            param1 = 0  # Hold time in seconds
            param2 = 0  # Acceptance radius in meters
            param3 = 0  # Pass through waypoint
            param4 = 0  # Desired yaw angle
            autocontinue = 1
            line = f"{index}\t{current_wp}\t{frame}\t{command}\t{param1}\t{param2}\t{param3}\t{param4}\t{lat}\t{lon}\t{alt}\t{autocontinue}\n"
            f.write(line)

        # Landing command at the end
        index += 1
        current_wp = 0
        frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        command = 21  # MAV_CMD_NAV_LAND
        param1 = 0  # Abort Alt
        param2 = 0  # Precision land mode
        param3 = 0  # Empty
        param4 = 0  # Desired yaw angle
        landing_lat = home_lat
        landing_lon = home_lon
        landing_alt = 0  # Landing altitude is usually zero
        autocontinue = 1
        line = f"{index}\t{current_wp}\t{frame}\t{command}\t{param1}\t{param2}\t{param3}\t{param4}\t{landing_lat}\t{landing_lon}\t{landing_alt}\t{autocontinue}\n"
        f.write(line)


def main():
    # User-defined parameters
    lat0 = 42.730326557346  # Center latitude
    lon0 = -73.6799289286137  # Center longitude
    size_meters = 50  # Size of the figure eight
    altitude = 20  # Altitude in meters
    num_points_per_loop = 100  # Number of waypoints per loop
    rotation_degrees = -10  # Rotation of the figure eight in degrees
    loop_count = 2  # Number of loops
    takeoff_altitude = 10  # Takeoff altitude in meters

    # Generate waypoints
    waypoints = generate_figure_eight_waypoints(
        lat0, lon0, size_meters, altitude, num_points_per_loop,
        rotation_degrees=rotation_degrees, loop_count=loop_count)

    # Write mission file
    filename = 'figure_eight_mission.waypoints'
    write_mission_file(waypoints, filename, takeoff_altitude)
    print(f"Mission file '{filename}' has been generated successfully.")


if __name__ == "__main__":
    main()
