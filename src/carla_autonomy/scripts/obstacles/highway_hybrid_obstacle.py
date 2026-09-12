import carla
import time

def get_ego_vehicle(world):
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'ego_vehicle':
            return actor
    return None


def get_driving_transform(world, location):
    """Project a raw location onto a driving lane and return lane-aligned transform."""
    wmap = world.get_map()
    wp = wmap.get_waypoint(location, project_to_road=True, lane_type=carla.LaneType.Driving)
    if wp is None:
        return carla.Transform(location, carla.Rotation(yaw=90.0)), None

    tf = wp.transform
    tf.location.z = location.z
    return tf, wp


def build_route_locations(start_wp, end_wp, step=5.0, max_points=30):
    """Build a deterministic forward route list for Traffic Manager."""
    if start_wp is None or end_wp is None:
        return []

    route = []
    cur = start_wp
    target_loc = end_wp.transform.location

    for _ in range(max_points):
        route.append(cur.transform.location)
        if cur.transform.location.distance(target_loc) < 6.0:
            break
        nxt = cur.next(step)
        if not nxt:
            break
        cur = nxt[0]

    route.append(target_loc)
    return route

def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    actor_list = []
    tm = client.get_trafficmanager(8000)
    tm.set_random_device_seed(42)

    # 위치 설정
    STATIC_LOC = carla.Location(x=382.71, y=-234.26, z=0.99)
    DYNAMIC_START = carla.Location(x=382.89, y=-258.57, z=0.82)
    DYNAMIC_END = carla.Location(x=385.51, y=-176.46, z=1.73)
    
    TRIGGER_DIST = 50.0
    is_spawned = False

    try:
        print("고속도로 혼합 장애물 감시 중...")
        while True:
            ego = get_ego_vehicle(world)
            if ego and not is_spawned:
                dist = ego.get_location().distance(STATIC_LOC)
                if dist < TRIGGER_DIST:
                    static_tf, _ = get_driving_transform(world, STATIC_LOC)
                    dynamic_start_tf, dynamic_start_wp = get_driving_transform(world, DYNAMIC_START)
                    _, dynamic_end_wp = get_driving_transform(world, DYNAMIC_END)

                    # 1. 정적 장애물 소환
                    static_bp = blueprint_library.filter('vehicle.tesla.model3')[0]
                    static_car = world.try_spawn_actor(static_bp, static_tf)
                    if static_car:
                        actor_list.append(static_car)
                        # 고정

                    # 2. 동적 장애물 소환
                    dynamic_bp = blueprint_library.filter('vehicle.audi.tt')[0]
                    dynamic_car = world.try_spawn_actor(dynamic_bp, dynamic_start_tf)
                    if dynamic_car:
                        actor_list.append(dynamic_car)
                        dynamic_car.set_autopilot(True, tm.get_port())

                        # Deterministic TM behavior: keep lane and follow fixed route.
                        tm.auto_lane_change(dynamic_car, False)
                        tm.random_left_lanechange_percentage(dynamic_car, 0.0)
                        tm.random_right_lanechange_percentage(dynamic_car, 0.0)
                        tm.ignore_vehicles_percentage(dynamic_car, 0.0)
                        tm.distance_to_leading_vehicle(dynamic_car, 6.0)

                        route_locs = build_route_locations(dynamic_start_wp, dynamic_end_wp)
                        if route_locs:
                            tm.set_path(dynamic_car, route_locs)
                        else:
                            tm.set_path(dynamic_car, [DYNAMIC_END])

                        tm.vehicle_percentage_speed_difference(dynamic_car, 20) # 20% 저속
                    
                    print("⚠️ 고속도로 돌발 상황 발생!")
                    is_spawned = True

            if is_spawned:
                time.sleep(15)
                break
            time.sleep(0.5)

    finally:
        for actor in actor_list:
            if actor and actor.is_alive:
                actor.destroy()
        print("고속도로 시나리오 정리 완료.")

if __name__ == '__main__':
    main()