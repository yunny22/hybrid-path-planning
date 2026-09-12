import carla
import time

def get_ego_vehicle(world):
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == 'ego_vehicle':
            return actor
    return None

def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    actor_list = []

    # 좌표 설정
    CAR1_START = carla.Location(x=202.00, y=-162.24, z=1.22)
    CAR1_END   = carla.Location(x=202.20, y=-182.66, z=1.11)
    CAR2_START = carla.Location(x=198.17, y=-182.41, z=1.11)
    CAR2_END   = carla.Location(x=198.00, y=-161.02, z=0.80)
    
    TRIGGER_DIST = 70.0
    SLOWDOWN_PERCENT = 55
    is_triggered = False

    try:
        print("교차로 장애물 감시 중...")
        while True:
            ego = get_ego_vehicle(world)
            if ego and not is_triggered:
                dist = ego.get_location().distance(CAR1_START)
                if dist < TRIGGER_DIST:
                    print("⚠️ 교차로 진입 감지! 장애물 소환 시작")
                    tm = client.get_trafficmanager(8000)

                    # 차량 1 소환 및 출발
                    v_bp1 = blueprint_library.filter('vehicle.tesla.model3')[0]
                    tf1 = carla.Transform(CAR1_START, carla.Rotation(yaw=-90))
                    car1 = world.try_spawn_actor(v_bp1, tf1)
                    if car1:
                        actor_list.append(car1)
                        car1.set_autopilot(True, tm.get_port())
                        tm.set_path(car1, [CAR1_END])
                        tm.vehicle_percentage_speed_difference(car1, SLOWDOWN_PERCENT)

                    time.sleep(1.0) # 1초 대기 후 두 번째 차량 출발

                    # 차량 2 소환 및 출발
                    v_bp2 = blueprint_library.filter('vehicle.audi.tt')[0]
                    tf2 = carla.Transform(CAR2_START, carla.Rotation(yaw=90))
                    car2 = world.try_spawn_actor(v_bp2, tf2)
                    if car2:
                        actor_list.append(car2)
                        car2.set_autopilot(True, tm.get_port())
                        tm.set_path(car2, [CAR2_END])
                        tm.vehicle_percentage_speed_difference(car2, SLOWDOWN_PERCENT)
                    
                    is_triggered = True

            # 두 차량이 모두 목적지 근처에 가면 종료 로직
            if is_triggered:
                # 간단히 10초 후 혹은 거리 기반으로 종료 가능
                time.sleep(10)
                break
                
            time.sleep(0.5)

    finally:
        for actor in actor_list:
            if actor and actor.is_alive:
                actor.destroy()
        print("교차로 시나리오 종료.")

if __name__ == '__main__':
    main()