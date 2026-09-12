import carla
import time

def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    V_LOC = carla.Location(x=55.0, y=-324.0, z=3.0)
    TRIGGER_DIST = 50.0
    static_car = None
    
    print("🚗 [정적] 감시 및 자동 삭제 시스템 시작...", flush=True)
    
    try:
        while True:
            ego = next((a for a in world.get_actors() if a.attributes.get('role_name') == 'ego_vehicle'), None)
            
            if ego:
                dist = ego.get_location().distance(V_LOC)
                print(f"현재 거리: {dist:.2f}m", end='\r', flush=True)
                
                # 1. 소환 로직
                if dist < TRIGGER_DIST and static_car is None:
                    spawn_tf = carla.Transform(V_LOC, carla.Rotation(yaw=0))
                    static_car = world.try_spawn_actor(
                        world.get_blueprint_library().find('vehicle.tesla.model3'), spawn_tf)
                    if static_car:
                        static_car.apply_control(carla.VehicleControl(hand_brake=True))
                        print("\n⚠️ 장애물 소환 완료!", flush=True)

                # 2. 자동 삭제 로직: 소환된 상태에서 거리가 다시 60m 이상 멀어지면 삭제
                if static_car is not None and dist > (TRIGGER_DIST + 10):
                    print("\n✅ 장애물 구간 통과. 루프를 종료합니다.", flush=True)
                    break # 루프를 탈출하여 finally의 destroy()를 실행하게 함
            
            time.sleep(0.5)
    finally:
        if static_car:
            static_car.destroy()
            print("✨ 정적 장애물 제거 완료.", flush=True)

if __name__ == '__main__':
    main()
