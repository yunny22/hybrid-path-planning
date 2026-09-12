import carla
import time
import sys

def main():
    try:
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        
        START_LOC = carla.Location(x=207.72, y=-363.22, z=3.0) # 하늘 소환
        END_LOC = carla.Location(x=349.0, y=-336.0, z=2.0)
        TRIGGER_DIST = 38.0
        car = None

        print("🛣️ [고속도로] 시스템 가동 및 로그 시작...", flush=True)
        
        while True:
            ego = next((a for a in world.get_actors() if a.attributes.get('role_name') == 'ego_vehicle'), None)
            
            if ego:
                dist = ego.get_location().distance(START_LOC)
                print(f"고속도로 대기 중... 내 차 거리: {dist:.1f}m / 소환상태: {car is not None}", flush=True)

                if dist < TRIGGER_DIST and car is None:
                    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
                    car = world.try_spawn_actor(bp, carla.Transform(START_LOC))
                    if car:
                        car.set_autopilot(True)
                        tm = client.get_trafficmanager()
                        
                        # 도로 규정 속도 대비 몇 % 느리게 갈지 설정합니다.
                        # 예: 50.0 = 규정 속도의 절반(50% 감속)으로 주행
                        # 예: 80.0 = 규정 속도보다 80% 느리게 (거의 기어감)
                        # 예: -20.0 = 규정 속도보다 20% 빠르게 (과속)
                        tm.vehicle_percentage_speed_difference(car, 70.0)
                        print("⚠️ [성공] 고속도로 차량 소환 완료!", flush=True)
                    else:
                        print("❌ [실패] 고속도로 소환 위치 충돌!", flush=True)

                if car:
                    if car:
                        time.sleep(15)
                        print("✅ 고속도로 차량 끝점 도달. 삭제합니다.", flush=True)
                        break
            else:
                print("검색 중: ego_vehicle을 찾을 수 없습니다.", flush=True)
            
            time.sleep(1.0)
    except Exception as e:
        print(f"에러: {e}", flush=True)
    finally:
        if car:
        
            car.destroy()
            print("✨ 고속도로 차량 제거 완료.", flush=True)

if __name__ == '__main__': main()
