import carla
import time
import sys

def main():
    try:
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        
        START_LOC = carla.Location(x=315.0, y=-191.0, z=3.0) 
        END_LOC = carla.Location(x=240.0, y=-191.0, z=2.0)
        TRIGGER_DIST = 35.0
        SLOWDOWN_PERCENT = 70.0
        car = None

        print("🛣️ [고속도로] 시스템 가동 및 로그 시작...", flush=True)
        
        while True:
            ego = next((a for a in world.get_actors() if a.attributes.get('role_name') == 'ego_vehicle'), None)
            
            if ego:
                dist = ego.get_location().distance(START_LOC)
                print(f"고속도로 대기 중... 내 차 거리: {dist:.1f}m / 소환상태: {car is not None}", flush=True)

                if dist < TRIGGER_DIST and car is None:
                    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
                    
                    # ==========================================
                    # 🚀 CARLA 맵에서 현재 위치의 '진짜 차선 방향'을 읽어옵니다!
                    waypoint = world.get_map().get_waypoint(START_LOC)
                    START_ROT = waypoint.transform.rotation
                    
                    # 위치는 내가 정한 공중(START_LOC), 방향은 도로 방향(START_ROT)
                    spawn_tf = carla.Transform(START_LOC, START_ROT)
                    
                    # 최종 소환!
                    car = world.try_spawn_actor(bp, spawn_tf)
                    # ==========================================

                    if car:
                        tm = client.get_trafficmanager()
                        car.set_autopilot(True, tm.get_port())
                        tm.vehicle_percentage_speed_difference(car, SLOWDOWN_PERCENT)
                        print("⚠️ [성공] 고속도로 차량 소환 완료! (차선 방향 자동 정렬 100%)", flush=True)
                    else:
                        print("❌ [실패] 고속도로 소환 위치 충돌!", flush=True)
                if car:
                    if car:
                        time.sleep(25)
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
