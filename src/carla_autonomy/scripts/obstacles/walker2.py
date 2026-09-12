import carla
import time
import sys

def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        START_LOC = carla.Location(x=263.0, y=-207.0, z=3.0)
        DEST_LOC = carla.Location(x=245.0, y=-207.0, z=0.5)
        TRIGGER_DIST = 42.0
        walker = None

        print("🏃 [보행자2] 로그 기록 시작...", flush=True)
        
        while True:
            # ego_vehicle 찾기
            ego = next((a for a in world.get_actors() if a.attributes.get('role_name') == 'ego_vehicle'), None)
            
            if ego:
                dist = ego.get_location().distance(START_LOC)
                # 로그 확인을 위해 매 초마다 거리를 출력 (줄바꿈 포함)
                print(f"진행 중... 내 차 거리: {dist:.1f}m / 소환상태: {walker is not None}", flush=True)

                if dist < TRIGGER_DIST and walker is None:
                    bp = world.get_blueprint_library().filter('walker.pedestrian.0002')[0]
                    walker = world.try_spawn_actor(bp, carla.Transform(START_LOC, carla.Rotation(yaw=180)))
                    if walker:
                        print("⚠️ [성공] 보행자 2 소환 완료!", flush=True)
                    else:
                        print("❌ [실패] 소환 위치 충돌!", flush=True)

                if walker:
                    walker.apply_control(carla.WalkerControl(direction=carla.Vector3D(x=-1.0, y=0.0, z=0.0), speed=1.6))
                    if walker.get_location().distance(DEST_LOC) < 2.0:
                        print("✅ 목적지 도착. 삭제합니다.", flush=True)
                        break
            else:
                print("검색 중: ego_vehicle을 찾을 수 없습니다.", flush=True)
            
            time.sleep(1.0) # 로그 가독성을 위해 1초로 조정

    except Exception as e:
        print(f"스크립트 에러 발생: {e}", flush=True)
    finally:
        if walker:
            walker.destroy()
            print("✨ 보행자 2 제거 완료.", flush=True)

if __name__ == '__main__':
    main()
