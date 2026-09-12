import carla
import time

def main():
    # 1. CARLA 연결
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 2. 내 차량(ego_vehicle) 찾기
    ego_vehicle = None
    while ego_vehicle is None:
        print("🔍 차량(ego_vehicle)을 찾는 중...")
        for actor in world.get_actors():
            if actor.attributes.get('role_name') == 'ego_vehicle':
                ego_vehicle = actor
                break
        time.sleep(1)

    print("✅ 차량 발견! 카메라 추적을 시작합니다. (Ctrl+C로 종료)")

    # 3. 카메라(Spectator) 추적 루프
    spectator = world.get_spectator()

    try:
        while True:
            # 차량의 현재 위치와 방향 가져오기
            vehicle_transform = ego_vehicle.get_transform()
            vehicle_loc = vehicle_transform.location
            vehicle_rot = vehicle_transform.rotation

            # 카메라 위치 설정 (차량 뒤쪽 10m, 위쪽 5m)
            # 차량의 방향(Yaw)을 고려하여 뒤쪽 좌표 계산
            yaw_rad = vehicle_rot.yaw * (3.14159 / 180.0)
            offset_x = -10.0 * (1.0 if abs(vehicle_rot.yaw) < 90 else -1) # 단순화된 계산
            
            # 더 정확한 카메라 위치 계산 (차량 뒤에서 내려다보는 뷰)
            fwd_vec = vehicle_transform.get_forward_vector()
            spectator_pos = vehicle_loc - fwd_vec * 10.0 + carla.Location(z=5.0)
            
            # 카메라가 차량을 내려다보도록 각도 조절 (-30도 정도)
            spectator_rot = carla.Rotation(pitch=-30, yaw=vehicle_rot.yaw, roll=0)
            
            # 카메라 즉시 이동
            spectator.set_transform(carla.Transform(spectator_pos, spectator_rot))
            
            time.sleep(0.02) # 50Hz

    except KeyboardInterrupt:
        print("\n종료합니다.")

if __name__ == '__main__':
    main()