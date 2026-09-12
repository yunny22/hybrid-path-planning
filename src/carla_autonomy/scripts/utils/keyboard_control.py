import carla
import pygame
import time

def main():
    # 1. 초기화 및 연결
    pygame.init()
    display = pygame.display.set_mode((400, 300)) # 입력 감지를 위한 작은 창
    pygame.display.set_caption("CARLA Keyboard Control")
    
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    # 2. ego_vehicle 스폰
    blueprint = world.get_blueprint_library().find('vehicle.tesla.model3')
    blueprint.set_attribute('role_name', 'ego_vehicle') # 트리거 인식을 위해 필수!
    
    spawn_point = world.get_map().get_spawn_points()[0] # 기본 스폰 지점
    vehicle = world.spawn_actor(blueprint, spawn_point)
    
    print("🚗 ego_vehicle 생성 완료! (W: 가속, S: 감속, A/D: 조향, Space: 핸드브레이크)")

    try:
        control = carla.VehicleControl()
        while True:
            # pygame 이벤트 처리
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            keys = pygame.key.get_pressed()
            
            # 키보드 입력 매핑
            control.throttle = 1.0 if keys[K_w] else 0.0
            control.brake = 1.0 if keys[K_s] else 0.0
            control.steer = -0.5 if keys[K_a] else 0.5 if keys[K_d] else 0.0
            control.hand_brake = keys[K_SPACE]
            control.reverse = keys[K_q] # Q키를 누르면 후진 모드 (가속키와 함께 사용)

            vehicle.apply_control(control)
            pygame.display.flip()
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n중단되었습니다.")
    finally:
        if vehicle:
            vehicle.destroy()
            print("차량이 제거되었습니다.")
        pygame.quit()

# Pygame 키 상수 정의 (환경에 따라 직접 import 가능)
from pygame.locals import K_w, K_s, K_a, K_d, K_SPACE, K_q

if __name__ == '__main__':
    main()