import carla

def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    
    # 에서 사용한 모델들 리스트
    obs_models = ['vehicle.tesla.model3', 'vehicle.audi.tt', 'walker.pedestrian.*']
    
    destroyed_count = 0
    actors = world.get_actors()
    
    print("🧹 장애물 클린업 시작...")
    
    for actor in actors:
        # 우리가 소환한 장애물 모델인지 확인
        # (단, 내 차량인 ego_vehicle은 제외해야 합니다)
        if any(actor.type_id.startswith(model.replace('*', '')) for model in obs_models):
            if actor.attributes.get('role_name') != 'ego_vehicle':
                actor.destroy()
                destroyed_count += 1
                
    print(f"✅ 총 {destroyed_count}개의 장애물 액터를 제거했습니다.")

if __name__ == '__main__':
    main()
    