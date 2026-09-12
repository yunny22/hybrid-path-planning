import carla
import time

def main():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    S_LOC = carla.Location(x=120.0, y=-179.0, z=2.0)
    TRIGGER_DIST = 70.0
    walker = None
    print("🏃 [보행자1] 차량 접근 감시 시작...")
    try:
        while True:
            ego = next((a for a in world.get_actors() if a.attributes.get('role_name') == 'ego_vehicle'), None)
            if ego:
                dist = ego.get_location().distance(S_LOC)
                if dist < TRIGGER_DIST and walker is None:
                    bp = world.get_blueprint_library().filter('walker.pedestrian.0001')[0]
                    walker = world.try_spawn_actor(bp, carla.Transform(S_LOC))
                    if walker: print("🏃 보행자 1 출현!")
                if walker:
                    ctrl = carla.WalkerControl(direction=carla.Vector3D(x=1.0, y=0.0, z=0.0), speed=1.4)
                    walker.apply_control(ctrl)
                    if walker.get_location().x >= 135.0: break
            time.sleep(0.05)
    finally:
        if walker: walker.destroy()

if __name__ == '__main__': main()
