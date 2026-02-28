import carla
import weakref


class CollisionSensor(object):

    def __init__(self, parent_actor, sensor_name='collision', trans = None):
        self.sensor = None
        self._parent = parent_actor
        self.collided: bool = False
        self.collided_with: str = ''

        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        bp.set_attribute('role_name', sensor_name)

        if trans == None:
            trans = carla.Transform()

        self.sensor = world.spawn_actor(
            bp,
            trans,
            attach_to=self._parent
        )

        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda event: CollisionSensor._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if event.other_actor.type_id == "static.road":
            return
        self.collided = True
        self.collided_with = event.other_actor.type_id
        if not self:
            return
        
    def check_collision(self) -> bool:
        return self.collided
    
    def check_collision_with(self) -> str:
        return self.collided_with
        

class LaneInvasionSensor(object):

    def __init__(self, parent_actor, sensor_name='lane_invasion', trans = None):
        self.sensor = None
        self.lines_invaded: List[carla.LaneMarking] = []

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor

            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            bp.set_attribute('role_name', sensor_name)

            if trans == None:
                trans = carla.Transform()

            self.sensor = world.spawn_actor(
                bp,
                trans,
                attach_to=self._parent
            )

            weak_self = weakref.ref(self)
            self.sensor.listen(
                lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lines_invaded = list(
            map(lambda _l: str(_l.type), event.crossed_lane_markings))

    def check_lane_invasion(self) -> bool:
        return len(self.lines_invaded) > 0
