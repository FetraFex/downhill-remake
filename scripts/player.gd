class_name Player
extends RigidBody3D

@export var front: RayCast3D
@export var rear: RayCast3D
@export var spring: float = 8800
@export var damping: float = 450
@export var max_force := 4500.0 
 
# Front Wheel
@export var front_wheel_radius: float
@export var front_ray_to_hub: float 

# Rear Wheel
@export var rear_wheel_radius: float
@export var rear_ray_to_hub: float

@export_group("Drive")
@export var friction := 0.8
@export var pedal_force := 250.0
@export var pedal_max_speed := 10.0
@export var brake_force := 600.0
@export var front_brake_share := 0.6
@export var rolling_coeff := 0.02
@export var air_drag := 0.3
@export var lateral_grip := 20.0

var front_force := 0.0
var rear_force := 0.0

@onready var pcam: PhantomCamera3D = $ThirdPersonCamera
@export var mouse_sensitivity: float = 0.1
var min_pitch: float = -90.0  # Look-up limit (degrees)
var max_pitch: float = 90.0  # Look-down limit (degrees)

var pcam_rotation_degrees: Vector3 = Vector3.ZERO

func _ready() -> void:
	Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
	
	# Optional: Sync with any initial rotation set in the editor
	if pcam:
		pcam_rotation_degrees = pcam.get_third_person_rotation_degrees()

func _input(event: InputEvent) -> void:
	if not pcam:
		return
		
	# Toggle mouse cursor visibility with ESC
	if event.is_action_pressed("ui_cancel"):
		if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
			Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		else:
			Input.mouse_mode = Input.MOUSE_MODE_CAPTURED

	# Process mouse movement to orbit around the target
	if event is InputEventMouseMotion and Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		# Horizontal movement rotates around the Y axis (Yaw)
		pcam_rotation_degrees.y -= event.relative.x * mouse_sensitivity
		
		# Vertical movement rotates around the X axis (Pitch)
		pcam_rotation_degrees.x -= event.relative.y * mouse_sensitivity
		
		# Clamp the pitch so the camera can't flip over the top or bottom
		pcam_rotation_degrees.x = clampf(pcam_rotation_degrees.x, min_pitch, max_pitch)
		
		# Pass the updated degrees directly into Phantom Camera
		pcam.set_third_person_rotation_degrees(pcam_rotation_degrees)

	
func _physics_process(delta: float) -> void:
	front_force = suspension(front, front_wheel_radius, front_ray_to_hub)
	rear_force = suspension(rear, rear_wheel_radius, rear_ray_to_hub)
	
	var pedal := Input.get_action_strength("pedal")
	var brake := Input.get_action_strength("brake")
	var speed := linear_velocity.length()
	
	var drive := pedal * pedal_force * clampf(1.0 - speed / pedal_max_speed, 0.0, 1.0)
	
	tire_forces(front, front_force, 0.0, brake * brake_force * front_brake_share)
	tire_forces(rear, rear_force, drive, brake * brake_force * (1.0 - front_brake_share))
	
	if speed > 0.1:
		apply_central_force(-linear_velocity.normalized() * air_drag * speed * speed)
	
func suspension(ray: RayCast3D, wheel_radius: float, ray_to_hub: float) -> float:
	ray.force_raycast_update()
	
	if not ray.is_colliding():
		return 0.0
	
	var down := (ray.global_basis * ray.target_position).normalized()
	print("Down: ", down)
	var distance := ray.global_position.distance_to(ray.get_collision_point())
	print ("Impact Distance: ", distance)
	
	var compression := ray_to_hub - (distance - wheel_radius)
	if compression <= 0.0:
		return 0.0
	
	var com_global := global_transform * center_of_mass
	var point_velocity := linear_velocity + angular_velocity.cross(ray.global_position - com_global)
	var compression_speed := point_velocity.dot(down)
	
	var force := spring * compression + damping * compression_speed
	force = clamp(force, 0.0, max_force)
	
	var offset := ray.global_position - global_position
	apply_force(ray.get_collision_normal() * force, offset)
	return force
	
func tire_forces(ray: RayCast3D, wheel_load: float, drive: float, brake: float) -> void:
	# No load or no contact: the tyre can't push on anything.
	if wheel_load <= 0.0 or not ray.is_colliding():
		return
 
	var normal := ray.get_collision_normal()
	var contact := ray.get_collision_point()
 
	# Two directions lying on the ground: where the tyre rolls ("forward") and its side ("right").
	var forward := -global_basis.z
	forward = (forward - normal * forward.dot(normal)).normalized()
	var right := forward.cross(normal)
 
	# Speed of the contact point, split into "along the road" and "sideways".
	var com_global := global_transform * center_of_mass
	var contact_velocity := linear_velocity + angular_velocity.cross(contact - com_global)
	var v_long := contact_velocity.dot(forward)
	var v_side := contact_velocity.dot(right)
 
	# ALONG the road: brake + rolling resistance always oppose the motion. They fade to zero
	# near standstill (below 0.5 m/s) so the force doesn't flip sign every frame.
	var fade := clampf(absf(v_long) / 0.5, 0.0, 1.0)
	var resist := (brake + rolling_coeff * wheel_load) * fade
	var force_long := drive - signf(v_long) * resist
 
	# SIDEWAYS: the tyre pushes against any sideways sliding, harder the faster it slides.
	# Each wheel carries about half of the bike's mass.
	var wheel_mass := mass * 0.5
	var force_side := -wheel_mass * lateral_grip * v_side
 
	# Grip budget ("friction circle"): along + sideways together can't exceed friction * load.
	# If you ask for more, both are scaled down: a wheel that brakes hard has less grip left to corner.
	var tyre_force := Vector2(force_long, force_side)
	var grip_limit := friction * wheel_load
	if tyre_force.length() > grip_limit:
		tyre_force = tyre_force.normalized() * grip_limit
 
	apply_force(forward * tyre_force.x + right * tyre_force.y, contact - global_position)
