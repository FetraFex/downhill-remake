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

func suspension(ray: RayCast3D, wheel_radius: float, ray_to_hub: float) -> void:
	ray.force_raycast_update()
	
	if not ray.is_colliding():
		return
	
	var down := (ray.global_basis * ray.target_position).normalized()
	print("Down: ", down)
	var distance := ray.global_position.distance_to(ray.get_collision_point())
	print ("Impact Distance: ", distance)
	
	var compression := ray_to_hub - (distance - wheel_radius)
	if compression <= 0.0:
		return
	
	var com_global := global_transform * center_of_mass
	var point_velocity := linear_velocity + angular_velocity.cross(ray.global_position - com_global)
	var compression_speed := point_velocity.dot(down)
	
	var force := spring * compression + damping * compression_speed
	force = clamp(force, 0.0, max_force)
	
	var offset := ray.global_position - global_position
	apply_force(ray.get_collision_normal() * force, offset)
	
func _physics_process(delta: float) -> void:
	suspension(front, front_wheel_radius, front_ray_to_hub)
	suspension(rear, rear_wheel_radius, rear_ray_to_hub)
