class_name Player
extends RigidBody3D

signal landed(quality: float, air_time: float)
signal crashed
signal jumped(strength: float)

enum State {GROUNDED, AIRBORNE, CRASHED}

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

@export_group("Steering")
@export var max_steer := 0.35
@export var steer_speed_factor := 0.15
@export var steer_smoothing :=  8.0

@export_group("Balance (virtual rider)")
@export var balance_stiffness := 2500.0
@export var balance_damping := 350.0
@export var max_lean := 0.9
@export var gravity := 9.8

@export_group("Rider (Weigh Shift)")
@export var rider_travel := 0.3
@export var rider_speed := 5.0
@export var rider_mass_share := 0.83
@export var auto_pitch_stiffness := 1.0
@export var auto_pitch_damping := 0.2
@export var slope_shift := 0.5

@export_group("Air and Crash")
@export var air_delay :=  0.12
@export var air_pitch_torque := 180.0
@export var air_yaw_torque := 60.0
@export var air_roll_stiffness := 400.0
@export var air_damping := 60.0
@export var crash_tilt := 1.1
@export var crash_time := 1.5
@export var respawn_lift := 0.4

@export_group("Jump")
@export var jump_charge_time := 0.6
@export var preload_force := 800.0
@export var pop_impulse := 300.0
@export var pop_front_share := 0.6
@export var pop_rear_delay :=  0.08
@export var jump_coyote :=  0.12
@export var jump_cooldown := 0.3

var front_force := 0.0
var rear_force := 0.0
var front_compression := 0.0
var rear_compression := 0.0
var _last_compression := 0.0 
var steer_angle := 0.0
var rider_shift := 0.0
var _com_rest := Vector3.ZERO

var state := State.GROUNDED
var air_time := 0.0
var _since_ground := 0.0
var _crash_timer := 0.0
var _grace := 1.0
var _safe_position := Vector3.ZERO
var _safe_yaw := 0.0

var _jump_charge := 0.0
var _jump_cooldown :=  0.0
var _pop_rear_timer :=  0.0
var _pop_rear_strength := 0.0
var _pop_direction := Vector3.UP

@onready var pcam: PhantomCamera3D = $ThirdPersonCamera
@export var mouse_sensitivity: float = 0.1
var min_pitch: float = -90.0  # Look-up limit (degrees)
var max_pitch: float = 90.0  # Look-down limit (degrees)

var pcam_rotation_degrees: Vector3 = Vector3.ZERO

func _ready() -> void:
	can_sleep = false
	_com_rest = center_of_mass
	
	if center_of_mass_mode != RigidBody3D.CENTER_OF_MASS_MODE_CUSTOM:
		push_warning("Set Center Of Mass Mode to Custom on the Player, or the weight shift won't work.")
	
	contact_monitor = true
	max_contacts_reported = 4
	body_entered.connect(_on_body_entered)
	
	_safe_position = global_position
	var f := -global_basis.z
	_safe_yaw = atan2(-f.x, -f.z)
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
	front_compression = _last_compression
	rear_force = suspension(rear, rear_wheel_radius, rear_ray_to_hub)
	rear_compression = _last_compression	
	
	# State 
	update_state(delta)
	if state == State.CRASHED:
		return
	
	var pedal := Input.get_action_strength("pedal")
	var brake := Input.get_action_strength("brake")
	var speed := linear_velocity.length()
	
	var drive := pedal * pedal_force * clampf(1.0 - speed / pedal_max_speed, 0.0, 1.0)
	
	var steer_input := Input.get_axis("steer_right", "steer_left")
	var steer_limit := max_steer / (1.0 + speed * steer_speed_factor)
	steer_angle = lerpf(steer_angle, steer_input * steer_limit, 1.0 - exp(-steer_smoothing * delta))

	tire_forces(front, front_force, 0.0, brake * brake_force * front_brake_share, steer_angle)
	tire_forces(rear, rear_force, drive, brake * brake_force * (1.0 - front_brake_share), 0.0)
	
	# Balance
	balance()
	rider_pitch(delta)
	jump_control(delta)
	
	if state == State.AIRBORNE:
		air_control()
	
	if speed > 0.1:
		apply_central_force(-linear_velocity.normalized() * air_drag * speed * speed)
	
func suspension(ray: RayCast3D, wheel_radius: float, ray_to_hub: float) -> float:
	ray.force_raycast_update()
	
	if not ray.is_colliding():
		return 0.0
	
	var down := (ray.global_basis * ray.target_position).normalized()
	var distance := ray.global_position.distance_to(ray.get_collision_point())
	
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
	
func tire_forces(ray: RayCast3D, wheel_load: float, drive: float, brake: float, steer: float) -> void:
	# No load or no contact: the tyre can't push on anything.
	if wheel_load <= 0.0 or not ray.is_colliding():
		return
 
	var normal := ray.get_collision_normal()
	var contact := ray.get_collision_point()
 
	# Two directions lying on the ground: where the tyre rolls ("forward") and its side ("right").
	var forward := -global_basis.z
	forward = (forward - normal * forward.dot(normal)).normalized()
	forward = forward.rotated(normal, steer)
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


func balance() -> void:
	var normal := ground_normal()
	if normal == Vector3.ZERO:
		return   # in the air there is no ground to balance against
 
	# Forward direction on the ground, and speed along it.
	var forward := -global_basis.z
	forward = (forward - normal * forward.dot(normal)).normalized()
	var speed := linear_velocity.dot(forward)
 
	# Lean angle the current turn needs (positive = left), limited to max_lean.
	var yaw_rate := angular_velocity.dot(normal)
	var lean := clampf(atan2(speed * yaw_rate, gravity), -max_lean, max_lean)
 
	# The "up" the rider wants: the ground normal tilted sideways by the lean angle.
	var target_up := normal.rotated(forward, -lean)
 
	# PD controller on the roll axis: push toward the target, damp the roll speed.
	var roll_error := global_basis.y.cross(target_up).dot(forward)
	var roll_rate := angular_velocity.dot(forward)
	var torque := balance_stiffness * roll_error - balance_damping * roll_rate
	apply_torque(forward * torque)
	

func ground_normal() -> Vector3:
	var normal := Vector3.ZERO
	if front.is_colliding() and front_force > 0.0:
		normal += front.get_collision_normal()
	if rear.is_colliding() and rear_force > 0.0:
		normal += rear.get_collision_normal()
	return normal.normalized()
	

func rider_pitch(delta: float) -> void:
	var target := 0.0
	var normal := ground_normal()
 
	if normal != Vector3.ZERO:
		var forward := -global_basis.z
		var pitch := asin(clampf(forward.dot(normal), -1.0, 1.0))   # positive = nose above the ground
		var pitch_rate := angular_velocity.dot(global_basis.x)      # positive = nose rising
		var forward_ground := (forward - normal * forward.dot(normal)).normalized()
		var slope := asin(clampf(forward_ground.y, -1.0, 1.0))      # positive = uphill
 
		# Reflex: nose up -> weight forward (pushes the nose down). Nose down -> weight back.
		var auto := auto_pitch_stiffness * pitch + auto_pitch_damping * pitch_rate + slope_shift * slope
 
		# Your input wins over the reflex.
		var manual := Input.get_axis("lean_back", "lean_forward")    # +1 = forward
		target = manual * rider_travel + auto * (1.0 - absf(manual))
		target = clampf(target, -rider_travel, rider_travel)
 
	# In the air (target 0) the rider returns to the middle.
	rider_shift = lerpf(rider_shift, target, 1.0 - exp(-rider_speed * delta))
 
	# Moving the rider by d moves the COM of the whole bike by d * rider_mass_share.
	# Forward on the bike is -Z, so a forward shift means a negative Z offset.
	center_of_mass = _com_rest + Vector3(0.0, 0.0, -rider_shift * rider_mass_share)
 

func update_state(delta: float) -> void:
	_grace = maxf(_grace - delta, 0.0)
 
	# A wheel "touches" when its suspension pushes. Count the time since that last happened.
	var on_ground := front_force > 0.0 or rear_force > 0.0
	if on_ground:
		_since_ground = 0.0
	else:
		_since_ground += delta
 
	if Input.is_action_just_pressed("respawn"):
		respawn()
		return
 
	match state:
		State.GROUNDED:
			if _since_ground > air_delay:
				# Both wheels off the ground for a moment: this is a jump (not just a bump).
				state = State.AIRBORNE
				air_time = 0.0
			elif tilt_angle() > crash_tilt and _grace <= 0.0:
				start_crash()
			elif front_force > 0.0 and rear_force > 0.0 and tilt_angle() < 0.3:
				# Riding normally on both wheels: remember this spot for the respawn.
				_safe_position = global_position
				var f := -global_basis.z
				_safe_yaw = atan2(-f.x, -f.z)
 
		State.AIRBORNE:
			air_time += delta
			if on_ground:
				state = State.GROUNDED
				var quality := 1.0 - clampf(tilt_angle() / crash_tilt, 0.0, 1.0)
				if air_time > 0.25:
					print("Landed: quality %.2f after %.2f s in the air" % [quality, air_time])
					landed.emit(quality, air_time)
				if tilt_angle() > crash_tilt and _grace <= 0.0:
					start_crash()
 
		State.CRASHED:
			_crash_timer -= delta
			if _crash_timer <= 0.0:
				respawn()
 

func tilt_angle() -> float:
	var normal := ground_normal()
	if normal == Vector3.ZERO:
		return 0.0
	return global_basis.y.angle_to(normal)
 
 
func start_crash() -> void:
	state = State.CRASHED
	_crash_timer = crash_time
	print("CRASH")
	crashed.emit()
 
 
func _on_body_entered(_body: Node) -> void:
	if state != State.CRASHED and _grace <= 0.0:
		start_crash()
 
func respawn() -> void:
	var t := Transform3D(Basis(Vector3.UP, _safe_yaw), _safe_position + Vector3.UP * respawn_lift)
	var rid := get_rid()
	PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_TRANSFORM, t)
	PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_LINEAR_VELOCITY, Vector3.ZERO)
	PhysicsServer3D.body_set_state(rid, PhysicsServer3D.BODY_STATE_ANGULAR_VELOCITY, Vector3.ZERO)
 
	steer_angle = 0.0
	rider_shift = 0.0
	_jump_charge = 0.0
	_pop_rear_timer = 0.0
	state = State.AIRBORNE      # it drops a little, then lands normally
	air_time = 0.0
	_since_ground = 1.0
	_grace = 1.0                # ignore crashes for a second while it settles
	reset_physics_interpolation()

func air_control() -> void:
	var pitch_in := Input.get_axis("lean_forward", "lean_back")   # +1 = pull the bars back = nose up
	var yaw_in := Input.get_axis("steer_right", "steer_left")     # +1 = turn left
	apply_torque(global_basis.x * pitch_in * air_pitch_torque)     # + torque around "right" = nose up
	apply_torque(global_basis.y * yaw_in * air_yaw_torque)
 
	# The rider keeps the wheels below him: a gentle torque against the roll angle.
	var forward := -global_basis.z
	var roll_error := global_basis.y.cross(Vector3.UP).dot(forward)
	apply_torque(forward * air_roll_stiffness * roll_error)
 
	# Damping stops any rotation from growing forever and caps the rotation speed.
	apply_torque(-angular_velocity * air_damping)
 
func jump_control(delta: float) -> void:
	_jump_cooldown = maxf(_jump_cooldown - delta, 0.0)
 
	# Second half of a pop that is already under way: the rear wheel.
	if _pop_rear_timer > 0.0:
		_pop_rear_timer -= delta
		if _pop_rear_timer <= 0.0:
			apply_impulse(_pop_direction * _pop_rear_strength, rear.global_position - global_position)
 
	# You can jump while on the ground, or just after leaving it (coyote time).
	var can_jump := _since_ground < jump_coyote and _jump_cooldown <= 0.0
 
	if Input.is_action_pressed("jump") and can_jump:
		# PRELOAD: more and more downward push while the button is held.
		_jump_charge = minf(_jump_charge + delta / jump_charge_time, 1.0)
		apply_central_force(-global_basis.y * preload_force * _jump_charge)
 
	elif Input.is_action_just_released("jump") and _jump_charge > 0.0 and can_jump:
		# POP: a tap gives 35 % of the jump, a full preload gives 100 %.
		var strength := lerpf(0.35, 1.0, _jump_charge)
		var normal := ground_normal()
		if normal == Vector3.ZERO:
			normal = global_basis.y
		_pop_direction = normal.lerp(Vector3.UP, 0.5).normalized()   # between "away from the ground" and "straight up"
 
		var total := pop_impulse * strength
		apply_impulse(_pop_direction * total * pop_front_share, front.global_position - global_position)
		_pop_rear_strength = total * (1.0 - pop_front_share)
		_pop_rear_timer = pop_rear_delay
		_jump_cooldown = jump_cooldown
		_jump_charge = 0.0
		jumped.emit(strength)
 
	elif not Input.is_action_pressed("jump"):
		_jump_charge = 0.0   # button released but no jump possible (in the air, cooldown): forget the preload
 
