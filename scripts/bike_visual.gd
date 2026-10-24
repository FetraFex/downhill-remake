class_name BikeVisuals
extends Node3D
## Makes the bike MODEL follow the PHYSICS of the Player:
##   - front wheel slides up the (tilted) fork, rear wheel moves up, both from the suspension compression
##   - wheels spin with the ground speed
##   - the handlebar turns with the steer angle
##   - the cranks turn while pedalling and the pedals stay level
##
## Attach to the ROOT node of bike_model.tscn (replace bike_model.gd with this script).
## The model must be somewhere below the Player node. Parts are found by NAME, so the names in
## your Blender hierarchy must be: SteeringPivot, FrontWheelPivot, FrontWheel, RearWheelPivot,
## RearWheel, CrankPivot, LeftPedal, RightPedal.
##
## You don't have to tell the script which axis is which: it works them out from the model itself
## (the wheel axle is the bike's left-right direction, the steering axis is the pivot axis closest
## to vertical). It assumes the model faces -Z (the front of the bike points to the Player's -Z).

@export var player: Player
@export var gear_ratio := 2.5       # wheel turns per crank turn
@export var smoothing := 30.0       # smooths the compression so rough ground doesn't make the fork jitter

var _steering_pivot: Node3D
var _front_pivot: Node3D
var _front_wheel: Node3D
var _rear_pivot: Node3D
var _rear_wheel: Node3D
var _crank: Node3D
var _left_pedal: Node3D
var _right_pedal: Node3D

# Pose of every part at the start (in its own parent's space), and its left-right axis in its own space.
var _rest_basis: Dictionary = {}
var _rest_pos: Dictionary = {}
var _right_axis: Dictionary = {}

var _steer_axis := Vector3.UP   # steering axis, in SteeringPivot's own space
var _fork_cos := 1.0            # cosine of the head-tube tilt (1 = vertical fork)
var _front_dir := Vector3.UP    # fork direction ("up the fork"), in FrontWheelPivot's parent space
var _rear_dir := Vector3.UP     # vertical direction, in RearWheelPivot's parent space

var _front_c := 0.0             # smoothed compressions (m)
var _rear_c := 0.0
var _front_angle := 0.0         # wheel rotation (rad)
var _rear_angle := 0.0
var _crank_angle := 0.0


func _ready() -> void:
	if player == null:
		var n := get_parent()
		while n != null and not (n is Player):
			n = n.get_parent()
		player = n as Player
	if player == null:
		push_error("BikeVisuals: the bike model must be below the Player node.")
		set_physics_process(false)
		return

	_steering_pivot = find_child("SteeringPivot", true, false) as Node3D
	_front_pivot = find_child("FrontWheelPivot", true, false) as Node3D
	_front_wheel = find_child("FrontWheel", true, false) as Node3D
	_rear_pivot = find_child("RearWheelPivot", true, false) as Node3D
	_rear_wheel = find_child("RearWheel", true, false) as Node3D
	_crank = find_child("CrankPivot", true, false) as Node3D
	_left_pedal = find_child("LeftPedal", true, false) as Node3D
	_right_pedal = find_child("RightPedal", true, false) as Node3D

	var parts := {
		"SteeringPivot": _steering_pivot, "FrontWheelPivot": _front_pivot, "FrontWheel": _front_wheel,
		"RearWheelPivot": _rear_pivot, "RearWheel": _rear_wheel, "CrankPivot": _crank,
		"LeftPedal": _left_pedal, "RightPedal": _right_pedal,
	}
	for part_name in parts:
		if parts[part_name] == null:
			push_warning("BikeVisuals: part not found in the model: " + part_name)

	# Remember the rest pose of each part, and which of its own axes is the bike's left-right axis.
	# "to_local" converts world transforms into the Player's space (the model faces -Z in that space).
	var to_local := player.global_transform.affine_inverse()
	for part in parts.values():
		if part == null:
			continue
		var rel: Transform3D = to_local * part.global_transform
		_rest_basis[part] = part.basis
		_rest_pos[part] = part.position
		_right_axis[part] = (rel.basis.orthonormalized().inverse() * Vector3.RIGHT).normalized()

	# Steering axis: the pivot's own axis closest to vertical (that is the head tube).
	# Its tilt tells us how far the wheel travels along the fork for a given vertical compression.
	if _steering_pivot:
		var basis_s := (to_local * _steering_pivot.global_transform).basis.orthonormalized()
		_steer_axis = _snap_to_axis(basis_s.inverse() * Vector3.UP)
		var fork_dir := basis_s * _steer_axis                 # "up the fork", in Player space
		_fork_cos = maxf(fork_dir.y, 0.5)
		if _front_pivot:
			var front_parent := _front_pivot.get_parent() as Node3D
			var basis_fp := (to_local * front_parent.global_transform).basis.orthonormalized()
			_front_dir = basis_fp.inverse() * fork_dir

	if _rear_pivot:
		var rear_parent := _rear_pivot.get_parent() as Node3D
		var basis_rp := (to_local * rear_parent.global_transform).basis.orthonormalized()
		_rear_dir = basis_rp.inverse() * Vector3.UP


func _physics_process(delta: float) -> void:
	# 1. Suspension: the wheels move up when the suspension compresses (smoothed a little).
	var k := 1.0 - exp(-smoothing * delta)
	_front_c = lerpf(_front_c, player.front_compression, k)
	_rear_c = lerpf(_rear_c, player.rear_compression, k)

	if _front_pivot:
		# The hub rises by _front_c vertically; along a tilted fork that is a longer distance.
		_front_pivot.position = (_rest_pos[_front_pivot] as Vector3) + _front_dir * (_front_c / _fork_cos)
	if _rear_pivot:
		_rear_pivot.position = (_rest_pos[_rear_pivot] as Vector3) + _rear_dir * _rear_c

	# 2. Wheels spin with the forward speed (angle = distance travelled / radius).
	var forward_speed := player.linear_velocity.dot(-player.global_basis.z)
	_front_angle = fposmod(_front_angle + forward_speed / maxf(player.front_wheel_radius, 0.05) * delta, TAU)
	_rear_angle = fposmod(_rear_angle + forward_speed / maxf(player.rear_wheel_radius, 0.05) * delta, TAU)
	_spin(_front_wheel, _front_angle)
	_spin(_rear_wheel, _rear_angle)

	# 3. Handlebar: turns with the steer angle (positive = left), around the head-tube axis.
	if _steering_pivot:
		_steering_pivot.basis = (_rest_basis[_steering_pivot] as Basis) * Basis(_steer_axis, player.steer_angle)

	# 4. Cranks turn while pedalling; the pedals turn the opposite way so they stay level.
	if Input.is_action_pressed("pedal") and player.state == Player.State.GROUNDED:
		_crank_angle = fposmod(_crank_angle + forward_speed / maxf(player.rear_wheel_radius, 0.05) / gear_ratio * delta, TAU)
	_spin(_crank, _crank_angle)
	_spin(_left_pedal, -_crank_angle)
	_spin(_right_pedal, -_crank_angle)


## Rotates a part around the bike's left-right axis. Rolling forward = top of the wheel moves forward.
func _spin(part: Node3D, angle: float) -> void:
	if part:
		part.basis = (_rest_basis[part] as Basis) * Basis(_right_axis[part] as Vector3, -angle)


## Returns the unit axis (+-X, +-Y or +-Z) closest to the given direction.
func _snap_to_axis(v: Vector3) -> Vector3:
	var a := v.abs()
	if a.x >= a.y and a.x >= a.z:
		return Vector3(signf(v.x), 0.0, 0.0)
	if a.y >= a.z:
		return Vector3(0.0, signf(v.y), 0.0)
	return Vector3(0.0, 0.0, signf(v.z))
