class_name BikeVisuals
extends Node3D
## Makes the bike MODEL follow the PHYSICS of the Player.
##   - front wheel + lower fork slide up the (tilted) fork with the suspension compression
##   - the whole rear end (swingarm + wheel) ROTATES around SwingarmPivot so the hub rises by the compression
##   - the shock shaft slides into the shock body
##   - wheels spin, the handlebar turns, the cranks turn and the pedals stay level
##
## Attach to the ROOT node of bike_model.tscn. The model must be below the Player node, facing -Z.
## Parts are found by NAME: SteeringPivot, FrontWheelPivot, FrontWheel, SwingarmPivot, RearWheelPivot,
## RearWheel, CrankPivot, LeftPedal, RightPedal, ShockBody, ShockShaftPivot.
##
## ShockBody's origin must be at the UPPER eyelet and ShockShaftPivot's origin at the LOWER eyelet:
## the shaft slides along the line between them.

@export var player: Player

@export_group("Travel")
@export var front_travel := 0.2     # fork travel (m): the wheel never moves more than this
@export var rear_travel := 0.2      # rear wheel travel (m)
@export var shock_stroke := 0.07    # how far the shaft slides into the body at full rear travel (m)

@export_group("Other")
@export var gear_ratio := 2.5       # wheel turns per crank turn
@export var compress_smoothing := 40.0  # how fast the visual follows when the suspension compresses (1/s)
@export var extend_smoothing := 15.0    # how fast it follows when the suspension extends (1/s)
@export var max_sink := 0.008           # when compressing, the wheel may lag at most this far (m) behind the ground

@export_group("Calibration")
@export var print_report := true    # prints measured values at start + how far each wheel overlaps the ground
@export var auto_calibrate := false # true = copy the measured radius and ray_to_hub into the Player at start
@export var exact_contact := true   # true = place each wheel from the live ray hit + the MEASURED model (never sinks)

var _steering_pivot: Node3D
var _front_pivot: Node3D
var _front_wheel: Node3D
var _swingarm_pivot: Node3D
var _rear_wheel: Node3D
var _crank: Node3D
var _left_pedal: Node3D
var _right_pedal: Node3D
var _shock_body: Node3D
var _shock_pivot: Node3D

# Rest pose of every part (in its parent's space) and its left-right axis (in its own space).
var _rest_basis: Dictionary = {}
var _rest_pos: Dictionary = {}
var _right_axis: Dictionary = {}

var _steer_axis := Vector3.UP
var _fork_cos := 1.0
var _front_dir := Vector3.UP
var _rear_dir := Vector3.UP
var _shock_dir := Vector3.UP

# Swingarm geometry in Player space: hub position relative to the main pivot.
var _arm_y := 0.0
var _arm_z := 0.0
var _arm_len := 0.0
var _arm_alpha := 0.0

var _front_c := 0.0
var _rear_c := 0.0
var _front_angle := 0.0
var _rear_angle := 0.0
var _crank_angle := 0.0

var _front_mesh_radius := 0.0
var _front_a := 0.0          # measured: distance from the front ray start down to the front hub (rest pose)
var _rear_a := 0.0
var _rear_mesh_radius := 0.0
var _report_timer := 1.0


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
	_swingarm_pivot = find_child("SwingarmPivot", true, false) as Node3D
	_rear_wheel = find_child("RearWheel", true, false) as Node3D
	_crank = find_child("CrankPivot", true, false) as Node3D
	_left_pedal = find_child("LeftPedal", true, false) as Node3D
	_right_pedal = find_child("RightPedal", true, false) as Node3D
	_shock_body = find_child("ShockBody", true, false) as Node3D
	_shock_pivot = find_child("ShockShaftPivot", true, false) as Node3D

	var parts := {
		"SteeringPivot": _steering_pivot, "FrontWheelPivot": _front_pivot, "FrontWheel": _front_wheel,
		"SwingarmPivot": _swingarm_pivot, "RearWheel": _rear_wheel, "CrankPivot": _crank,
		"LeftPedal": _left_pedal, "RightPedal": _right_pedal,
		"ShockBody": _shock_body, "ShockShaftPivot": _shock_pivot,
	}
	for part_name in parts:
		if parts[part_name] == null:
			push_warning("BikeVisuals: part not found in the model: " + part_name)

	# "to_local" converts world transforms into the Player's space (the model faces -Z in that space).
	var to_local := player.global_transform.affine_inverse()
	for part in parts.values():
		if part == null:
			continue
		var rel: Transform3D = to_local * part.global_transform
		_rest_basis[part] = part.basis
		_rest_pos[part] = part.position
		_right_axis[part] = (rel.basis.orthonormalized().inverse() * Vector3.RIGHT).normalized()

	# Fork: the steering pivot's own axis closest to vertical is the head tube.
	if _steering_pivot:
		var basis_s := (to_local * _steering_pivot.global_transform).basis.orthonormalized()
		_steer_axis = _snap_to_axis(basis_s.inverse() * Vector3.UP)
		var fork_dir := basis_s * _steer_axis
		_fork_cos = maxf(fork_dir.y, 0.5)
		if _front_pivot:
			var front_parent := _front_pivot.get_parent() as Node3D
			var basis_fp := (to_local * front_parent.global_transform).basis.orthonormalized()
			_front_dir = basis_fp.inverse() * fork_dir

	# Swingarm: where the rear hub is relative to the main pivot (in Player space).
	if _swingarm_pivot:
		var arm_parent := _swingarm_pivot.get_parent() as Node3D
		var basis_ap := (to_local * arm_parent.global_transform).basis.orthonormalized()
		_rear_dir = basis_ap.inverse() * Vector3.UP
		if _rear_wheel:
			var pivot_pos := (to_local * _swingarm_pivot.global_transform).origin
			var hub_pos := (to_local * _rear_wheel.global_transform).origin
			var r := hub_pos - pivot_pos
			_arm_y = r.y
			_arm_z = r.z
			_arm_len = Vector2(r.y, r.z).length()
			_arm_alpha = atan2(absf(r.z), r.y)

	# Shock: the shaft slides from its own origin (lower eyelet) toward ShockBody's origin (upper eyelet).
	if _shock_pivot and _shock_body:
		var shock_parent := _shock_pivot.get_parent() as Node3D
		var body_in_parent := shock_parent.to_local(_shock_body.global_position)
		_shock_dir = (body_in_parent - _shock_pivot.position).normalized()

	_calibrate(to_local)


func _physics_process(delta: float) -> void:
	# 1. Suspension compression: what the ground asks for, then smoothed so the fork and shock glide
	#    instead of jumping. While compressing, the wheel may lag at most max_sink behind the ground.
	var kc := 1.0 - exp(-compress_smoothing * delta)
	var ke := 1.0 - exp(-extend_smoothing * delta)
	var target_f := _target_compression(player.front, _front_a, _front_mesh_radius, player.front_compression, front_travel)
	var target_r := _target_compression(player.rear, _rear_a, _rear_mesh_radius, player.rear_compression, rear_travel)
	_front_c = _follow(_front_c, target_f, kc, ke)
	_rear_c = _follow(_rear_c, target_r, kc, ke)

	# 2. Front: the wheel and lower legs slide up the tilted fork.
	if _front_pivot:
		_front_pivot.position = (_rest_pos[_front_pivot] as Vector3) + _front_dir * (_front_c / _fork_cos)

	# 3. Rear: the swingarm rotates around the main pivot so that the hub rises by exactly _rear_c.
	if _swingarm_pivot:
		if _arm_len > 0.15 and absf(_arm_z) > 0.02:
			var c := clampf((_arm_y + _rear_c) / _arm_len, -1.0, 1.0)
			var theta := (acos(c) - _arm_alpha) * signf(_arm_z)
			_swingarm_pivot.basis = (_rest_basis[_swingarm_pivot] as Basis) * Basis(_right_axis[_swingarm_pivot] as Vector3, theta)
		else:
			# Pivot too close to the hub for a rotation: just lift the whole rear end.
			_swingarm_pivot.position = (_rest_pos[_swingarm_pivot] as Vector3) + _rear_dir * _rear_c

	# 4. Shock: the shaft slides into the body in proportion to the rear compression.
	if _shock_pivot:
		var slide := (_rear_c / maxf(rear_travel, 0.01)) * shock_stroke
		_shock_pivot.position = (_rest_pos[_shock_pivot] as Vector3) + _shock_dir * slide

	# 5. Wheels spin with the forward speed (angle = distance travelled / radius).
	var forward_speed := player.linear_velocity.dot(-player.global_basis.z)
	_front_angle = fposmod(_front_angle + forward_speed / maxf(player.front_wheel_radius, 0.05) * delta, TAU)
	_rear_angle = fposmod(_rear_angle + forward_speed / maxf(player.rear_wheel_radius, 0.05) * delta, TAU)
	_spin(_front_wheel, _front_angle)
	_spin(_rear_wheel, _rear_angle)

	# 6. Handlebar: turns with the steer angle (positive = left), around the head-tube axis.
	if _steering_pivot:
		_steering_pivot.basis = (_rest_basis[_steering_pivot] as Basis) * Basis(_steer_axis, player.steer_angle)

	# 7. Cranks turn while pedalling; the pedals turn the opposite way so they stay level.
	if Input.is_action_pressed("pedal") and player.state == Player.State.GROUNDED:
		_crank_angle = fposmod(_crank_angle + forward_speed / maxf(player.rear_wheel_radius, 0.05) / gear_ratio * delta, TAU)
	_spin(_crank, _crank_angle)
	_spin(_left_pedal, -_crank_angle)
	_spin(_right_pedal, -_crank_angle)

	if print_report:
		_report_timer -= delta
		if _report_timer <= 0.0:
			_report_timer = 1.0
			print("[BikeVisuals] wheel overlap with the ground (+ = sunk in, - = floating): front %s | rear %s | compression front %.1f cm, rear %.1f cm" % [
				_overlap(_front_wheel, player.front, _front_mesh_radius),
				_overlap(_rear_wheel, player.rear, _rear_mesh_radius),
				_front_c * 100.0, _rear_c * 100.0])


## Moves `current` toward `target`: quickly when compressing (but never lagging more than max_sink),
## slowly when extending.
func _follow(current: float, target: float, k_compress: float, k_extend: float) -> float:
	if target > current:
		return maxf(lerpf(current, target, k_compress), target - max_sink)
	return lerpf(current, target, k_extend)


## How far the wheel must rise (along the ray) so that the TIRE BOTTOM touches the ground hit point.
## With exact_contact it uses the live ray and the measured model, so it does not depend on the numbers
## typed into the Player. Otherwise it uses the compression computed by the Player's suspension.
func _target_compression(ray: RayCast3D, hub_a: float, radius: float, fallback: float, travel: float) -> float:
	if exact_contact and ray != null and radius > 0.0 and hub_a > 0.0:
		if not ray.is_colliding():
			return 0.0
		var distance := ray.global_position.distance_to(ray.get_collision_point())
		return clampf(hub_a - (distance - radius), 0.0, travel)
	return clampf(fallback, 0.0, travel)


## Measures the model and tells you which wheel_radius and ray_to_hub match it.
func _calibrate(to_local: Transform3D) -> void:
	_front_mesh_radius = _origin_to_bottom(_front_wheel, to_local)
	_rear_mesh_radius = _origin_to_bottom(_rear_wheel, to_local)

	_front_a = _hub_below_ray(_front_wheel, player.front)
	_rear_a = _hub_below_ray(_rear_wheel, player.rear)
	var front_a := _front_a
	var rear_a := _rear_a

	if print_report:
		print("[BikeVisuals] FRONT wheel: tire bottom is %.3f m below the wheel origin (Player.front_wheel_radius = %.3f) | hub is %.3f m below the ray start (Player.front_ray_to_hub = %.3f)" % [
			_front_mesh_radius, player.front_wheel_radius, front_a, player.front_ray_to_hub])
		print("[BikeVisuals] REAR  wheel: tire bottom is %.3f m below the wheel origin (Player.rear_wheel_radius = %.3f) | hub is %.3f m below the ray start (Player.rear_ray_to_hub = %.3f)" % [
			_rear_mesh_radius, player.rear_wheel_radius, rear_a, player.rear_ray_to_hub])

	if auto_calibrate:
		if _front_mesh_radius > 0.0:
			player.front_wheel_radius = _front_mesh_radius
			player.front_ray_to_hub = front_a
		if _rear_mesh_radius > 0.0:
			player.rear_wheel_radius = _rear_mesh_radius
			player.rear_ray_to_hub = rear_a


## Distance from the wheel's ORIGIN (the hub) down to the lowest point of its tire.
## It is measured in the WHEEL's own space, so a tilted fork (which rotates the wheel's bounding box
## relative to the ground) cannot inflate the result. The wheel is a disc: its radius is the largest
## size in the two directions perpendicular to the axle, and the origin may be off-centre.
func _origin_to_bottom(wheel: Node3D, to_local: Transform3D) -> float:
	if wheel == null:
		return 0.0
	var wheel_inv := wheel.global_transform.affine_inverse()
	var meshes := wheel.find_children("*", "MeshInstance3D", true, false)
	if wheel is MeshInstance3D:
		meshes.append(wheel)
	var box := AABB()
	var first := true
	for m in meshes:
		var mi := m as MeshInstance3D
		var b: AABB = wheel_inv * mi.global_transform * mi.get_aabb()
		box = b if first else box.merge(b)
		first = false
	if first:
		return 0.0

	# Axle direction in the wheel's own space (a coordinate axis).
	var axle := _snap_to_axis(_right_axis[wheel] as Vector3)

	# Radius of the disc: the biggest size in the two directions that are not the axle.
	var size := box.size
	var radius := maxf(maxf(size.x * (1.0 - absf(axle.x)), size.y * (1.0 - absf(axle.y))), size.z * (1.0 - absf(axle.z))) * 0.5

	# "Up" seen from the wheel's own space, flattened into the disc plane.
	var up_local := (to_local * wheel.global_transform).basis.orthonormalized().inverse() * Vector3.UP
	up_local -= axle * up_local.dot(axle)
	up_local = up_local.normalized()

	# Where the centre of the tire is relative to the origin (inside the disc plane).
	var centre := box.get_center()
	centre -= axle * centre.dot(axle)

	# Lowest point of the tire = centre moved down by one radius.
	return radius - centre.dot(up_local)


## Distance from the ray's start down to the wheel origin (the hub), measured along the ray.
func _hub_below_ray(wheel: Node3D, ray: RayCast3D) -> float:
	if wheel == null or ray == null:
		return 0.0
	var down := (ray.global_basis * ray.target_position).normalized()
	return (wheel.global_position - ray.global_position).dot(down)


## How far the wheel's lowest point is below the ground hit point (millimetres, as text).
func _overlap(wheel: Node3D, ray: RayCast3D, radius: float) -> String:
	if wheel == null or ray == null or not ray.is_colliding():
		return "air"
	var down := (ray.global_basis * ray.target_position).normalized()
	var bottom := wheel.global_position + down * radius
	return "%.1f mm" % ((bottom - ray.get_collision_point()).dot(down) * 1000.0)


## Rotates a part around the bike's left-right axis. Rolling forward = top of the wheel moves forward.
func _spin(part: Node3D, angle: float) -> void:
	if part:
		part.basis = (_rest_basis[part] as Basis) * Basis(_right_axis[part] as Vector3, -angle)


func _snap_to_axis(v: Vector3) -> Vector3:
	var a := v.abs()
	if a.x >= a.y and a.x >= a.z:
		return Vector3(signf(v.x), 0.0, 0.0)
	if a.y >= a.z:
		return Vector3(0.0, signf(v.y), 0.0)
	return Vector3(0.0, 0.0, signf(v.z))
