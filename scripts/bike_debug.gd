extends MeshInstance3D
## Standalone debug drawer. It follows your player NODE (a RigidBody3D) and does not depend on
## the player's script: no class_name needed, no extra variables in the player script.
##
## Setup: add a MeshInstance3D named "DebugDraw" as a child of the player, attach this script.
## Press F3 in game to hide/show.
##
## Auto-detect: any field left empty/0 below is read from the player by property name, if the
## player has it: front, rear, front_wheel_radius, front_ray_to_hub, rear_wheel_radius,
## rear_ray_to_hub, spring. Fill a field here to override it.
##
## Colours:
##   green line       ray from its start to the ground hit
##   grey line        unused rest of the ray (hit point -> end of the ray)
##   red line         ray that hits nothing (wheel in the air)
##   red cross        contact point with the ground
##   white line       ground normal
##   magenta cross    where the hub REALLY is now (contact point + one wheel radius upward)
##   light-blue cross where the hub is at zero compression (ray start + ray_to_hub)
##   orange line      between the two hubs: its length IS the spring compression
##   yellow line      spring force estimate (spring * compression only, no damping)
##   yellow cross     centre of mass
##   cyan line        velocity
##   green / blue     body up axis / body forward axis (-Z)

@export var body: RigidBody3D
@export var front_ray: RayCast3D
@export var rear_ray: RayCast3D
@export var front_wheel_radius := 0.0
@export var front_ray_to_hub := 0.0
@export var rear_wheel_radius := 0.0
@export var rear_ray_to_hub := 0.0
@export var spring_k := 0.0              # 0 = read "spring" from the player, if it exists
@export var draw_enabled := true
@export var force_scale := 0.002         # metres of line per newton (441 N -> 0.88 m)
@export var velocity_scale := 0.3        # metres of line per m/s

var _mesh := ImmediateMesh.new()


func _ready() -> void:
	if body == null:
		body = get_parent() as RigidBody3D
	if body == null:
		push_error("DebugDraw must be a child of the player RigidBody3D (or assign 'body').")
		return

	top_level = true                  # ignore the parent's transform: we draw in world coordinates
	global_transform = Transform3D.IDENTITY
	physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
	mesh = _mesh

	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.vertex_color_use_as_albedo = true
	mat.no_depth_test = true          # visible through the ground and the bike
	material_override = mat
	cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_F3:
		draw_enabled = not draw_enabled


func _physics_process(_delta: float) -> void:
	_mesh.clear_surfaces()
	if not draw_enabled or body == null:
		return

	var f_ray := front_ray if front_ray else body.get("front") as RayCast3D
	var r_ray := rear_ray if rear_ray else body.get("rear") as RayCast3D
	var k := _value(spring_k, "spring")

	_mesh.surface_begin(Mesh.PRIMITIVE_LINES)

	if f_ray:
		_draw_wheel(f_ray, _value(front_wheel_radius, "front_wheel_radius"),
				_value(front_ray_to_hub, "front_ray_to_hub"), k)
	if r_ray:
		_draw_wheel(r_ray, _value(rear_wheel_radius, "rear_wheel_radius"),
				_value(rear_ray_to_hub, "rear_ray_to_hub"), k)

	var com := body.global_transform * body.center_of_mass
	_cross(com, 0.15, Color.YELLOW)
	_line(com, com + body.linear_velocity * velocity_scale, Color.CYAN)

	var origin := body.global_position
	_line(origin, origin + body.global_basis.y * 0.5, Color.GREEN)
	_line(origin, origin - body.global_basis.z * 0.5, Color.BLUE)

	_mesh.surface_end()


## Returns `own` if it was filled here, otherwise the player's property `prop_name` (or 0).
func _value(own: float, prop_name: String) -> float:
	if own != 0.0:
		return own
	var v = body.get(prop_name)
	return v if v is float else 0.0


func _draw_wheel(ray: RayCast3D, wheel_radius: float, ray_to_hub: float, k: float) -> void:
	var start := ray.global_position
	var along := ray.global_basis * ray.target_position      # vector from the start to the end of the ray
	var end := start + along
	var down := along.normalized()
	var hub_rest := start + down * ray_to_hub                # hub at zero compression

	if not ray.is_colliding():
		_line(start, end, Color(1.0, 0.2, 0.2))
		_cross(hub_rest, 0.1, Color(0.4, 0.8, 1.0))
		return

	var hit := ray.get_collision_point()
	var distance := start.distance_to(hit)
	var hub := hit - down * wheel_radius                     # the hub is one radius above the contact
	var compression := ray_to_hub - (distance - wheel_radius)

	_line(start, hit, Color.GREEN)
	_line(hit, end, Color(0.4, 0.4, 0.4))
	_cross(hit, 0.08, Color.RED)
	_line(hit, hit + ray.get_collision_normal() * 0.5, Color.WHITE)
	_cross(hub, 0.1, Color.MAGENTA)
	_cross(hub_rest, 0.06, Color(0.4, 0.8, 1.0))

	if compression > 0.0:
		_line(hub_rest, hub, Color.ORANGE)                   # length of this segment = compression
		if k > 0.0:
			_line(hub, hub - down * k * compression * force_scale, Color.YELLOW)


func _line(a: Vector3, b: Vector3, color: Color) -> void:
	_mesh.surface_set_color(color)
	_mesh.surface_add_vertex(a)
	_mesh.surface_set_color(color)
	_mesh.surface_add_vertex(b)


func _cross(p: Vector3, size: float, color: Color) -> void:
	_line(p - Vector3(size, 0.0, 0.0), p + Vector3(size, 0.0, 0.0), color)
	_line(p - Vector3(0.0, size, 0.0), p + Vector3(0.0, size, 0.0), color)
	_line(p - Vector3(0.0, 0.0, size), p + Vector3(0.0, 0.0, size), color)
