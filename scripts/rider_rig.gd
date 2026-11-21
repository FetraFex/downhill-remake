class_name RiderRig
extends Node3D
## First version of the rider on the bike (Godot 4.7, uses TwoBoneIK3D).
##   - the hips are placed on the seat, the torso leans forward
##   - hands follow the handlebar grips, feet follow the pedals (two-bone IK + pole targets)
## Attach to the ROOT of Rider.tscn (the scene made from the rider .glb). Rider.tscn is a child of the Player,
## next to the bike model. The rider must face -Z like the bike (rotate the Rider instance 180 degrees on Y if not).
##
## You must fill the bone names below (read them in the Skeleton3D inspector). If a name is missing or wrong,
## the script prints the list of real bone names in the Output panel and stops.

@export var player: Player
@export var skeleton: Skeleton3D      # found automatically if empty

@export_group("Bike anchors (names of the empties in the bike model)")
@export var seat_name := "Seat"
@export var grip_l_name := "HandGrip_L"
@export var grip_r_name := "HandGrip_R"
@export var pedal_l_name := "Pedal_L"
@export var pedal_r_name := "Pedal_R"

@export_group("Rider bones (the character's LEFT must match HandGrip_L / Pedal_L)")
@export var hips_bone := ""
@export var spine_bones: PackedStringArray = []   # from the lowest to the highest, for example spine, chest
@export var upper_arm_l := ""
@export var lower_arm_l := ""
@export var hand_l := ""
@export var upper_arm_r := ""
@export var lower_arm_r := ""
@export var hand_r := ""
@export var upper_leg_l := ""
@export var lower_leg_l := ""
@export var foot_l := ""
@export var upper_leg_r := ""
@export var lower_leg_r := ""
@export var foot_r := ""

@export_group("Pose")
@export var hips_offset := Vector3(0.0, 0.12, 0.0)   # from the seat marker up to the hips
@export var torso_lean_deg := 35.0                   # how far the torso leans forward
@export var hand_offset := Vector3.ZERO              # from the grip marker to the wrist, in the MARKER's own axes (it turns with the bars)
@export var foot_offset := Vector3(0.0, 0.10, 0.0)   # from the pedal marker to the ankle, in the MARKER's own axes
@export var knee_pole_forward := 0.4                 # knees point this far forward
@export var elbow_pole_back := 0.3                   # elbows point back and outward
@export var elbow_pole_out := 0.2

@export_group("Orientation of hands and feet (degrees, relative to the grip / pedal marker)")
@export var copy_orientation := true      # hands wrap the grips, feet lie flat on the pedals
@export var hand_l_rotation_deg := Vector3.ZERO
@export var hand_r_rotation_deg := Vector3.ZERO
@export var foot_l_rotation_deg := Vector3.ZERO
@export var foot_r_rotation_deg := Vector3.ZERO
@export var print_report := true

var _chains: Array = []                  # [upper, middle, end] bone names for: arm L, arm R, leg L, leg R
var _anchors: Array[Node3D] = []         # grip L, grip R, pedal L, pedal R
var _targets: Array[Marker3D] = []
var _poles: Array[Marker3D] = []
var _pole_local: Array[Vector3] = []     # pole positions in the Player's space
var _end_bone: Array[int] = []
var _report_timer := 1.0


func _ready() -> void:
	process_physics_priority = 10        # run after the bike has moved this frame

	if player == null:
		var n := get_parent()
		while n != null and not (n is Player):
			n = n.get_parent()
		player = n as Player
	if player == null:
		push_error("RiderRig: the rider must be below the Player node.")
		set_physics_process(false)
		return

	if skeleton == null:
		var found := find_children("*", "Skeleton3D", true, false)
		if found.is_empty():
			push_error("RiderRig: no Skeleton3D found in the rider.")
			set_physics_process(false)
			return
		skeleton = found[0] as Skeleton3D

	# 1. Check every bone name; if one is wrong, show the real names and stop.
	_chains = [
		[upper_arm_l, lower_arm_l, hand_l], [upper_arm_r, lower_arm_r, hand_r],
		[upper_leg_l, lower_leg_l, foot_l], [upper_leg_r, lower_leg_r, foot_r],
	]
	var needed: Array[String] = [hips_bone]
	for c in _chains:
		for bone_name in c:
			needed.append(bone_name)
	for bone_name in spine_bones:
		needed.append(bone_name)
	var all_ok := true
	for bone_name in needed:
		if skeleton.find_bone(bone_name) < 0:
			all_ok = false
			print("[RiderRig] bone not found: '%s'" % bone_name)
	if not all_ok:
		var names := PackedStringArray()
		for b in skeleton.get_bone_count():
			names.append(skeleton.get_bone_name(b))
		print("[RiderRig] The bones that really exist: ", ", ".join(names))
		set_physics_process(false)
		return

	# 2. Find the anchors on the bike.
	var seat := player.find_child(seat_name, true, false) as Node3D
	if seat == null:
		push_error("RiderRig: anchor not found: " + seat_name)
		set_physics_process(false)
		return
	for anchor_name in [grip_l_name, grip_r_name, pedal_l_name, pedal_r_name]:
		var anchor := player.find_child(anchor_name, true, false) as Node3D
		if anchor == null:
			push_error("RiderRig: anchor not found: " + anchor_name)
			set_physics_process(false)
			return
		_anchors.append(anchor)

	# 3. Put the hips on the seat (move the whole rider).
	var hips_idx := skeleton.find_bone(hips_bone)
	var hips_world := skeleton.global_transform * skeleton.get_bone_global_pose(hips_idx).origin
	global_position += (seat.global_position + player.global_basis * hips_offset) - hips_world

	# 4. Lean the torso forward: rotate the spine bones around the bike's left-right axis.
	if not spine_bones.is_empty():
		var axis := (skeleton.global_transform.basis.orthonormalized().inverse() * player.global_basis * Vector3.RIGHT).normalized()
		var step := -deg_to_rad(torso_lean_deg) / spine_bones.size()   # negative = forward
		for bone_name in spine_bones:
			var idx := skeleton.find_bone(bone_name)
			var g := skeleton.get_bone_global_pose(idx)
			g.basis = Basis(axis, step) * g.basis
			skeleton.set_bone_global_pose(idx, g)

	# 5. Pole targets: they decide which way the knees and elbows point. Each starts at the joint's
	#    current position and is moved forward (knees) or back and outward (elbows), fixed to the bike.
	for i in 4:
		var mid_idx := skeleton.find_bone(_chains[i][1])
		var mid_local := player.to_local(skeleton.global_transform * skeleton.get_bone_global_pose(mid_idx).origin)
		var side := signf(mid_local.x) if absf(mid_local.x) > 0.001 else 1.0
		var anchor_side := signf(player.to_local(_anchors[i].global_position).x)
		if anchor_side != 0.0 and anchor_side != side:
			push_warning("RiderRig: limb %d is on the opposite side of its anchor. Check the left/right bone names, and that the rider faces -Z." % i)
		var pole_local := mid_local
		if i < 2:
			pole_local += Vector3(side * elbow_pole_out, 0.0, elbow_pole_back)
		else:
			pole_local += Vector3(side * 0.05, 0.0, -knee_pole_forward)
		_pole_local.append(pole_local)

		var target := Marker3D.new()
		target.name = "IKTarget%d" % i
		add_child(target)
		_targets.append(target)
		var pole := Marker3D.new()
		pole.name = "IKPole%d" % i
		add_child(pole)
		_poles.append(pole)
		_end_bone.append(skeleton.find_bone(_chains[i][2]))
	_update_targets()

	# 6. One TwoBoneIK3D with four settings: left arm, right arm, left leg, right leg.
	var ik := TwoBoneIK3D.new()
	ik.name = "RiderIK"
	skeleton.add_child(ik)
	ik.setting_count = 4
	for i in 4:
		ik.set_root_bone_name(i, _chains[i][0])
		ik.set_middle_bone_name(i, _chains[i][1])
		ik.set_end_bone_name(i, _chains[i][2])
		ik.set_target_node(i, ik.get_path_to(_targets[i]))
		ik.set_pole_node(i, ik.get_path_to(_poles[i]))

	# 7. After the IK has put each wrist/ankle at its target, copy the rotation of the target onto the hand
	#    or foot bone. Modifiers run in child order, so this one must be added AFTER the IK.
	if copy_orientation:
		_make_orientation_modifier()


func _physics_process(delta: float) -> void:
	_update_targets()
	if print_report:
		_report_timer -= delta
		if _report_timer <= 0.0:
			_report_timer = 1.0
			var errors := PackedStringArray()
			for i in 4:
				var end_world := skeleton.global_transform * skeleton.get_bone_global_pose(_end_bone[i]).origin
				errors.append("%.0f" % (end_world.distance_to(_targets[i].global_position) * 1000.0))
			print("[RiderRig] distance from the targets in mm (hand L, hand R, foot L, foot R): ", ", ".join(errors))


## Targets follow the grips and pedals (they move with the steering and the cranks); poles stay fixed to the bike.
## The offset and the rotation are expressed in the anchor's OWN axes, so when the bars turn, the hand turns with them.
func _update_targets() -> void:
	var rotations: Array[Vector3] = [hand_l_rotation_deg, hand_r_rotation_deg, foot_l_rotation_deg, foot_r_rotation_deg]
	for i in 4:
		var anchor_basis := _anchors[i].global_basis
		var offset := hand_offset if i < 2 else foot_offset
		var rot := rotations[i]
		_targets[i].global_position = _anchors[i].global_position + anchor_basis * offset
		_targets[i].global_basis = anchor_basis * Basis.from_euler(Vector3(deg_to_rad(rot.x), deg_to_rad(rot.y), deg_to_rad(rot.z)))
		_poles[i].global_position = player.global_transform * _pole_local[i]


## Adds a CopyTransformModifier3D that copies the ROTATION of each target onto its hand or foot bone.
## The calls are dynamic on purpose: if a method name differs in your Godot version, you get a message
## instead of a broken script, and you can set the same options by hand in the Inspector.
func _make_orientation_modifier() -> void:
	var copy_mod := CopyTransformModifier3D.new()
	copy_mod.name = "RiderCopyOrientation"
	skeleton.add_child(copy_mod)          # added after RiderIK, so it runs after it
	copy_mod.setting_count = 4
	var node_type := ClassDB.class_get_integer_constant("BoneConstraint3D", "REFERENCE_TYPE_NODE")
	for i in 4:
		for call_info in [
				["set_apply_bone_name", _chains[i][2]],
				["set_reference_type", node_type],
				["set_reference_node", copy_mod.get_path_to(_targets[i])],
				["set_copy_position", false],
				["set_copy_rotation", true],
				["set_copy_scale", false],
				["set_relative", false],
				["set_additive", false]]:
			if copy_mod.has_method(call_info[0]):
				copy_mod.call(call_info[0], i, call_info[1])
			else:
				push_warning("RiderRig: CopyTransformModifier3D has no method '%s' in this Godot version: set it in the Inspector." % call_info[0])
