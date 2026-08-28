#!/usr/bin/env python3
"""
build_anim.py — Phase 2.1 Avatar scene + 5 skeletal animations in Blender 4.4.

Outputs:
  assets/source/avatar/avatar-anim.blend — Blender source file
  assets/source/avatar/avatar.glb        — re-exported glTF binary

Workflow:
  1. Build 9-node 4-wheel procedural platform (matches build-avatar.mjs).
  2. Create 5 Actions, one per animation. Each Action is shared across the
     relevant objects so the glTF exporter produces ONE animation per
     name (turn_left covers all 4 wheels via per-object NLA pushdowns).
  3. Push each Action onto the appropriate NLA track of each affected object.
  4. Export glTF binary with `export_nla_strips=True`, animation_mode="NLA".

Animations (loop):
  1. idle             (3.0 s, 32 fps samples)
     - chassis: translation Y bobs ±1 cm
     - head:    rotation Y scan ±10°
  2. drive_forward    (1.0 s, 16 fps samples)
     - all 4 wheels: rotation X +2π rad/s
  3. drive_backward   (1.0 s, 16 fps samples)
     - all 4 wheels: rotation X −2π rad/s
  4. turn_left        (1.0 s, 16 fps samples)
     - front (fl, fr): +1.5π rad/s
     - rear  (rl, rr): +0.5π rad/s
  5. turn_right       (1.0 s, 16 fps samples)
     - front (fl, fr): −1.5π rad/s
     - rear  (rl, rr): −0.5π rad/s
"""
import os
import sys
import math
import bpy

ROOT = "/home/builder/.hermes/kanban/boards/robbox/workspaces/t_1fa6e505/rob_box_repo"
OUT_BLEND = os.path.join(ROOT, "assets/source/avatar/avatar-anim.blend")
OUT_GLB   = os.path.join(ROOT, "assets/source/avatar/avatar.glb")

ANIM_IDLE_DURATION    = 3.0
ANIM_DRIVE_DURATION   = 1.0
ANIM_TURN_DURATION    = 1.0
IDLE_FPS              = 32
DRIVE_FPS             = 16
TURN_FPS              = 16
ROT_MODE              = "XYZ"

# ─────────────────────────────────────────────────────────────────────────────
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)

# ─────────────────────────────────────────────────────────────────────────────
def make_pbr(name, color, *, metal=0.0, rough=1.0, emissive=None):
    mat = bpy.data.materials.new(name=name)
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Metallic"].default_value = metal
    bsdf.inputs["Roughness"].default_value = rough
    if emissive:
        bsdf.inputs["Emission Color"].default_value = (*emissive, 1.0)
        bsdf.inputs["Emission Strength"].default_value = 1.0
    return mat

# ─────────────────────────────────────────────────────────────────────────────
def build_geometry():
    """Build the 9-node 4-wheel procedural platform (matches build-avatar.mjs).

    IMPORTANT — coordinate convention:
      Blender is Z-up internally, glTF is Y-up. The glTF exporter applies the
      conversion (X, Y, Z)_blender → (X, Z, -Y)_gltf.
      To produce a glTF that matches the original (programmatically-generated
      via @gltf-transform in Y-up directly), we construct the Blender scene
      with the inverse mapping. E.g.:
        glTF chassis (0, 0.25, 0) → Blender location (0, 0, 0.25)
        glTF wheel_fl (-0.27, 0.16, +0.30) → Blender (-0.27, -0.30, 0.16)
        glTF head (0, 0.34, +0.35) → Blender (0, -0.35, 0.34)
    """
    chassis_mat   = make_pbr("chassis",   (0.604, 0.639, 0.678), metal=0.6, rough=0.45)
    wheel_mat     = make_pbr("wheel",     (0.114, 0.125, 0.153), metal=0.2, rough=0.85)
    head_mat      = make_pbr("head",      (0.039, 0.302, 0.302), metal=0.5, rough=0.3,
                              emissive=(0.05, 0.4, 0.45))
    head_edge_mat = make_pbr("head_edge", (0.267, 0.867, 1.0), metal=0.6, rough=0.3,
                              emissive=(0.2, 0.6, 0.7))
    mast_mat      = make_pbr("mast",      (0.165, 0.180, 0.212), metal=0.7, rough=0.5)
    tip_mat       = make_pbr("tip",       (1.0, 0.322, 0.322), metal=0.0, rough=0.4,
                              emissive=(1.0, 0.322, 0.322))

    # helper: glTF -> Blender translation
    def g2b(x, y, z):
        return (x, -z, y)

    # Chassis: Box 0.6×0.18×0.8 — in glTF at (0, 0.25, 0)
    bpy.ops.mesh.primitive_cube_add(size=1)
    chassis = bpy.context.active_object
    chassis.name = "chassis"
    chassis.scale = (0.6, 0.8, 0.18)   # scale.x=0.6 (X), scale.y=0.8 (Z glTF),
                                       # scale.z=0.18 (Y glTF)
    chassis.location = g2b(0, 0.16 + 0.09, 0)   # (0, 0, 0.25)
    chassis.data.materials.append(chassis_mat)

    def add_wheel(name, gl_x, gl_z):
        """Wheel at glTF (gl_x, 0.16, gl_z) — front wheels at +Z, rear at -Z."""
        bl = g2b(gl_x, 0.16, gl_z)
        # Cylinder is created along Z. To lay it on its side (long axis = X),
        # rotate around Z by π/2 — same in both Blender and glTF since X is X.
        bpy.ops.mesh.primitive_cylinder_add(
            radius=0.16, depth=0.12, vertices=16, location=bl)
        w = bpy.context.active_object
        w.name = name
        w.rotation_euler = (0.0, 0.0, math.pi / 2)
        w.data.materials.append(wheel_mat)
        w.parent = chassis
        return w

    wheel_fl = add_wheel("wheel_fl", -0.27, +0.30)
    wheel_fr = add_wheel("wheel_fr",  0.27, +0.30)
    wheel_rl = add_wheel("wheel_rl", -0.27, -0.30)
    wheel_rr = add_wheel("wheel_rr",  0.27, -0.30)

    # Head: Box 0.28×0.18×0.18 — glTF at (0, 0.34, +0.35)
    bpy.ops.mesh.primitive_cube_add(size=1)
    head = bpy.context.active_object
    head.name = "head"
    head.scale = (0.28, 0.18, 0.18)  # scale.x=0.28, scale.y=Z glTF=0.18,
                                     # scale.z=Y glTF=0.18
    head.location = g2b(0, 0.34, 0.35)
    head.data.materials.append(head_mat)
    head.parent = chassis

    # Head edge: Box 0.30×0.04×0.04 — glTF at (0, 0.34, +0.46)
    bpy.ops.mesh.primitive_cube_add(size=1)
    head_edge = bpy.context.active_object
    head_edge.name = "head_edge"
    head_edge.scale = (0.30, 0.04, 0.04)
    head_edge.location = g2b(0, 0.34, 0.46)
    head_edge.data.materials.append(head_edge_mat)
    head_edge.parent = chassis

    # Antenna mast: Cylinder R=0.012, h=0.22 — glTF at (+0.12, 0.43, -0.25)
    bl = g2b(0.12, 0.43, -0.25)
    bpy.ops.mesh.primitive_cylinder_add(
        radius=0.012, depth=0.22, vertices=8, location=bl)
    mast = bpy.context.active_object
    mast.name = "mast"
    # Cylinder along Z (Blender). We want it vertical in glTF (along Y).
    # In Blender, vertical = along Z. So no rotation needed.
    mast.rotation_euler = (0, 0, 0)
    mast.data.materials.append(mast_mat)
    mast.parent = chassis

    # Antenna tip: Sphere R=0.025 — glTF at (+0.12, 0.66, -0.25)
    bl = g2b(0.12, 0.66, -0.25)
    bpy.ops.mesh.primitive_uv_sphere_add(
        radius=0.025, segments=12, ring_count=8, location=bl)
    tip = bpy.context.active_object
    tip.name = "tip"
    tip.data.materials.append(tip_mat)
    tip.parent = chassis

    expected = {"chassis", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr",
                "head", "head_edge", "mast", "tip"}
    actual = {o.name for o in bpy.data.objects}
    missing = expected - actual
    if missing:
        raise RuntimeError(f"Geometry build failed; missing: {missing}")
    return {o.name: o for o in bpy.data.objects}

# ─────────────────────────────────────────────────────────────────────────────
def find_node(name):
    return bpy.data.objects.get(name)

# ─────────────────────────────────────────────────────────────────────────────
def assign_unique_action(obj, name):
    """Create (or fetch) a uniquely-named Action and bind it to obj.

    We use unique internal names (e.g., 'idle_chassis', 'idle_head') so that
    different objects with the same conceptual animation don't share an Action.
    Then we apply NLA pushdown with a uniform strip name so the glTF exporter
    groups them.
    """
    internal = f"{name}__{obj.name}"
    action = bpy.data.actions.get(internal)
    if action is None:
        action = bpy.data.actions.new(name=internal)
    if obj.animation_data is None:
        obj.animation_data_create()
    obj.animation_data.action = action
    return action

def insert_loc(obj, frame, x, y, z):
    obj.location = (x, y, z)
    obj.keyframe_insert(data_path="location", frame=frame)

def insert_rot(obj, frame, rx, ry, rz):
    obj.rotation_mode = ROT_MODE
    obj.rotation_euler = (rx, ry, rz)
    obj.keyframe_insert(data_path="rotation_euler", frame=frame)

# ─────────────────────────────────────────────────────────────────────────────
def keyframe_loc(animation_name, obj, base_pos, axis, amplitude, duration, n_frames):
    """Insert keyframes for sin-bobbing on `axis` of `obj` translation."""
    internal = f"{animation_name}__{obj.name}"
    action = assign_unique_action(obj, internal)
    for i in range(n_frames + 1):
        t = (i / n_frames) * duration
        v = math.sin((t / duration) * 2 * math.pi) * amplitude
        if axis == 0:
            insert_loc(obj, i, base_pos[0] + v, base_pos[1], base_pos[2])
        elif axis == 1:
            insert_loc(obj, i, base_pos[0], base_pos[1] + v, base_pos[2])
        else:
            insert_loc(obj, i, base_pos[0], base_pos[1], base_pos[2] + v)
    action.frame_start = 0
    action.frame_end   = n_frames
    # Tag the action's user-strip name so glTF groups by it
    action["gltf_anim_name"] = animation_name

def keyframe_rot_y(animation_name, obj, base_rot, amplitude_rad, duration, n_frames):
    """Insert keyframes for sin-sweep on Y-axis (yaw) of `obj` rotation."""
    internal = f"{animation_name}__{obj.name}"
    action = assign_unique_action(obj, internal)
    for i in range(n_frames + 1):
        t = (i / n_frames) * duration
        v = math.sin((t / duration) * 2 * math.pi) * amplitude_rad
        insert_rot(obj, i, base_rot[0], base_rot[1] + v, base_rot[2])
    action.frame_start = 0
    action.frame_end   = n_frames
    action["gltf_anim_name"] = animation_name

def keyframe_rot_x(animation_name, obj, base_rot, omega, duration, n_frames):
    """Insert keyframes for constant ω rotation on X-axis of `obj`."""
    internal = f"{animation_name}__{obj.name}"
    action = assign_unique_action(obj, internal)
    for i in range(n_frames + 1):
        t = (i / n_frames) * duration
        insert_rot(obj, i, base_rot[0] + omega * t, base_rot[1], base_rot[2])
    action.frame_start = 0
    action.frame_end   = n_frames
    action["gltf_anim_name"] = animation_name

# ─────────────────────────────────────────────────────────────────────────────
def push_to_nla(obj, animation_name, *, repeat=False, scale=1.0):
    """Convert obj's currently-bound Action into an NLA strip and clear the
    Action binding so multiple animations can be stacked on the same object.

    Each animation gets its OWN NLA track (so they don't overlap on frame 0).
    The strip name is the conceptual animation name — the glTF exporter uses
    that as the resulting Animation's name, grouping all 4 wheel strips
    named e.g. "turn_left" into ONE glTF Animation with 4 channels.
    """
    ad = obj.animation_data
    if ad is None or ad.action is None:
        return
    action = ad.action
    # Each animation needs its own NLA track (otherwise they'd overlap at
    # frame 0). Find or create a track for this animation_name.
    track = None
    for t in obj.animation_data.nla_tracks:
        if t.name == animation_name:
            track = t
            break
    if track is None:
        track = obj.animation_data.nla_tracks.new()
        track.name = animation_name
    strip = track.strips.new(name=animation_name, start=0, action=action)
    strip.repeat = repeat
    # Clear the active Action so subsequent edits go to a NEW Action
    ad.action = None

# ─────────────────────────────────────────────────────────────────────────────
def build_all_animations():
    """Build all 5 animations as Actions and push them to NLA tracks."""
    print("[build_anim] building animations…", file=sys.stderr)

    # ── 1. idle ── chassis bobs, head yaw scans
    chassis = find_node("chassis")
    head    = find_node("head")
    base_chassis = tuple(chassis.location)
    base_head    = tuple(head.rotation_euler)

    keyframe_loc("idle", chassis, base_chassis, axis=1,
                 amplitude=0.01,            # ±1 cm
                 duration=ANIM_IDLE_DURATION,
                 n_frames=IDLE_FPS)
    keyframe_rot_y("idle", head, base_head,
                   amplitude_rad=10 * math.pi / 180,  # ±10°
                   duration=ANIM_IDLE_DURATION,
                   n_frames=IDLE_FPS)
    push_to_nla(chassis, "idle", repeat=True)
    push_to_nla(head,    "idle", repeat=True)

    # ── 2/3/4/5. drive_forward/backward/turn_left/turn_right — wheels rotate
    def wheels_pair(name, omega_front, omega_rear):
        wheel_fl = find_node("wheel_fl")
        wheel_fr = find_node("wheel_fr")
        wheel_rl = find_node("wheel_rl")
        wheel_rr = find_node("wheel_rr")
        base_fl = tuple(wheel_fl.rotation_euler)
        base_fr = tuple(wheel_fr.rotation_euler)
        base_rl = tuple(wheel_rl.rotation_euler)
        base_rr = tuple(wheel_rr.rotation_euler)
        keyframe_rot_x(name, wheel_fl, base_fl, omega_front, ANIM_TURN_DURATION, TURN_FPS)
        keyframe_rot_x(name, wheel_fr, base_fr, omega_front, ANIM_TURN_DURATION, TURN_FPS)
        keyframe_rot_x(name, wheel_rl, base_rl, omega_rear,  ANIM_TURN_DURATION, TURN_FPS)
        keyframe_rot_x(name, wheel_rr, base_rr, omega_rear,  ANIM_TURN_DURATION, TURN_FPS)
        for w in (wheel_fl, wheel_fr, wheel_rl, wheel_rr):
            push_to_nla(w, name, repeat=True)

    # drive_forward: all wheels +2π rad/s
    wheels_pair("drive_forward",  +2*math.pi, +2*math.pi)
    wheels_pair("drive_backward", -2*math.pi, -2*math.pi)
    wheels_pair("turn_left",      +1.5*math.pi, +0.5*math.pi)
    wheels_pair("turn_right",     -1.5*math.pi, -0.5*math.pi)

# ─────────────────────────────────────────────────────────────────────────────
def export_glb(path):
    print(f"[build_anim] exporting glTF → {path}…", file=sys.stderr)
    for o in bpy.data.objects:
        o.select_set(False)
    # When exporting NLA strips, each strip's name becomes the animation name
    # in glTF. So all 4 wheel NLA strips named "turn_left" become ONE glTF
    # Animation called "turn_left" (the exporter groups them).
    bpy.ops.export_scene.gltf(
        filepath=path,
        export_format="GLB",
        export_animations=True,
        export_animation_mode="NLA_TRACKS",   # one clip per NLA track name
        export_nla_strips=True,
        export_optimize_animation_size=False,
        export_bake_animation=False,
        export_morph=False,
        export_skins=False,
        export_cameras=False,
        export_lights=False,
        export_yup=True,
        # NOTE: do NOT use export_apply=True — that bakes parent transforms
        # into child meshes, which double-applies translations (we already
        # authored keyframes in object-local space, so apply would corrupt
        # the animation transforms).
    )

# ─────────────────────────────────────────────────────────────────────────────
def verify(path):
    size = os.path.getsize(path)
    print(f"[build_anim] exported size: {size} bytes ({size/1024:.1f} KB)", file=sys.stderr)
    print("[build_anim] NLA strips per object:", file=sys.stderr)
    for o in bpy.data.objects:
        if not o.animation_data or not o.animation_data.nla_tracks:
            continue
        strips = []
        for track in o.animation_data.nla_tracks:
            for s in track.strips:
                strips.append(f"{s.name}({s.action.frame_start}…{s.action.frame_end})")
        print(f"  - {o.name}: {strips}", file=sys.stderr)

# ─────────────────────────────────────────────────────────────────────────────
def main():
    reset_scene()
    print("[build_anim] building geometry…", file=sys.stderr)
    build_geometry()

    # The glTF exporter assumes scene.frame_rate for duration calculations.
    # We authored at 16 fps for drive/turn and 32 fps for idle; set the
    # scene rate to the lowest common denominator (16) so all animations
    # get their frame count right.
    bpy.context.scene.frame_set(0)
    bpy.context.scene.frame_start = 0
    # Use 24 fps (glTF standard) and re-author frame counts accordingly:
    #   idle             = 3.0s × 24 fps = 72 frames
    #   drive_forward    = 1.0s × 24 fps = 24 frames
    #   drive_backward   = 1.0s × 24 fps = 24 frames
    #   turn_left        = 1.0s × 24 fps = 24 frames
    #   turn_right       = 1.0s × 24 fps = 24 frames
    bpy.context.scene.render.fps = 24
    bpy.context.scene.render.fps_base = 1.0
    bpy.context.scene.frame_end = 72  # max action length

    # Override IDLE_FPS / DRIVE_FPS / TURN_FPS to 24-based counts
    global IDLE_FPS, DRIVE_FPS, TURN_FPS
    IDLE_FPS = 72
    DRIVE_FPS = 24
    TURN_FPS = 24

    build_all_animations()

    print(f"[build_anim] saving .blend → {OUT_BLEND}", file=sys.stderr)
    bpy.ops.wm.save_as_mainfile(filepath=OUT_BLEND)

    export_glb(OUT_GLB)
    verify(OUT_GLB)
    print("[build_anim] DONE", file=sys.stderr)

if __name__ == "__main__":
    main()
