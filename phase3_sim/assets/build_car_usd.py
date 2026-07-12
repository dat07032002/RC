#!/usr/bin/env python3
"""
Builds roboracer_car.usd — an Ackermann articulation matching the real car
(Traxxas Fiesta ST Rally VXL, 1/10, AWD) as far as Phase 1 has measured it.

Structure:
  chassis (rigid box)
   ├─ steer_FL/FR: revolute Z, -19.44/+24.93 deg, position drive
   │   └─ wheel_FL/FR: revolute Y, velocity drive        (AWD front)
   └─ wheel_RL/RR: revolute Y, velocity drive            (AWD rear)

MEASURED (Phase 1): wheelbase 0.33, track 0.24, total mass ~3.6 kg,
steer right/left = -19.44/+24.93 deg.
PLACEHOLDERS (update when measured — marked [P]):
  wheel radius/width, chassis dims, mass split, tire friction.

Run (server):
  export OMNI_KIT_ACCEPT_EULA=YES
  python build_car_usd.py --headless --out ~/roboracer_project/phase3_sim/assets/roboracer_car.usd
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--out", type=str, default="roboracer_car.usd")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

# ---- parameters -----------------------------------------------------------
WHEELBASE = 0.33          # measured
TRACK = 0.24              # measured
STEER_LEFT_LIMIT_DEG = 24.93   # measured from 0.710 m full-lock radius
STEER_RIGHT_LIMIT_DEG = 19.44  # measured from 0.935 m full-lock radius
MASS_TOTAL = 3.6          # measured estimate

WHEEL_R = 0.045           # measured on-car 2026-07-11 (loaded rally tire w/ foam)
WHEEL_W = 0.028           # researched (Traxxas rally tire section width ~28 mm)
CHASSIS = (0.35, 0.16, 0.09)   # inertial mass-envelope proxy (NOT body shell dims)
M_WHEEL = 0.12            # [P] per-wheel mass (kg)
M_KNUCKLE = 0.05          # [P]
M_CHASSIS = MASS_TOTAL - 4 * M_WHEEL - 2 * M_KNUCKLE  # 3.02 kg
TIRE_FRICTION = 1.0       # [P] until coast-down / slide test

FRONT_X = WHEELBASE / 2.0     # chassis origin midway between axles
REAR_X = -WHEELBASE / 2.0
HALF_TRACK = TRACK / 2.0
Z = WHEEL_R                   # body frame height so wheels touch ground


def make_stage(path: str) -> Usd.Stage:
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    return stage


def rigid_box(stage, path, size, pos, mass, color):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
    geom = UsdGeom.Cube.Define(stage, path + "/geom")
    geom.CreateSizeAttr(1.0)
    geom.AddScaleOp().Set(Gf.Vec3f(*size))
    geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return prim


def rigid_wheel(stage, path, pos, mass, color=(0.12, 0.12, 0.12)):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(*pos))
    # visual: cylinder tire
    geom = UsdGeom.Cylinder.Define(stage, path + "/geom")
    geom.CreateAxisAttr("Y")
    geom.CreateRadiusAttr(WHEEL_R)
    geom.CreateHeightAttr(WHEEL_W)
    geom.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    # collision: sphere proxy — thin-cylinder rim contact is numerically weak
    # (near-zero lateral grip); a sphere gives a stable contact patch
    col = UsdGeom.Sphere.Define(stage, path + "/collision")
    col.CreateRadiusAttr(WHEEL_R)
    col.CreatePurposeAttr(UsdGeom.Tokens.guide)  # invisible
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
    return prim


def revolute(stage, path, body0, body1, axis, anchor0, anchor1=(0, 0, 0),
             limits_deg=None, drive=None):
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(Gf.Vec3f(*anchor0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*anchor1))
    if limits_deg is not None:
        j.CreateLowerLimitAttr(limits_deg[0])
        j.CreateUpperLimitAttr(limits_deg[1])
    if drive is not None:
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(drive.get("stiffness", 0.0))
        d.CreateDampingAttr(drive.get("damping", 0.0))
        d.CreateMaxForceAttr(drive.get("max_force", 1e6))
    return j


def main():
    stage = make_stage(args_cli.out)
    car = UsdGeom.Xform.Define(stage, "/car")
    stage.SetDefaultPrim(car.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(car.GetPrim())
    px = PhysxSchema.PhysxArticulationAPI.Apply(car.GetPrim())
    px.CreateSolverPositionIterationCountAttr(16)  # tire contact needs iterations
    px.CreateSolverVelocityIterationCountAttr(4)

    # tire physics material (friction is the sim-to-real lever here)
    mat = UsdShade.Material.Define(stage, "/car/tire_material")
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(TIRE_FRICTION)
    pm.CreateDynamicFrictionAttr(TIRE_FRICTION * 0.9)

    chassis = rigid_box(stage, "/car/chassis", CHASSIS, (0, 0, Z + 0.02),
                        M_CHASSIS, (0.75, 0.17, 0.10))

    wheels = []
    # front: knuckle (steering) -> wheel (rolling)
    for side, y in (("FL", HALF_TRACK), ("FR", -HALF_TRACK)):
        kpath = f"/car/knuckle_{side}"
        rigid_box(stage, kpath, (0.03, 0.03, 0.03), (FRONT_X, y, Z),
                  M_KNUCKLE, (0.3, 0.3, 0.3))
        revolute(stage, f"/car/joints/steer_{side}", "/car/chassis", kpath,
                 "Z", (FRONT_X, y, -0.02), (0, 0, 0),
                 limits_deg=(-STEER_RIGHT_LIMIT_DEG, STEER_LEFT_LIMIT_DEG),
                 drive={"stiffness": 40.0, "damping": 2.0, "max_force": 10.0})
        wpath = f"/car/wheel_{side}"
        woff = WHEEL_W / 2 + 0.02
        rigid_wheel(stage, wpath, (FRONT_X, y + (woff if y > 0 else -woff), Z), M_WHEEL)
        revolute(stage, f"/car/joints/wheel_{side}", kpath, wpath,
                 "Y", (0, (woff if y > 0 else -woff), 0), (0, 0, 0),
                 drive={"stiffness": 0.0, "damping": 0.3, "max_force": 0.6})
        wheels.append(wpath)

    # rear: wheel joints straight on the chassis (AWD -> driven too)
    for side, y in (("RL", HALF_TRACK), ("RR", -HALF_TRACK)):
        wpath = f"/car/wheel_{side}"
        rigid_wheel(stage, wpath, (REAR_X, y, Z), M_WHEEL)
        revolute(stage, f"/car/joints/wheel_{side}", "/car/chassis", wpath,
                 "Y", (REAR_X, y, -0.02), (0, 0, 0),
                 drive={"stiffness": 0.0, "damping": 0.3, "max_force": 0.6})
        wheels.append(wpath)

    # bind tire material to wheel collision geoms
    for wpath in wheels:
        geom = stage.GetPrimAtPath(wpath + "/geom")
        UsdShade.MaterialBindingAPI.Apply(geom).Bind(
            mat, materialPurpose="physics")

    stage.Save()
    print(f"[CarBuilder] wrote {args_cli.out}")
    print(f"[CarBuilder] wheelbase={WHEELBASE} track={TRACK} mass={MASS_TOTAL} "
          f"steer=-{STEER_RIGHT_LIMIT_DEG}/+{STEER_LEFT_LIMIT_DEG} deg | "
          f"[P] wheel_r={WHEEL_R} friction={TIRE_FRICTION}")


if __name__ == "__main__":
    main()
    simulation_app.close()
