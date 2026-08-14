#!/usr/bin/env python3
"""Phase 19: generate leg-joint damping variant models for a denser Pareto
sweep, per reviewer backlog item.

The existing 3-point damping sweep (c=0.005/0.05/0.15, referenced in the
paper) is PRE-REDESIGN data (commit 89f3331, 2026-07-16) -- it predates
Phase 1-2's mass/inertia rebuild, Phase 3's gain retuning, and Phase 6's
V_GAIN recalibration entirely. This is not just "denser," it's the first
time this sweep has been run against the current shipped model/controller
stack at all. c=0.05 is locked in as the CURRENT shipped value (reused
directly, no variant needed for it).

Six leg-joint damping values swept: 0.005, 0.02, 0.05 (current, reused),
0.08, 0.12, 0.15 -- same overall range as the original 3-point sweep,
with 3 new intermediate points. 0.4 excluded (original notes: "joints
freeze, DO NOT USE" -- a known-bad value, no need to retest).

Modifies ONLY the 6 leg joints' damping (hip_joint_0/1/2, knee_joint_0/1/2)
via two distinguishing string patterns verified unique to hip vs. knee
dynamics blocks -- drill_joint also has <damping>0.05</damping> but with
a different spring_stiffness (0.2 vs. hip's bare form / knee's 0.00028),
confirmed via grep before writing this generator, and is NOT touched.

Run: python3 generate_damping_variants.py
"""
import shutil

SRC = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/models/spacehopper"
OUT_DIR = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase19_damping_pareto_sweep/variant_models"

HIP_PATTERN = "<dynamics><damping>0.05</damping></dynamics>"
KNEE_PATTERN = ("<spring_reference>0</spring_reference><spring_stiffness>0.00028</spring_stiffness>"
                 "<damping>0.05</damping><friction>0.00001</friction>")
DRILL_PATTERN = "<spring_stiffness>0.2</spring_stiffness><damping>0.05</damping>"

DAMPING_VALUES = [0.005, 0.02, 0.08, 0.12, 0.15]  # 0.05 is the current shipped value, reused not duplicated


def main():
    with open(f"{SRC}/model.sdf") as f:
        base_sdf = f.read()

    n_hip = base_sdf.count(HIP_PATTERN)
    n_knee = base_sdf.count(KNEE_PATTERN)
    n_drill = base_sdf.count(DRILL_PATTERN)
    print(f"baseline model.sdf: {n_hip} hip-pattern, {n_knee} knee-pattern, "
          f"{n_drill} drill-pattern occurrences")
    assert n_hip == 3 and n_knee == 3 and n_drill == 1, \
        "expected exactly 3 hip + 3 knee leg-joint damping occurrences, 1 unrelated drill occurrence"

    for c in DAMPING_VALUES:
        tag = f"spacehopper_damp{str(c).replace('.', 'p')}"
        dst = f"{OUT_DIR}/{tag}"
        import os
        os.makedirs(dst, exist_ok=True)

        new_sdf = base_sdf.replace(HIP_PATTERN, f"<dynamics><damping>{c}</damping></dynamics>")
        new_sdf = new_sdf.replace(
            KNEE_PATTERN,
            f"<spring_reference>0</spring_reference><spring_stiffness>0.00028</spring_stiffness>"
            f"<damping>{c}</damping><friction>0.00001</friction>")
        # drill_joint's damping must remain untouched at 0.05
        assert new_sdf.count(DRILL_PATTERN) == 1, f"drill pattern was disturbed for c={c}"
        assert new_sdf.count(f"<dynamics><damping>{c}</damping></dynamics>") == 3
        assert new_sdf.count(f"<damping>{c}</damping><friction>0.00001</friction>") == 3

        with open(f"{dst}/model.sdf", 'w') as f:
            f.write(new_sdf)

        with open(f"{SRC}/model.config") as f:
            cfg = f.read()
        cfg = cfg.replace("<name>spacehopper</name>", f"<name>{tag}</name>", 1)
        with open(f"{dst}/model.config", 'w') as f:
            f.write(cfg)

        meshes_dst = f"{dst}/meshes"
        if os.path.exists(meshes_dst):
            shutil.rmtree(meshes_dst)
        shutil.copytree(f"{SRC}/meshes", meshes_dst)

        print(f"  wrote {tag} (c={c})")

    print("done")


if __name__ == '__main__':
    main()
