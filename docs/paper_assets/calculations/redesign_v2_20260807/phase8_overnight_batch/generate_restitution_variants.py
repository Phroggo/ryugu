#!/usr/bin/env python3
"""Phase 8, Priority 3b: generate terrain-restitution model+world variants.
Current live regolith_plane restitution_coefficient is 0.15 (not one of
the requested sweep points 0.1/0.2/0.4), so all 3 are new variants -- no
baseline reuse this time. Only regolith_plane's restitution_coefficient is
varied; the world-boundary walls (irrelevant to a vertical drop test, the
robot never touches them) are left at their existing value.
"""
import os, re, shutil

BASE_DIR = "/home/melvin/ryugu_v2_ws/src/ryugu_sim"
# Kept out of the real models/ and worlds/ trees (matches the project's
# established convention, e.g. Phase 0/4's ryugu_4ms.sdf) -- resolved via
# an additional GZ_SIM_RESOURCE_PATH entry at run time, not by polluting
# the shipped directories.
PHASE8_DIR = f"{BASE_DIR}/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch"
MODELS_SRC_DIR = f"{BASE_DIR}/models"
WORLDS_SRC_DIR = f"{BASE_DIR}/worlds"
MODELS_DIR = f"{PHASE8_DIR}/variant_models"
WORLDS_DIR = f"{PHASE8_DIR}/variant_worlds"
E_VALUES = [0.1, 0.2, 0.4]


def main():
    with open(f"{MODELS_SRC_DIR}/regolith_plane/model.sdf") as f:
        base_model_sdf = f.read()
    with open(f"{MODELS_SRC_DIR}/regolith_plane/model.config") as f:
        base_config = f.read()
    with open(f"{WORLDS_SRC_DIR}/ryugu.sdf") as f:
        base_world = f.read()

    n_e = base_model_sdf.count("<restitution_coefficient>0.15</restitution_coefficient>")
    print(f"baseline regolith_plane/model.sdf: {n_e} restitution_coefficient=0.15 occurrence(s)")
    assert n_e == 1, "expected exactly 1 restitution_coefficient in regolith_plane/model.sdf"
    assert "model://regolith_plane" in base_world

    for e in E_VALUES:
        tag = f"regolith_plane_e{int(round(e*100)):03d}"
        dst = f"{MODELS_DIR}/{tag}"
        os.makedirs(dst, exist_ok=True)

        # BUG FIX (found live): the heightmap collision/visual geometry
        # references relative <uri>s (meshes/ryugu_terrain.*,
        # materials/textures/*.png) resolved against the model's OWN
        # directory -- omitting these on the first attempt gave "Parser
        # configurations requested resolved uris, but uri [...] could not
        # be resolved" / "Failed to load a world" (gz sim exited, no
        # odometry ever available, rest_z stayed None for the full 250s
        # wait). Both subdirectories must be copied alongside model.sdf.
        for subdir in ("meshes", "materials"):
            src_sub = f"{MODELS_SRC_DIR}/regolith_plane/{subdir}"
            dst_sub = f"{dst}/{subdir}"
            if os.path.exists(dst_sub):
                shutil.rmtree(dst_sub)
            shutil.copytree(src_sub, dst_sub)

        new_model_sdf = base_model_sdf.replace(
            "<restitution_coefficient>0.15</restitution_coefficient>",
            f"<restitution_coefficient>{e}</restitution_coefficient>")
        assert f"<restitution_coefficient>{e}</restitution_coefficient>" in new_model_sdf
        with open(f"{dst}/model.sdf", 'w') as f:
            f.write(new_model_sdf)

        new_config = base_config.replace("<name>regolith_plane</name>", f"<name>{tag}</name>", 1)
        with open(f"{dst}/model.config", 'w') as f:
            f.write(new_config)

        world_out = f"{WORLDS_DIR}/ryugu_e{int(round(e*100)):03d}.sdf"
        new_world = base_world.replace(
            "<uri>model://regolith_plane</uri>", f"<uri>model://{tag}</uri>")
        assert f"model://{tag}" in new_world
        with open(world_out, 'w') as f:
            f.write(new_world)

        print(f"  wrote {dst}/model.sdf, {dst}/model.config, {world_out} (e={e})")

    print("done")


if __name__ == '__main__':
    main()
