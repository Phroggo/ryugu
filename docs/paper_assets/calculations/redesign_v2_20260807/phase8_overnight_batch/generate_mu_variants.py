#!/usr/bin/env python3
"""Phase 8, Priority 3: generate foot-friction (mu) model variants for the
sweep. mu=0.62 is the live baseline (models/spacehopper/model.sdf, foot
collisions on all 3 legs) -- reused as-is, not duplicated. Generates
separate named model packages (spacehopper_mu040/050/075/090) for the
other 4 sweep points, each a full copy of spacehopper with only <mu>/<mu2>
in the 3 foot_collision surface blocks changed, so each can be spawned
independently via model://spacehopper_muXXX (GZ_SIM_RESOURCE_PATH already
covers models/).
"""
import os, re, shutil

SRC = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/models/spacehopper"
# Kept out of the real models/ tree (matches the project's established
# convention for experiment-specific variants, e.g. Phase 0/4's
# ryugu_4ms.sdf) -- these variants live in the phase8 experiment folder
# and are resolved via an additional GZ_SIM_RESOURCE_PATH entry, not by
# polluting the shipped models/ directory.
MODELS_DIR = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase8_overnight_batch/variant_models"
MU_VALUES = [0.40, 0.50, 0.75, 0.90]  # 0.62 baseline reused, not regenerated


def main():
    with open(f"{SRC}/model.sdf") as f:
        base_sdf = f.read()

    n_mu = base_sdf.count("<mu>0.62</mu>")
    n_mu2 = base_sdf.count("<mu2>0.62</mu2>")
    print(f"baseline model.sdf: {n_mu} <mu>0.62</mu>, {n_mu2} <mu2>0.62</mu2> occurrences")
    assert n_mu == 3 and n_mu2 == 3, "expected exactly 3 foot collisions (one per leg)"

    for mu in MU_VALUES:
        tag = f"spacehopper_mu{int(round(mu*100)):03d}"
        dst = f"{MODELS_DIR}/{tag}"
        os.makedirs(dst, exist_ok=True)

        new_sdf = base_sdf.replace("<mu>0.62</mu>", f"<mu>{mu}</mu>")
        new_sdf = new_sdf.replace("<mu2>0.62</mu2>", f"<mu2>{mu}</mu2>")
        assert new_sdf.count(f"<mu>{mu}</mu>") == 3
        assert new_sdf.count(f"<mu2>{mu}</mu2>") == 3
        assert "0.62" not in new_sdf, "leftover baseline mu value found post-substitution"

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

        print(f"  wrote {tag} (mu={mu})")

    print("done")


if __name__ == '__main__':
    main()
