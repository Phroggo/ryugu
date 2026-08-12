#!/usr/bin/env python3
"""Phase 12 (scoped #1b): generate base_link inertia-tensor perturbation
variants for the mass-sensitivity analysis.

Reviewer critique: the lumped base_link (1.3839 kg, everything but
reaction wheels/legs/drill folded into one rigid body) is the top-cited
threat to validity for attitude-control/self-righting/landing-impact
claims. Scoped answer (not a full distributed-mass remodel): perturb
I_bot by +-20% and rerun self-righting reliability at the perturbed
values, to check whether the qualitative conclusions (recovery rate per
bucket) are robust to the lumped-model's inertia uncertainty.

base_link's <inertial> block (models/spacehopper/model.sdf, lines 12-18):
  mass 1.3839, COM (0.00243, 0.0, -0.00703),
  ixx=0.009844 ixy=-0.000090 ixz=-0.000123
  iyy=0.010118 iyz=-0.000008 izz=0.007611
Mass and COM are NOT perturbed here (out of this scoped run -- COM shift
was part of the original, unscoped item 1, not this greenlit scope).
All six inertia-tensor components are scaled UNIFORMLY by 1.20 and 0.80
-- this preserves the tensor's principal-axis directions and eigenvalue
ratios exactly, giving a clean +-20% perturbation of the overall
rotational inertia magnitude rather than distorting its shape.

Same pattern as phase8_overnight_batch/generate_mu_variants.py: full
copy of the spacehopper model package (model.sdf/model.config/meshes),
kept out of the real models/ tree, resolved via an extra
GZ_SIM_RESOURCE_PATH entry.

Run: python3 generate_ibot_variants.py
"""
import os, re, shutil

SRC = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/models/spacehopper"
MODELS_DIR = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase12_mass_sensitivity_scoped/variant_models"

BASE_INERTIA = {
    "ixx": 0.009844, "ixy": -0.000090, "ixz": -0.000123,
    "iyy": 0.010118, "iyz": -0.000008, "izz": 0.007611,
}
SCALE_FACTORS = {"ibot_p20": 1.20, "ibot_m20": 0.80}


def main():
    with open(f"{SRC}/model.sdf") as f:
        base_sdf = f.read()

    base_block = (
        "<inertia>\n"
        "          <ixx>0.009844</ixx><ixy>-0.000090</ixy><ixz>-0.000123</ixz>\n"
        "          <iyy>0.010118</iyy><iyz>-0.000008</iyz>\n"
        "          <izz>0.007611</izz>"
    )
    assert base_sdf.count(base_block) == 1, \
        "expected exactly one base_link <inertia> block matching the known baseline text"

    for tag, scale in SCALE_FACTORS.items():
        scaled = {k: v * scale for k, v in BASE_INERTIA.items()}
        new_block = (
            "<inertia>\n"
            f"          <ixx>{scaled['ixx']:.6f}</ixx><ixy>{scaled['ixy']:.6f}</ixy><ixz>{scaled['ixz']:.6f}</ixz>\n"
            f"          <iyy>{scaled['iyy']:.6f}</iyy><iyz>{scaled['iyz']:.6f}</iyz>\n"
            f"          <izz>{scaled['izz']:.6f}</izz>"
        )
        new_sdf = base_sdf.replace(base_block, new_block)
        assert new_sdf != base_sdf, f"substitution had no effect for {tag}"
        assert new_sdf.count(new_block) == 1

        model_name = f"spacehopper_{tag}"
        dst = f"{MODELS_DIR}/{model_name}"
        os.makedirs(dst, exist_ok=True)

        with open(f"{dst}/model.sdf", 'w') as f:
            f.write(new_sdf)

        with open(f"{SRC}/model.config") as f:
            cfg = f.read()
        cfg = cfg.replace("<name>spacehopper</name>", f"<name>{model_name}</name>", 1)
        with open(f"{dst}/model.config", 'w') as f:
            f.write(cfg)

        meshes_dst = f"{dst}/meshes"
        if os.path.exists(meshes_dst):
            shutil.rmtree(meshes_dst)
        shutil.copytree(f"{SRC}/meshes", meshes_dst)

        print(f"wrote {model_name} (scale={scale}): "
              f"ixx={scaled['ixx']:.6f} iyy={scaled['iyy']:.6f} izz={scaled['izz']:.6f} "
              f"ixy={scaled['ixy']:.6f} ixz={scaled['ixz']:.6f} iyz={scaled['iyz']:.6f}")

    print("done")


if __name__ == '__main__':
    main()
