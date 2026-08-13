#!/usr/bin/env python3
"""Phase 18: generate the timestep-variant world files needed for a proper
convergence sweep (0.5/1/2/4/8 ms), per reviewer backlog item.

Current timestep checks only compare 1ms (shipped default) vs. 4ms, n=5
each, and only for launch -- not a real convergence sweep, and not for
landing at all. This generates the two missing variants (0.5ms, 2ms) plus
reuses 8ms if it doesn't exist yet -- checked first via diff that the
ONLY difference between worlds/ryugu.sdf and
phase4_attitude_revalidation/ryugu_4ms.sdf is <max_step_size>, confirming
this is a safe, minimal pattern to replicate. 1ms reuses the live
worlds/ryugu.sdf directly (not duplicated); 4ms reuses Phase 4's existing
ryugu_4ms.sdf (not regenerated).

Run: python3 generate_timestep_variants.py
"""
import re

SRC = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/worlds/ryugu.sdf"
OUT_DIR = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase18_timestep_convergence_sweep"

VARIANTS = {
    "0p5ms": 0.0005,
    "2ms": 0.002,
    "8ms": 0.008,
}


def main():
    with open(SRC) as f:
        base = f.read()

    assert base.count("<max_step_size>0.001</max_step_size>") == 1, \
        "expected exactly one <max_step_size>0.001</max_step_size> in the source world"

    for tag, step in VARIANTS.items():
        new_sdf = base.replace(
            "<max_step_size>0.001</max_step_size>",
            f"<max_step_size>{step}</max_step_size>")
        assert new_sdf != base
        out_path = f"{OUT_DIR}/ryugu_{tag}.sdf"
        with open(out_path, 'w') as f:
            f.write(new_sdf)
        print(f"wrote {out_path} (max_step_size={step})")

    print("done")


if __name__ == '__main__':
    main()
