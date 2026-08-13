#!/usr/bin/env python3
"""Phase 16: friction-vs-delivered-velocity plot for the paper.

Reviewer backlog item: "denser friction sweep with a velocity-vs-mu
plot... current sweep already shows the ratio is flat, wants this as an
actual plotted curve rather than only a table". Checked before treating
this as new work, per the user's own instruction: Phase 10's
friction_sweep_postfix_results.json (post-timing-fix, n=20/mu, mu=0.40/
0.50/0.62/0.75/0.90) already has per-trial `delivered` (m/s, measured
separation velocity) and `ratio` (delivered/v_req) fields -- this is
purely a plotting task against existing data, no new sim runs needed,
confirming the user's own suspicion.

Plots delivered separation velocity (mean +/- std per mu, individual
trials as scatter) against mu, with v_req marked as a reference line, to
visually confirm the plateau the table already reports numerically.

Run: python3 generate_friction_velocity_plot.py
"""
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SRC = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase10_postfix_full_revalidation/friction_sweep_postfix_results.json"
OUT_PNG = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase16_friction_velocity_plot/friction_velocity_plot.png"
OUT_SVG = "/home/melvin/ryugu_v2_ws/src/ryugu_sim/docs/paper_assets/calculations/redesign_v2_20260807/phase16_friction_velocity_plot/friction_velocity_plot.svg"


def main():
    d = json.load(open(SRC))
    mus = sorted(set(r["mu"] for r in d))
    v_req = d[0]["v_req"]

    means, stds, all_mu_scatter, all_v_scatter = [], [], [], []
    for mu in mus:
        rows = [r for r in d if r["mu"] == mu and r["status"] == "stabilized"]
        vals = [r["delivered"] for r in rows]
        n = len(vals)
        mean_v = sum(vals) / n
        std_v = (sum((v - mean_v) ** 2 for v in vals) / n) ** 0.5 if n > 1 else 0.0
        means.append(mean_v)
        stds.append(std_v)
        all_mu_scatter.extend([mu] * n)
        all_v_scatter.extend(vals)
        print(f"mu={mu}: n={n} mean={mean_v:.5f} std={std_v:.5f} "
              f"ratio={mean_v/v_req:.4f}")

    all_ratio_scatter = [v / v_req for v in all_v_scatter]
    ratio_means = [m / v_req for m in means]
    ratio_stds = [s / v_req for s in stds]

    # Two panels: left is raw delivered velocity zoomed to the data (a
    # v_req reference line at 0.043 m/s would squash the ~0.0093 m/s
    # plateau into an unreadable sliver at that scale, so v_req is called
    # out in the title/caption instead of drawn into this axis); right is
    # the normalized ratio, which is what the paper's table already
    # reports numerically -- this panel is the direct plotted version of
    # that number.
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.scatter(all_mu_scatter, all_v_scatter, alpha=0.35, s=18, color="#4C72B0",
                label="individual trials (n=20/μ)", zorder=2)
    ax1.errorbar(mus, means, yerr=stds, fmt="o-", color="#C44E52", capsize=4,
                 linewidth=2, markersize=7, label="mean ± std", zorder=3)
    ax1.set_xlabel("Foot friction coefficient, μ")
    ax1.set_ylabel("Delivered separation velocity (m/s)")
    ax1.set_title(f"Delivered velocity (v_req={v_req:.4f} m/s, ~4.6x higher --\n"
                   f"see companion ratio panel for normalized view)")
    ax1.legend(loc="best", fontsize=9)
    ax1.grid(alpha=0.25)

    ax2.scatter(all_mu_scatter, all_ratio_scatter, alpha=0.35, s=18, color="#4C72B0",
                label="individual trials (n=20/μ)", zorder=2)
    ax2.errorbar(mus, ratio_means, yerr=ratio_stds, fmt="o-", color="#C44E52", capsize=4,
                 linewidth=2, markersize=7, label="mean ± std", zorder=3)
    ax2.set_xlabel("Foot friction coefficient, μ")
    ax2.set_ylabel("Delivery ratio (delivered / v_req)")
    ax2.set_title("Normalized delivery ratio -- flat across the full μ range")
    ax2.set_ylim(0.0, 0.30)
    ax2.legend(loc="best", fontsize=9)
    ax2.grid(alpha=0.25)

    fig.suptitle("Launch delivery vs. foot friction (post-fix, n=20/μ, 9.0 m target)",
                 fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(OUT_PNG, dpi=200)
    fig.savefig(OUT_SVG)
    print(f"wrote {OUT_PNG} and {OUT_SVG}")


if __name__ == "__main__":
    main()
