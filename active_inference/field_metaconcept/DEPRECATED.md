# field_metaconcept — DEPRECATED (2026-11-04)

The field is no longer a separate metaconcept node (`field_0`). The **`field` node is the robot's frame in the open
field and the concept of the place** at the same time, owned by `openfield_concept`:

- plants, rows and plots hang from it (`field —RT→ X`);
- its plots are its members (`field —group_member→ plot_<id>`), written by `openfield_concept`
  (`OpenFieldSceneGraph::group_plots`);
- its inspection state is on the node (`model_stable`, `model_uncertainty`).

The intermediate `world` frame node (2026-11-03) was removed at the same time. This agent is no longer launched
(`utils/cognitive_husky.toml`); its code is kept for reference only. See `active_inference/conceptos.md`.
