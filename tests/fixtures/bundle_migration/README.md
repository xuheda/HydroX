# VehicleBundle migration baselines

These 19 fixtures record effective controller/allocator parameters and 1,344
wrench/actuator values per profile from the pre-migration implementation.
The capture used the old loader/factory in an isolated build, not the new Bundle
reader. Source documents were the twelve named OceanX control documents and seven
existing Bundle documents, before their migration to schema 2.0.

`tests/bundle_parity_scenario.h` defines the deterministic, explicitly zero-initialized
state sequence (three modes, 32 updates each, six wrench axes and eight channels).
`test_vehicle_bundle` compares the new implementation against these snapshots.
Do not regenerate the expected output from the implementation under test merely
to make a regression pass. Update a baseline only for an intentional control
change with independently reviewed expected results.

Fields that were runtime-overridden duplicates (controller mass and VTOL lift
allocator limits) remain in this historical snapshot but are not accepted by the
new schema. Their authoritative values are the top-level control model values.
This data is test-only; no old-format runtime loader is retained.

The immutable VTOL snapshots retain the former shared surface gain and negative
per-axis override sentinels. The parity test expands those historical sentinels
to their effective shared gain when comparing parameters; recorded output samples
are unchanged. Runtime Bundles require three explicit nonnegative axis gains and
reject the shared field. Non-fin snapshots also retain the unused pitch inertia;
that field is no longer accepted for non-fin archetypes.
