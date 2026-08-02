# Numerical Policy

Nurbsman separates exact representation checks from tolerance-aware geometric decisions. This prevents a tolerance chosen for one model from silently changing the meaning of knot vectors, indices, or stored data.

## Tolerance Categories

`cad::GeometryTolerance` contains four values:

- Model absolute tolerance: distance in document units used for point coincidence and spatial decisions.
- Parameter absolute tolerance: distance in curve or surface parameter space.
- Angular tolerance: radians used when comparing normalized directions.
- Relative tolerance: scale-dependent allowance for scalar and parameter calculations.

The initial defaults are:

| Category | Default |
|---|---:|
| Model absolute | `1e-9` document units |
| Parameter absolute | `1e-12` parameter units |
| Angular | `1e-9` radians |
| Relative | `1e-12` |

These defaults are kernel defaults, not manufacturing or export accuracy. A future document-level units and accuracy setting may construct a different validated policy.

All absolute tolerances must be finite and greater than zero. Relative tolerance must be finite and in `[0, 1]`. Angular tolerance must be finite, greater than zero, and less than pi/2 radians. Invalid policies are rejected by the factory.

## Comparison Rules

Scalar values are approximately equal when:

```text
abs(a - b) <= absolute + relative * max(abs(a), abs(b))
```

Near-zero tests require an explicit reference scale and use:

```text
abs(value) <= absolute + relative * reference_scale
```

Point coincidence uses Euclidean separation and the model absolute tolerance only. It deliberately does not use coordinate magnitude, because moving identical geometry farther from the origin must not make distinct points become coincident.

Direction comparison normalizes both vectors and computes the angle with `atan2(cross magnitude, dot)`. This remains resolvable for small angles where a cosine threshold would round to one. Zero and non-finite vectors never compare as parallel or equal in direction.

All tolerance boundaries are inclusive. Non-finite inputs never compare equal or near.

## Exact Checks

The following remain exact and must not use geometric tolerance:

- Array sizes, control-net dimensions, degrees, and indices
- Finite-value and positive-weight validation
- Knot ordering and knot multiplicity
- Stored knot identity within a single representation
- Parameter-domain admission during evaluation
- Undo/redo change detection for stored values

Callers that intentionally want to accept a parameter near a domain boundary must perform an explicit tolerance-aware decision and clamp it before evaluation.

## Tolerance-Aware Operations

The following current or planned operations must receive a `GeometryTolerance` instead of embedding epsilon constants:

- Point coincidence and topological merging
- Curve and surface fitting or rebuilding
- Closest-point and intersection convergence
- Trim-loop closure and containment
- Patch gap and continuity analysis
- Sewing and shell validation
- Tessellation chordal and angular error checks

Screen-space hit radii and visual depth tie-breaking are interaction settings, not geometric tolerances.

## Scale And Range

Kernel storage accepts finite `double` coordinates, subject to geometry-specific accumulator checks. The default tolerance is intended for ordinary models whose useful dimensions are roughly `1e-6` to `1e9` document units.

Algorithms must reject non-finite intermediate results. Models near the extremes of `double`, or models combining extremely small details with extremely large coordinates, require rescaling or a deliberately constructed tolerance policy. Rendering currently narrows coordinates to `float`, so its practical range and precision are smaller than the geometry kernel's.
