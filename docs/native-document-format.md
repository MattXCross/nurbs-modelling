# Nurbsman Native Document Format

This document defines version 1 of the Nurbsman native document format. Native
documents use the `.nurbsman` extension and contain UTF-8 encoded JSON.

The normative JSON Schema is [`native-document-v1.schema.json`](native-document-v1.schema.json).
The rules below supplement constraints that JSON Schema cannot express.

## Compatibility

The root `format` field must be `nurbsman` and `version` must be the integer `1`.
A reader must reject a document with a different format identifier or a version
greater than the newest version it supports. The error must include the file
path, the encountered version, and the newest supported version, for example:

```text
model.nurbsman: unsupported document version 2; this build supports through version 1
```

Version 1 readers reject unknown fields. Adding, removing, or changing fields
therefore requires a new document version rather than relying on a particular
JSON parser's treatment of unknown data. Readers must also reject duplicate
object keys.

## Units And Coordinates

All positions and model-space distances are stored in millimeters. Parameters,
weights, and knot values are dimensionless.

Coordinates use a right-handed Cartesian system with +Y as up. Looking from +Y
toward the origin, +X points right and +Z points toward the viewer. Surface
control points are stored in U-major order: the point at `(u, v)` has linear
index `u * v_count + v`. A regular surface normal follows `dS/du cross dS/dv`.

`units.length` and every field in `coordinates` are required even though version
1 has only one accepted convention. This makes a document's interpretation
explicit and leaves a versioned path for future unit support.

## Number Encoding

Geometry numbers are JSON numbers representing finite IEEE 754 binary64 values.
Writers must emit enough decimal digits to recover the identical binary64 value
when read, such as the result of `std::to_chars` with `max_digits10`. Readers
must reject overflow, underflow to a different value, NaN, and infinities.

Entity IDs are positive unsigned 64-bit integers encoded as canonical decimal
strings matching `[1-9][0-9]*`. Encoding IDs as strings avoids loss in JSON
implementations whose numeric type cannot exactly represent all 64-bit integers.
Leading zeroes, signs, duplicate IDs, and values above `18446744073709551615`
are invalid.

## Root Object

| Field | Required | Meaning |
| --- | --- | --- |
| `format` | yes | The literal string `nurbsman`. |
| `version` | yes | The integer document version, currently `1`. |
| `generator` | no | Informational writer name/version. It has no effect on interpretation. |
| `units` | yes | Object whose required `length` field is `millimeter`. |
| `coordinates` | yes | Required coordinate-system declaration. |
| `entities` | yes | Scene entities in outliner/render order. May be empty. |

The coordinate object requires `handedness: "right"`, `up_axis: "y"`, and
`front_axis: "-z"`.

## Entities

Each item in `entities` has these fields:

| Field | Required | Meaning |
| --- | --- | --- |
| `id` | yes | Stable entity ID encoded as a decimal string. |
| `name` | yes | UTF-8 display name. Empty names are valid. |
| `visible` | yes | Whether the entity participates in display and viewport picking. |
| `geometry` | yes | Versioned geometry object. |

Array position is the entity order; there is no redundant numeric order field.
`geometry_revision` is not serialized because it is a runtime cache-invalidation
counter rather than document data.

Every geometry object has a required string `type` discriminator. Version 1
defines only `nurbs_surface`. Future geometry types require a document version
whose schema and reader define that discriminator; version 1 readers reject
unknown types.

## NURBS Surfaces

A `nurbs_surface` geometry object contains:

| Field | Required | Meaning |
| --- | --- | --- |
| `type` | yes | The literal string `nurbs_surface`. |
| `u_degree` | yes | Polynomial degree in U. |
| `v_degree` | yes | Polynomial degree in V. |
| `u_count` | yes | Number of control points in U. |
| `v_count` | yes | Number of control points in V. |
| `u_knots` | yes | Complete, nondecreasing U knot vector. |
| `v_knots` | yes | Complete, nondecreasing V knot vector. |
| `control_points` | yes | U-major array of control-point objects. |

Each control point requires finite binary64 `x`, `y`, `z`, and `weight` fields.
Weights must be positive.

In addition to schema validation, readers must enforce these invariants before
replacing the open scene:

- `u_degree < u_count` and `v_degree < v_count`.
- `control_points.size() == u_count * v_count` without integer overflow.
- Each knot-vector size equals `control_count + degree + 1` without overflow.
- Knots are finite and nondecreasing, multiplicity does not exceed `degree + 1`,
  and each parameter domain is nonempty.
- Every control point and homogeneous coordinate is within the numeric range
  supported by `NurbsSurface`.

These are the same validity rules used by the geometry constructor. A reader
must build and validate a temporary scene, including ID uniqueness, before
making it the active document.

## Example

```json
{
  "format": "nurbsman",
  "version": 1,
  "generator": "Nurbsman",
  "units": { "length": "millimeter" },
  "coordinates": {
    "handedness": "right",
    "up_axis": "y",
    "front_axis": "-z"
  },
  "entities": [
    {
      "id": "1",
      "name": "Plane",
      "visible": true,
      "geometry": {
        "type": "nurbs_surface",
        "u_degree": 1,
        "v_degree": 1,
        "u_count": 2,
        "v_count": 2,
        "u_knots": [0, 0, 1, 1],
        "v_knots": [0, 0, 1, 1],
        "control_points": [
          { "x": -1, "y": 0, "z": -1, "weight": 1 },
          { "x": -1, "y": 0, "z": 1, "weight": 1 },
          { "x": 1, "y": 0, "z": -1, "weight": 1 },
          { "x": 1, "y": 0, "z": 1, "weight": 1 }
        ]
      }
    }
  ]
}
```

## Load Errors

Load errors must contain the source path and a JSON field path when one exists,
using paths such as `$.entities[2].geometry.u_knots[5]`. Messages must distinguish
at least I/O failure, malformed JSON, schema/type failure, unsupported version,
duplicate entity ID, and invalid NURBS geometry. A failed read or validation must
leave the current scene, selection, and command history unchanged.

Selection, camera state, command history, current file path, saved-history
position, and dirty state are editor/session state and are not part of version 1.
