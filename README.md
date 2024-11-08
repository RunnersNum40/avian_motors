# Avian Motors

The `avian_motors` crate provides a modular motor control system for use with the `avian3d` physics engine and the `bevy` game engine. Proper motor joints are a planned feature of Avian, so this will hopefully be obsolete soon.

## Features

- Velocity or position control of revolute joints.
- Example of motor implementation in `examples/revolute.rs`.

## Getting Started

### Prerequisites

This library requires `bevy` and `avian3d` as dependencies:

- `bevy = "0.14"`
- `avian3d = "0.1"`

Currently, only the `f64` feature set of Avian is supported, but I'll fix that soon.

### Installation

Add the following dependencies to your `Cargo.toml`:

```toml
[dependencies]
avian_motors = "0.1.0"
bevy = "0.14"
avian3d = { version = "0.1", default-features = false, features = [
  "3d",
  "f64",
  "default-collider",
  "parry-f64",
  "collider-from-mesh",
  "parallel",
  "simd"
] }
```

## Example Usage

A basic example of a motor can be found in `examples/revolute.rs`. To run the example:

`cargo run --example revolute`
