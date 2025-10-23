# NOC — Ball Cube Physics

A small interactive Bevy demo showing a ball bouncing inside a translucent cubic container.

## Build

From the project root run:

```bash
cargo build --release
```

Or for a quick debug build:

```bash
cargo build
```

## Run

Run the demo with:

```bash
cargo run --release
```

## Controls

- Left click: place an attractor/repulser at the clicked world position (type depends on the `attractor_direction` setting in the inspector).
- Pan / orbit camera: Scroll wheel zooms in and out. Click and drag to pan.

## Configuration

The `Configuration` resource is exposed in the inspector and includes:

- `radius` — radius (scale) of the ball.
- `speed` — global speed multiplier for motion updates.
- `boundaries` — xyz dimensions of the container.
- `dampening` — collision dampening (0..1) applied on bounce. Low dampening makes the ball bouncier.
- `attractor_strength` — strength of placed force emitters.
- `attractor_direction` — `In` or `Out` (attractor vs repulser).

Change these values at runtime to experiment with different behaviors.

## Tests

```bash
cargo test
```
