# Voxora

### Deterministic 3D Video Projection in Pure Rust

**Voxora** is a pure-Rust computational vision library for transforming ordinary video into a spatial 3D representation using deterministic mathematics and geometric algorithms.

Instead of relying on pretrained neural networks, AI models, or machine-learning inference, Voxora approaches video-to-3D reconstruction as a **geometry, projection, motion, and spatial estimation problem**.

A camera moves.

The video contains that movement.

Voxora uses the mathematical information contained in the video to estimate how visual information should be positioned in three-dimensional space.

> **Video is not just a sequence of images. It is a sequence of observations of a changing spatial scene.**

---

## What Is Voxora?

Voxora explores a simple idea:

**Can a conventional video be projected into 3D space without a trained AI model?**

The answer is approached through classical computational geometry.

Given a video such as:

```text
Frame 1 → Frame 2 → Frame 3 → Frame 4 → ...
```

Voxora analyzes the relationship between frames and constructs a spatial representation where visual information can move according to estimated camera motion and geometric relationships.

Conceptually:

```text
                 3D Space
                    ↑
                    │
          ┌─────────┼─────────┐
          │         │         │
       Left      Camera      Right
      Region       ●        Region
          ╲        │        ╱
           ╲       │       ╱
            ╲      │      ╱
             └─────┴─────┘
                   ↑
                 Video
```

If the camera moves to the left, the reconstructed spatial representation changes accordingly.

If the camera moves to the right, the projection follows the opposite direction.

This creates a spatial interpretation of the original video rather than simply displaying a flat sequence of frames.

---

## Core Philosophy

Voxora follows three principles:

### 1. No Pretrained AI Models

Voxora does not require:

* pretrained depth models
* neural networks
* large language models
* diffusion models
* downloaded AI checkpoints
* model inference runtimes
* training datasets

The system is designed around deterministic computational algorithms.

### 2. Pure Mathematics

The underlying problem is treated as mathematics.

Relevant techniques include concepts such as:

* projective geometry
* camera geometry
* coordinate transformations
* homography
* optical flow
* feature correspondence
* motion estimation
* triangulation
* parallax
* perspective projection
* image warping
* interpolation
* geometric consistency
* spatial transformations

The goal is not to reproduce an AI model.

The goal is to determine how much spatial information can be recovered directly from the mathematical structure of visual observations.

### 3. Rust First

Voxora is implemented entirely in **Rust**.

Rust provides:

* predictable performance
* memory safety
* zero-cost abstractions
* strong numerical control
* efficient native execution
* excellent support for parallel computation
* cross-platform compilation

The project is designed to be usable as a native Rust library rather than being tied to a Python-based machine-learning ecosystem.

---

## From Video to 3D

A conventional video frame contains a 2D projection of the physical world.

For example:

```text
Real World
     │
     │ Camera projection
     ▼
┌───────────────┐
│   2D Frame    │
│               │
│   ●      ●    │
│       █       │
│   ●      ●    │
└───────────────┘
```

The important information is not only inside an individual frame.

It also exists in the **relationship between consecutive frames**.

When the camera moves:

```text
Frame A

      ●
          █
  ●


Frame B

          ●
     █
              ●


Frame C

              ●
  █
                   ●
```

Objects change their apparent position.

Different regions of the scene move differently.

Near objects exhibit stronger apparent motion than distant objects.

This phenomenon is known as **parallax**.

Voxora uses these changes as geometric information.

---

## Parallax as Spatial Information

Parallax is one of the fundamental ideas behind spatial reconstruction.

Consider a camera moving horizontally:

```text
Object A          Object B
   ●                  ●
   │                  │
   │                  │
───┼──────────────────┼──────
          Camera
            ●
```

When the camera changes position, the apparent displacement of the objects differs depending on their spatial relationship to the camera.

This allows relative spatial structure to be estimated without requiring a trained neural network.

Voxora therefore treats camera movement as a source of information rather than simply a nuisance in the video.

---

## Mathematical Projection

A point in 3D space can be represented as:

[
P =
\begin{bmatrix}
X \
Y \
Z
\end{bmatrix}
]

and projected into image coordinates using a camera model:

[
p = K[R|t]P
]

where:

* (K) represents camera intrinsic parameters
* (R) represents camera rotation
* (t) represents camera translation
* (P) represents a point in 3D space
* (p) represents its projected image position

Voxora works around these relationships to determine how image observations can be transformed into spatial coordinates.

The result is not an arbitrary AI-generated 3D scene.

It is a **geometrically derived representation of the observed video**.

---

## No Depth Model Required

Traditional modern video-to-3D systems often depend on monocular depth estimation models.

Voxora takes a different direction.

Instead of asking:

> "What depth does a neural network predict?"

the system asks:

> "What spatial constraints can be mathematically derived from the observations?"

This distinction is fundamental.

A deterministic geometric system may have less semantic understanding than a large trained model, but it has several important properties:

* reproducible results
* no model download
* no GPU requirement for inference
* no training process
* no model weights
* deterministic execution
* lower deployment complexity
* fully inspectable algorithms

---

## 3D Projection Instead of AI Generation

Voxora does not attempt to hallucinate information that was never observed.

Its purpose is to transform available visual evidence into spatial coordinates and projections.

Conceptually:

```text
        INPUT VIDEO
             │
             ▼
     Frame Extraction
             │
             ▼
    Feature / Motion Analysis
             │
             ▼
     Geometric Estimation
             │
             ▼
       Spatial Mapping
             │
             ▼
      3D Projection
             │
             ▼
        Spatial Video
```

The system therefore belongs closer to:

**computer vision + computational geometry + graphics**

than to:

**generative AI + neural rendering**.

---

## Classical Computer Vision

Voxora is intentionally inspired by classical computer vision techniques.

Potential mathematical components include:

### Feature Correspondence

Finding corresponding visual points between frames.

### Motion Estimation

Estimating how those points move between observations.

### Optical Flow

Representing apparent pixel motion as a vector field:

[
F(x,y) =
\begin{bmatrix}
u(x,y) \
v(x,y)
\end{bmatrix}
]

### Homography

Modeling planar transformations between image observations.

[
p' \sim Hp
]

### Camera Motion

Estimating transformations between camera poses.

### Triangulation

Recovering spatial points from multiple observations.

### Image Warping

Transforming image information according to geometric mappings.

### Perspective Projection

Mapping reconstructed spatial coordinates back into a virtual camera.

These techniques can be combined to construct a deterministic video-to-space pipeline.

---

## What Voxora Is Not

Voxora is **not**:

* a pretrained AI model
* a neural network
* a generative video model
* a text-to-3D model
* a depth-estimation checkpoint
* a NeRF implementation
* an LLM-powered vision system
* a cloud AI API

There are no model weights hidden behind the library.

The computational intelligence comes from algorithms and mathematics.

---

## Why Pure Mathematics?

Modern AI systems can approximate extremely complex relationships because they contain enormous amounts of learned parameters.

But the underlying operations are still mathematical.

Voxora explores the opposite direction:

Instead of learning a mapping from millions of examples:

[
Video \rightarrow Neural\ Network \rightarrow Depth
]

it investigates a deterministic mapping:

[
Video \rightarrow Geometry \rightarrow Spatial\ Representation
]

This makes the system particularly interesting for environments where:

* model files are undesirable
* offline execution is required
* deterministic behavior matters
* memory is limited
* GPU acceleration is unavailable
* reproducibility is important
* explainability is preferred

---

## Rust Crate

Voxora is designed to be consumed as a native Rust library.

```toml
[dependencies]
voxora = "*"
```

The API is intended to expose the computational pipeline without requiring users to build an AI infrastructure around it.

The project can therefore serve as a foundation for applications such as:

* spatial video experiments
* 3D visualization
* computational photography
* computer vision research
* robotics experiments
* camera-motion analysis
* video geometry
* experimental XR pipelines
* spatial media processing
* offline video reconstruction

---

## Designed for Ordinary Video

Voxora does not require extremely high-resolution footage to demonstrate the underlying concept.

The primary objective is not cinematic reconstruction quality.

The objective is:

> **Extract spatial structure from visual motion.**

A lower-resolution video can still contain useful information about:

* motion
* feature correspondence
* camera movement
* relative displacement
* perspective
* parallax

This makes Voxora suitable for experimentation with ordinary video sources.

---

## Deterministic by Design

Given the same input and configuration, a deterministic computational pipeline should produce reproducible results.

That makes it possible to inspect and reason about:

```text
Input
  ↓
Mathematical transformation
  ↓
Intermediate geometry
  ↓
Spatial representation
  ↓
Projection
```

There is no hidden training state.

There are no randomly learned weights.

There is no external inference service.

The result comes from the algorithms.

---

## Performance

Rust allows Voxora to operate close to the hardware while maintaining memory safety.

The architecture is suitable for optimization through:

* SIMD
* multithreading
* parallel frame processing
* efficient memory layouts
* zero-copy data paths where applicable
* native image processing
* CPU vectorization
* GPU acceleration where explicitly implemented

The goal is not to make the system dependent on a powerful GPU.

The goal is to make the mathematical pipeline efficient enough to run as a native computational library.

---

## Spatial Video Without a Neural Network

Voxora explores a fundamental question in computer vision:

> **How much 3D structure can be recovered from ordinary video using mathematics alone?**

The answer is not expected to replace every modern AI-based reconstruction system.

Instead, Voxora provides another engineering path.

A path based on:

**Geometry.**

**Projection.**

**Motion.**

**Parallax.**

**Algorithms.**

**Rust.**

---

## License

Voxora is open source and intended for experimentation, research, and practical applications involving deterministic video geometry and spatial reconstruction.
