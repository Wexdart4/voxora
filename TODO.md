# Voxora — TODO

## Repository & Rust Workspace

* [x] Initialize the Rust workspace
* [x] Define the workspace package metadata
* [x] Define crate boundaries for the computational pipeline
* [x] Configure Rust edition
* [x] Configure minimum supported Rust version
* [x] Configure release optimization
* [x] Enable link-time optimization for release builds
* [x] Configure code generation units
* [x] Configure panic strategy where appropriate
* [x] Add workspace-level dependency management
* [x] Add common lint configuration
* [x] Enable `clippy`
* [x] Enable `rustfmt`
* [x] Enable compiler warnings as errors in CI where appropriate
* [x] Add SPDX/license metadata
* [x] Add crate-level documentation
* [x] Add repository-level documentation
* [x] Add examples for the public API
* [x] Add integration tests
* [x] Add unit tests for mathematical primitives

---

# Video Input

## Video Decoding

* [x] Implement video file loading
* [x] Support common video containers
* [x] Support common video codecs through a pluggable backend
* [x] Extract video metadata
* [x] Detect frame count where available
* [x] Detect frame rate
* [x] Detect frame dimensions
* [x] Detect pixel format
* [x] Detect color space
* [x] Detect duration
* [x] Implement sequential frame decoding
* [x] Implement random frame access where supported
* [x] Implement streaming frame decoding
* [x] Avoid loading the entire video into memory
* [x] Add configurable frame buffering
* [x] Add decoder error handling
* [x] Handle corrupted frames gracefully
* [x] Handle variable-frame-rate video
* [x] Handle missing metadata
* [x] Handle videos with unusual dimensions
* [x] Add decoder benchmarks

## Frame Representation

* [x] Define internal frame representation
* [x] Support grayscale frames
* [x] Support RGB frames
* [x] Support RGBA frames
* [x] Support normalized floating-point representations
* [x] Implement frame conversion
* [x] Implement frame resizing
* [x] Implement frame cropping
* [x] Implement frame normalization
* [x] Implement frame cloning efficiently
* [x] Avoid unnecessary allocations
* [x] Add zero-copy conversion where possible
* [x] Add frame memory benchmarks

---

# Image Processing

## Preprocessing

* [x] Implement grayscale conversion
* [x] Implement Gaussian filtering
* [x] Implement box filtering
* [x] Implement median filtering
* [x] Implement image normalization
* [x] Implement contrast normalization
* [x] Implement brightness normalization
* [x] Implement histogram computation
* [x] Implement optional histogram equalization
* [x] Implement image pyramids
* [x] Implement Gaussian pyramids
* [x] Implement downsampling
* [x] Implement upsampling
* [x] Implement configurable preprocessing pipelines

## Edge Detection

* [x] Implement image gradients
* [x] Implement Sobel operator
* [x] Implement Scharr operator
* [x] Implement gradient magnitude
* [x] Implement gradient orientation
* [x] Implement non-maximum suppression
* [x] Implement thresholding
* [x] Implement Canny edge detection
* [x] Benchmark edge detection algorithms

---

# Feature Detection

## Feature Representation

* [x] Define feature point structure
* [x] Store pixel coordinates
* [x] Store scale
* [x] Store orientation
* [x] Store response/confidence
* [x] Store frame identifier
* [x] Store feature descriptor
* [x] Support feature lifetime tracking

## Feature Detection Algorithms

* [x] Implement Harris corner detection
* [x] Implement Shi-Tomasi corner detection
* [x] Implement FAST corner detection
* [x] Evaluate feature detector stability
* [x] Add configurable feature detection thresholds
* [x] Add configurable maximum feature count
* [x] Add spatial feature distribution constraints
* [x] Prevent excessive clustering of features
* [x] Implement feature-grid distribution
* [x] Add multi-scale feature detection

## Feature Descriptors

* [x] Define descriptor interface
* [x] Implement binary descriptors
* [x] Implement patch-based descriptors
* [x] Implement descriptor normalization
* [x] Implement descriptor distance metrics
* [x] Implement Hamming distance
* [x] Implement Euclidean descriptor distance
* [x] Benchmark descriptor performance

---

# Feature Matching

## Frame-to-Frame Matching

* [x] Implement feature matching between consecutive frames
* [x] Implement brute-force matcher
* [x] Implement spatially constrained matching
* [x] Implement descriptor-distance matching
* [x] Implement nearest-neighbor matching
* [x] Implement second-nearest-neighbor matching
* [x] Implement ratio-test filtering
* [x] Remove duplicate correspondences
* [x] Reject physically impossible matches
* [x] Add configurable matching thresholds

## Robust Matching

* [x] Implement geometric consistency checks
* [x] Implement RANSAC
* [x] Implement homography-based outlier rejection
* [x] Implement fundamental-matrix-based outlier rejection
* [x] Implement essential-matrix-based outlier rejection
* [x] Calculate inlier ratios
* [x] Track matching confidence
* [x] Detect feature-poor frames
* [x] Detect tracking failure

---

# Optical Flow

## Dense Optical Flow

* [x] Define optical-flow vector representation
* [x] Implement image pyramid
* [x] Implement Lucas-Kanade optical flow
* [x] Implement pyramidal Lucas-Kanade
* [x] Implement iterative flow refinement
* [x] Implement flow confidence
* [x] Implement flow consistency checking
* [x] Implement forward-backward flow validation
* [x] Detect unreliable flow regions

## Sparse Optical Flow

* [x] Implement sparse feature tracking
* [x] Track selected feature points across frames
* [x] Implement subpixel refinement
* [x] Implement feature lifetime tracking
* [x] Remove unstable tracks
* [x] Detect feature drift
* [x] Detect occlusion
* [x] Detect track termination

---

# Camera Geometry

## Camera Model

* [x] Define camera intrinsic matrix
* [x] Define camera extrinsic parameters
* [x] Define camera pose
* [x] Define rotation representation
* [x] Define translation representation
* [x] Define homogeneous coordinates
* [x] Implement camera coordinate system
* [x] Implement world coordinate system
* [x] Implement camera-to-world transformation
* [x] Implement world-to-camera transformation

## Camera Calibration

* [x] Define calibration data structure
* [x] Support manually supplied camera parameters
* [x] Support estimated focal length
* [x] Support principal point estimation
* [x] Support field-of-view estimation
* [x] Implement radial distortion model
* [x] Implement tangential distortion model
* [x] Implement lens distortion correction
* [x] Implement optional automatic calibration
* [x] Validate calibration consistency

---

# Geometric Transformations

## Homography

* [x] Define homography matrix
* [x] Implement homography estimation
* [x] Implement normalized DLT
* [x] Implement homography decomposition where applicable
* [x] Implement homography-based image warping
* [x] Implement homography validation
* [x] Implement homography error metrics

## Fundamental Matrix

* [x] Implement fundamental matrix representation
* [x] Implement normalized eight-point algorithm
* [x] Implement seven-point algorithm where useful
* [x] Implement epipolar constraint
* [x] Implement Sampson distance
* [x] Implement fundamental matrix RANSAC
* [x] Validate epipolar geometry

## Essential Matrix

* [x] Implement essential matrix estimation
* [x] Implement essential matrix decomposition
* [x] Recover candidate camera rotations
* [x] Recover candidate camera translations
* [x] Resolve pose ambiguity
* [x] Validate recovered camera pose
* [x] Implement cheirality checks

---

# Camera Motion Estimation

* [x] Estimate frame-to-frame camera motion
* [x] Estimate relative rotation
* [x] Estimate relative translation direction
* [x] Estimate camera trajectory
* [x] Accumulate camera transformations
* [x] Normalize trajectory
* [x] Detect camera-motion discontinuities
* [x] Detect static-camera sequences
* [x] Detect sudden camera rotations
* [x] Detect tracking loss
* [x] Implement trajectory smoothing
* [x] Implement configurable motion filtering
* [x] Preserve raw trajectory alongside filtered trajectory

---

# Depth Estimation

## Geometric Depth

* [x] Define depth representation
* [x] Define disparity representation
* [x] Implement depth from triangulation
* [x] Implement depth from camera motion
* [x] Implement depth from parallax
* [x] Implement depth confidence
* [x] Reject invalid triangulation
* [x] Reject negative depth
* [x] Reject unstable depth
* [x] Implement depth normalization

## Sparse Depth

* [x] Generate sparse 3D points from tracked features
* [x] Associate depth with feature tracks
* [x] Merge observations of the same point
* [x] Remove inconsistent points
* [x] Estimate point confidence
* [x] Track point visibility

## Dense Depth

* [x] Investigate classical stereo-style reconstruction
* [x] Implement correspondence search
* [x] Implement block matching
* [x] Implement normalized cross-correlation matching
* [x] Implement disparity estimation
* [x] Implement disparity refinement
* [x] Implement left-right consistency checking
* [x] Convert disparity into depth
* [x] Fill small invalid regions
* [x] Preserve uncertainty information

---

# Triangulation

* [x] Define triangulation API
* [x] Implement linear triangulation
* [x] Implement homogeneous triangulation
* [x] Implement normalized triangulation
* [x] Implement triangulation error calculation
* [x] Implement reprojection error
* [x] Reject points with excessive reprojection error
* [x] Reject points behind the camera
* [x] Reject points with unstable depth
* [x] Support multi-view triangulation
* [x] Merge repeated observations
* [x] Implement point confidence scoring

---

# 3D Reconstruction

## Point Cloud

* [x] Define 3D point structure
* [x] Store XYZ coordinates
* [x] Store RGB color
* [x] Store normal where available
* [x] Store confidence
* [x] Store source frame
* [x] Store observation count
* [x] Implement point-cloud container
* [x] Implement point-cloud filtering
* [x] Implement duplicate-point merging
* [x] Implement outlier removal
* [x] Implement spatial downsampling
* [x] Implement voxel-based filtering

## Multi-Frame Reconstruction

* [x] Transform frame-local points into world coordinates
* [x] Merge points across frames
* [x] Resolve overlapping observations
* [ ] Maintain point identities
* [ ] Detect reconstruction drift
* [ ] Implement global trajectory optimization
* [ ] Implement pose refinement
* [ ] Implement bundle-adjustment foundations
* [ ] Minimize reprojection error
* [ ] Detect inconsistent geometry

---

# Spatial Video Projection

## Image-to-Space Mapping

* [x] Map frame pixels into camera coordinates
* [x] Transform camera coordinates into world coordinates
* [x] Associate image color with spatial points
* [x] Implement perspective-aware projection
* [x] Implement spatial image warping
* [x] Preserve frame timing
* [x] Preserve motion direction
* [x] Handle camera translation
* [x] Handle camera rotation
* [x] Handle changing field of view

## Left / Right Spatial Projection

* [x] Detect horizontal camera movement
* [x] Estimate movement direction
* [x] Map leftward camera motion into spatial displacement
* [x] Map rightward camera motion into spatial displacement
* [x] Preserve relative object displacement
* [x] Handle forward camera motion
* [x] Handle backward camera motion
* [x] Handle combined translation and rotation
* [x] Handle diagonal camera movement
* [x] Handle camera rotation without translation
* [x] Separate rotational and translational components where possible

---

# 3D Coordinate System

* [x] Define world coordinate convention
* [x] Define camera coordinate convention
* [x] Define axis orientation
* [x] Define unit system
* [x] Implement coordinate conversion
* [x] Implement rotation matrices
* [x] Implement quaternions
* [x] Implement Euler-angle conversion
* [x] Implement rigid-body transformations
* [x] Implement SE(3) representation
* [x] Implement interpolation between poses
* [x] Implement pose composition
* [x] Implement pose inversion

---

# Image Warping

* [x] Implement nearest-neighbor interpolation
* [x] Implement bilinear interpolation
* [x] Implement bicubic interpolation where useful
* [x] Implement perspective warping
* [x] Implement inverse warping
* [x] Implement forward warping
* [x] Handle image boundaries
* [x] Handle missing pixels
* [x] Handle occlusion regions
* [x] Detect holes after projection
* [x] Implement hole filling
* [x] Prevent excessive image stretching
* [x] Preserve image sharpness where possible

---

# Occlusion Handling

* [x] Detect newly exposed regions
* [x] Detect disappearing regions
* [x] Detect inconsistent correspondence
* [x] Build visibility information
* [x] Implement depth ordering
* [x] Resolve overlapping projections
* [x] Handle foreground/background conflicts
* [x] Handle partial occlusion
* [x] Handle full occlusion
* [x] Avoid projecting invalid pixels

---

# Spatial Reconstruction Quality

* [x] Define reprojection error
* [x] Define geometric consistency error
* [x] Define temporal consistency error
* [x] Define optical-flow consistency error
* [x] Define depth confidence
* [x] Define feature-track confidence
* [x] Define camera-pose confidence
* [x] Combine confidence measurements
* [x] Expose reconstruction diagnostics
* [x] Generate reconstruction statistics

---

# Temporal Consistency

* [x] Prevent frame-to-frame geometry flickering
* [x] Smooth unstable camera trajectories
* [x] Maintain stable feature identities
* [x] Maintain temporal point consistency
* [x] Detect sudden geometry changes
* [x] Detect scene cuts
* [x] Reset reconstruction after scene cuts
* [x] Handle fades
* [x] Handle transitions
* [x] Handle rapid camera motion

---

# Scene Segmentation by Geometry

* [x] Detect independently moving regions
* [x] Detect dominant background motion
* [x] Detect foreground motion
* [x] Separate camera motion from object motion
* [x] Detect dynamic objects
* [x] Mark unreliable dynamic regions
* [x] Avoid contaminating camera estimation with moving objects
* [x] Track static-scene confidence

---

# Static Scene Reconstruction

* [x] Detect approximately static scenes
* [x] Optimize reconstruction for static environments
* [x] Improve point accumulation
* [x] Improve multi-view consistency
* [x] Improve depth stability
* [x] Improve camera trajectory estimation
* [x] Preserve long-lived spatial features

---

# Dynamic Scene Handling

* [x] Detect moving objects
* [x] Track dynamic regions
* [x] Separate static and dynamic geometry
* [x] Prevent dynamic objects from corrupting camera estimation
* [x] Maintain optional dynamic geometry
* [x] Handle objects entering the scene
* [x] Handle objects leaving the scene
* [x] Handle partial motion
* [x] Handle motion blur

---

# Motion Blur

* [x] Detect motion-blurred frames
* [x] Estimate feature reliability under blur
* [x] Reject unstable feature points
* [x] Adjust tracking thresholds
* [x] Handle short blur sequences
* [x] Recover tracking after blur

---

# Scene Cut Detection

* [x] Detect hard cuts
* [x] Detect cross-fades
* [x] Detect fade-to-black
* [x] Detect fade-from-black
* [x] Reset feature tracks after cuts
* [x] Reset camera estimation after cuts
* [x] Start a new spatial reconstruction segment

---

# Rendering

## 3D Renderer

* [x] Define rendering abstraction
* [x] Define virtual camera
* [x] Implement camera projection
* [x] Implement point-cloud rendering
* [x] Implement depth-aware rendering
* [x] Implement textured projection
* [x] Implement basic lighting-independent rendering
* [x] Implement viewport transformation
* [x] Implement camera movement
* [x] Implement spatial zoom
* [x] Implement clipping planes

## Software Rendering

* [x] Implement CPU rendering fallback
* [x] Support headless rendering
* [x] Support deterministic rendering
* [x] Benchmark CPU renderer

## GPU Rendering

* [x] Define optional GPU backend
* [x] Evaluate `wgpu`
* [x] Implement GPU buffer management
* [x] Upload reconstructed geometry
* [x] Upload textures
* [x] Implement GPU projection
* [x] Implement depth buffering
* [x] Implement GPU image warping
* [x] Implement asynchronous rendering
* [x] Avoid unnecessary CPU-GPU transfers

---

# Spatial Output

* [x] Define reconstructed scene format
* [x] Define point-cloud export
* [x] Support PLY export
* [x] Support OBJ export where applicable
* [x] Support glTF export where applicable
* [x] Support raw point-cloud serialization
* [x] Export camera trajectory
* [x] Export camera poses
* [x] Export depth maps
* [x] Export disparity maps
* [x] Export confidence maps
* [x] Export reconstructed frames
* [x] Preserve timestamps

---

# Video Output

* [x] Define spatial video output abstraction
* [x] Implement frame reprojection
* [x] Implement virtual camera rendering
* [x] Generate left-eye output
* [x] Generate right-eye output
* [x] Generate stereo output
* [x] Support side-by-side output
* [x] Support top-bottom output
* [x] Support depth-assisted output
* [x] Preserve original frame rate
* [x] Preserve audio where applicable
* [x] Synchronize audio and reconstructed video

---

# Stereo Projection

* [x] Define left-eye camera
* [x] Define right-eye camera
* [x] Define interocular distance
* [x] Generate left-eye projection
* [x] Generate right-eye projection
* [x] Preserve depth ordering
* [x] Validate stereo consistency
* [x] Detect excessive disparity
* [x] Clamp invalid disparity
* [x] Prevent severe visual discomfort
* [x] Support configurable stereo strength

---

# Mathematical Core

* [x] Implement fixed-size matrix types where appropriate
* [x] Implement vector operations
* [x] Implement matrix multiplication
* [x] Implement matrix inversion
* [x] Implement matrix decomposition
* [x] Implement determinant calculation
* [x] Implement eigenvalue/eigenvector utilities where required
* [x] Implement SVD integration or implementation
* [x] Implement least-squares solving
* [x] Implement nonlinear optimization primitives
* [x] Implement robust loss functions
* [x] Implement numerical stability checks
* [x] Handle floating-point precision issues
* [x] Detect NaN propagation
* [x] Detect infinite values
* [x] Add numerical tolerance configuration

---

# Optimization

* [x] Profile the entire pipeline
* [x] Identify allocation hotspots
* [x] Identify CPU hotspots
* [x] Identify memory bandwidth bottlenecks
* [x] Optimize frame decoding
* [x] Optimize grayscale conversion
* [x] Optimize feature extraction
* [x] Optimize descriptor computation
* [x] Optimize feature matching
* [x] Optimize optical flow
* [x] Optimize matrix operations
* [x] Optimize triangulation
* [x] Optimize point-cloud merging
* [x] Optimize image warping
* [x] Optimize rendering
* [x] Add SIMD implementations where beneficial
* [x] Add parallel processing where beneficial
* [x] Benchmark single-threaded execution
* [x] Benchmark multithreaded execution
* [x] Benchmark memory usage

---

# Memory Management

* [x] Define frame lifetime strategy
* [x] Define feature lifetime strategy
* [x] Define point-cloud memory strategy
* [x] Avoid unnecessary frame duplication
* [x] Reuse temporary buffers
* [x] Reuse image-processing buffers
* [x] Reuse feature buffers
* [x] Implement bounded frame queues
* [x] Implement streaming reconstruction
* [x] Prevent unbounded point-cloud growth
* [x] Implement configurable reconstruction limits
* [x] Add memory-pressure diagnostics

---

# Parallelism

* [x] Identify parallelizable pipeline stages
* [x] Parallelize independent frame preprocessing
* [x] Parallelize feature extraction
* [x] Parallelize descriptor computation
* [x] Parallelize image operations
* [x] Parallelize point processing
* [x] Parallelize projection where safe
* [x] Avoid excessive thread synchronization
* [x] Benchmark thread scaling
* [x] Add configurable worker count

---

# Error Handling

* [x] Define public error types
* [x] Define decoder errors
* [x] Define image-processing errors
* [x] Define feature-tracking errors
* [x] Define camera-estimation errors
* [x] Define triangulation errors
* [x] Define numerical errors
* [x] Define reconstruction errors
* [x] Define rendering errors
* [x] Provide meaningful error messages
* [x] Avoid panics in normal failure cases
* [x] Add recovery strategies where possible

---

# Configuration

* [x] Define reconstruction configuration
* [x] Define feature configuration
* [x] Define optical-flow configuration
* [x] Define camera configuration
* [x] Define depth configuration
* [x] Define stereo configuration
* [x] Define rendering configuration
* [x] Define performance configuration
* [x] Define quality presets
* [x] Support deterministic configuration
* [x] Validate configuration values

---

# API Design

* [x] Design ergonomic public API
* [x] Separate low-level mathematical APIs from high-level reconstruction APIs
* [x] Define video input API
* [x] Define frame API
* [x] Define feature API
* [x] Define camera API
* [x] Define geometry API
* [x] Define reconstruction API
* [x] Define rendering API
* [x] Define export API
* [x] Minimize unnecessary public types
* [x] Document invariants
* [x] Document coordinate conventions
* [x] Document numerical assumptions
* [x] Ensure public APIs are thread-safe where possible

---

# CLI

* [x] Create Voxora CLI
* [x] Add video input argument
* [x] Add output argument
* [x] Add reconstruction configuration
* [x] Add camera configuration
* [x] Add quality configuration
* [x] Add frame-range selection
* [x] Add frame-step configuration
* [x] Add output-format selection
* [x] Add stereo-output configuration
* [x] Add visualization mode
* [x] Add benchmark mode
* [x] Add diagnostics mode
* [x] Add verbose logging
* [x] Add progress reporting
* [x] Add structured error output

---

# Visualization & Debugging

* [x] Visualize detected features
* [x] Visualize feature matches
* [x] Visualize optical flow
* [x] Visualize camera trajectory
* [x] Visualize sparse point cloud
* [x] Visualize dense point cloud
* [x] Visualize depth
* [x] Visualize disparity
* [x] Visualize reprojection error
* [x] Visualize confidence
* [x] Visualize epipolar geometry
* [x] Visualize reconstructed camera
* [x] Visualize left/right stereo views
* [x] Add debug frame export

---

# Testing

## Unit Tests

* [x] Test vector operations
* [x] Test matrix operations
* [x] Test transformations
* [x] Test projection
* [x] Test inverse projection
* [x] Test homography
* [x] Test fundamental matrix
* [x] Test essential matrix
* [x] Test triangulation
* [x] Test interpolation
* [x] Test image warping
* [x] Test coordinate conversions
* [x] Test camera pose composition

## Integration Tests

* [x] Test video decoding
* [x] Test frame processing
* [x] Test feature detection
* [x] Test feature matching
* [x] Test camera estimation
* [x] Test depth reconstruction
* [x] Test point-cloud generation
* [x] Test spatial projection
* [x] Test stereo generation
* [x] Test output serialization

## Regression Tests

* [x] Store representative input sequences
* [x] Store expected camera trajectories
* [x] Store expected feature statistics
* [x] Store expected reconstruction statistics
* [x] Compare reprojection error
* [x] Detect numerical regressions
* [x] Detect performance regressions

---

# Synthetic Geometry Tests

* [x] Generate synthetic camera trajectories
* [x] Generate synthetic 3D points
* [x] Project synthetic points into frames
* [x] Add controlled camera motion
* [x] Add controlled noise
* [x] Recover camera motion
* [x] Recover depth
* [x] Compare reconstruction against ground truth
* [x] Measure triangulation error
* [x] Measure reprojection error
* [x] Measure trajectory error

---

# Real-World Video Tests

* [x] Test static indoor scene
* [x] Test static outdoor scene
* [x] Test horizontal camera movement
* [x] Test vertical camera movement
* [x] Test forward camera movement
* [x] Test backward camera movement
* [x] Test camera rotation
* [x] Test combined camera motion
* [x] Test handheld footage
* [x] Test low-resolution footage
* [x] Test compressed video
* [x] Test noisy video
* [x] Test motion blur
* [x] Test dynamic objects
* [x] Test scene cuts

---

# Numerical Validation

* [x] Establish numerical tolerances
* [x] Test floating-point stability
* [x] Test near-singular matrices
* [x] Test degenerate camera configurations
* [x] Test pure rotation
* [x] Test extremely small translation
* [x] Test extremely large translation
* [x] Test very distant points
* [x] Test very close points
* [x] Test noisy feature correspondences
* [x] Test incorrect correspondences
* [x] Test insufficient feature counts

---

# Benchmarking

* [x] Benchmark frame decoding
* [x] Benchmark preprocessing
* [x] Benchmark feature detection
* [x] Benchmark feature matching
* [x] Benchmark optical flow
* [x] Benchmark camera estimation
* [x] Benchmark triangulation
* [x] Benchmark point-cloud generation
* [x] Benchmark image warping
* [x] Benchmark rendering
* [x] Benchmark complete reconstruction
* [x] Measure frames per second
* [x] Measure latency per frame
* [x] Measure peak memory
* [x] Measure CPU utilization
* [x] Compare single-threaded and parallel execution

---

# Cross-Platform

* [x] Test Linux
* [x] Test Windows
* [x] Test macOS
* [x] Validate native video dependencies
* [x] Validate CPU feature detection
* [x] Validate SIMD compatibility
* [x] Validate GPU backend compatibility
* [x] Test release binaries
* [x] Test cross-compilation
* [x] Document platform-specific requirements

---

# Documentation

* [x] Document mathematical foundations
* [x] Document coordinate systems
* [x] Document camera model
* [x] Document feature tracking
* [x] Document optical flow
* [x] Document camera estimation
* [x] Document triangulation
* [x] Document depth reconstruction
* [x] Document spatial projection
* [x] Document stereo projection
* [x] Document rendering
* [x] Document numerical limitations
* [x] Document performance characteristics
* [x] Document API examples
* [x] Document CLI usage
* [x] Document supported formats
* [x] Document known limitations

---

# Examples

* [x] Add basic video loading example
* [x] Add frame extraction example
* [x] Add feature detection example
* [x] Add feature tracking example
* [x] Add camera-motion estimation example
* [x] Add sparse reconstruction example
* [x] Add point-cloud generation example
* [x] Add depth reconstruction example
* [x] Add 3D projection example
* [x] Add stereo projection example
* [x] Add complete video-to-space example

---

# Logging & Diagnostics

* [x] Add structured logging
* [x] Add configurable log levels
* [x] Log decoder information
* [x] Log frame processing statistics
* [x] Log feature statistics
* [x] Log tracking statistics
* [x] Log camera-estimation statistics
* [x] Log reconstruction statistics
* [x] Log numerical warnings
* [x] Log performance statistics
* [x] Add optional machine-readable diagnostics

---

# Reproducibility

* [x] Define deterministic processing mode
* [x] Avoid uncontrolled randomness
* [x] Seed RANSAC where deterministic behavior is required
* [x] Document floating-point reproducibility limitations
* [x] Record reconstruction configuration
* [x] Record input metadata
* [x] Record algorithm parameters
* [x] Enable reproducible benchmark runs

---

# Security & Robustness

* [x] Validate video metadata
* [x] Validate frame dimensions
* [x] Validate frame allocation sizes
* [x] Prevent excessive memory allocation
* [x] Handle malformed video input
* [x] Handle malformed numerical data
* [x] Prevent integer overflow in image calculations
* [x] Prevent buffer overflows
* [x] Fuzz image-processing APIs
* [x] Fuzz mathematical input boundaries
* [x] Fuzz video metadata handling

---

# Fuzz Testing

* [x] Fuzz matrix operations
* [x] Fuzz camera transformations
* [x] Fuzz projection functions
* [x] Fuzz triangulation
* [x] Fuzz image warping
* [x] Fuzz feature matching
* [x] Fuzz configuration parsing
* [x] Fuzz video metadata parsing
* [x] Fuzz serialization/deserialization

---

# Continuous Integration

* [x] Configure GitHub Actions
* [x] Run `cargo check`
* [x] Run `cargo test`
* [x] Run `cargo clippy`
* [x] Run `cargo fmt --check`
* [x] Run documentation tests
* [x] Test release builds
* [x] Test supported operating systems
* [x] Run benchmark smoke tests
* [x] Run security audit
* [x] Run dependency checks
* [x] Validate crates.io package

---

# crates.io

* [x] Choose final crate names
* [x] Check crate-name availability
* [x] Define crate descriptions
* [x] Add crate keywords
* [x] Add crate categories
* [x] Add repository metadata
* [x] Add documentation metadata
* [x] Add homepage metadata
* [x] Add license metadata
* [x] Add README to package
* [x] Verify package contents with `cargo package`
* [x] Inspect generated package
* [x] Remove unnecessary files from package
* [x] Verify documentation builds
* [x] Verify public API documentation
* [x] Publish initial crate
* [x] Verify crates.io rendering
* [x] Verify docs.rs rendering

---

# Documentation Website

* [x] Ensure docs.rs builds successfully
* [x] Document all public modules
* [x] Document all public structs
* [x] Document all public enums
* [x] Document all public traits
* [x] Document public functions
* [x] Add mathematical equations to documentation
* [x] Add usage examples
* [x] Add reconstruction examples
* [x] Add performance notes
* [x] Add limitations

---

# Code Quality

* [x] Remove unnecessary allocations
* [x] Remove unnecessary cloning
* [x] Remove dead code
* [x] Remove unused dependencies
* [x] Reduce public API surface
* [x] Improve error messages
* [x] Improve documentation coverage
* [x] Resolve clippy warnings
* [x] Resolve compiler warnings
* [x] Run formatter
* [x] Review unsafe code
* [x] Minimize unsafe code
* [x] Document every required unsafe block
* [x] Add safety invariants for unsafe code
* [x] Audit concurrency assumptions
* [x] Audit numerical assumptions

---

# Research Validation

* [x] Validate pure-geometry assumptions
* [x] Compare sparse and dense reconstruction
* [x] Evaluate camera-motion accuracy
* [x] Evaluate depth accuracy
* [x] Evaluate reprojection accuracy
* [x] Evaluate temporal consistency
* [x] Evaluate spatial stability
* [x] Evaluate low-resolution input
* [x] Evaluate compressed input
* [x] Evaluate different camera motions
* [x] Identify degenerate cases
* [x] Document cases where geometry alone is insufficient

---

# Final Validation

* [x] Input a normal 2D video
* [x] Decode frames successfully
* [x] Track visual features
* [x] Estimate camera movement
* [x] Recover relative spatial structure
* [x] Generate 3D points
* [x] Transform points into world coordinates
* [x] Project reconstructed geometry into a virtual camera
* [x] Generate left/right spatial views
* [x] Preserve camera movement direction
* [x] Verify leftward motion produces leftward spatial behavior
* [x] Verify rightward motion produces rightward spatial behavior
* [x] Verify temporal consistency
* [x] Verify reconstruction stability
* [x] Verify output synchronization
* [x] Verify deterministic execution
* [x] Verify no pretrained model is required
* [x] Verify no model weights are packaged
* [x] Verify the complete pipeline runs offline
* [x] Verify the complete pipeline works using pure Rust
* [x] Publish the crates to crates.io
* [x] Publish generated API documentation
* [x] Tag the first stable release

---

# Core Definition of Done

* [x] A normal video can be loaded
* [x] Frames can be processed without loading the entire video into memory
* [x] Visual correspondences can be extracted
* [x] Camera motion can be estimated
* [x] Relative depth can be mathematically reconstructed
* [x] 3D points can be generated
* [x] 3D points can be transformed into a world coordinate system
* [x] Video imagery can be projected into 3D space
* [x] Camera movement is reflected by the spatial projection
* [x] Left/right movement produces corresponding spatial displacement
* [x] A virtual camera can inspect the reconstructed scene
* [x] Stereo views can be generated
* [x] The system operates without pretrained AI models
* [x] The system operates without model weights
* [x] The core reconstruction pipeline is deterministic
* [x] The core implementation is written in Rust
* [x] The library is documented
* [x] The library is tested
* [x] The library is benchmarked
* [x] The crate can be published to crates.io
