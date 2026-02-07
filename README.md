# Growing Graphs & High-Performance Visualization

A high-performance engine for simulating and visualizing rule-based growing graphs. This project combines graph-rewriting automata with a custom force-directed layout engine optimized using WebAssembly and WebGL.

## 🚀 Key Features

-   **High-Performance Physics**: All core simulation logic (multibody forces, link forces, and octree construction) is implemented in **WebAssembly (C)** for near-native performance.
-   **N-Body Optimization**: Implements a dual-tree Barnes-Hut algorithm in WASM for efficient $O(N \log N)$ charge force calculations.
-   **GPU-Accelerated Rendering**: Uses **SwissGL** (WebGL 2.0) for smooth, high-bandwidth visualization of thousands of nodes and links.
-   **Dynamic Growth**: Nodes evolve according to neighborhood-based rewriting rules, creating complex emergent structures from simple initial conditions.
-   **Autonomous Mode**: A generative artwork mode with aggressive stall detection, noise-induced recovery, and smooth transitions between procedural rules.

## 🎨 Acknowledgments & Inspiration

This project draws significant inspiration from the following research into graph-based automata and emergent complexity:

-   **[Graph-Rewriting Automata](https://paulcousin.net/graph-rewriting-automata/introduction.html)** by Paul Cousin — An exploration of rule-based graph evolution.
-   **[Growing Graphs](https://arxiv.org/abs/2211.13619)** (*arXiv:2211.13619*) — Research into the structural properties and evolution of procedural graph systems.

## 🛠 Tech Stack

-   **Core Logic**: JavaScript (ES6+)
-   **Physics Engine**: WebAssembly (compiled from C via `zig build-exe`)
-   **Rendering**: [SwissGL](https://github.com/google/swissgl) (WebGL 2.0)
-   **Post-Processing**: Custom Bloom filter and GLSL color mapping.

## 🏗 Build Instructions

To compile the WebAssembly core:

```bash
bash build.sh
```

Requires the **[Zig](https://ziglang.org)** compiler to targeting `wasm32-freestanding`.

## 📂 Project Structure

-   `main.c`: WASM physics core (Octree, Barnes-Hut, Link Forces).
-   `force.js`: Interface between JS and WASM physics.
-   `graph.js`: Graph management and rewriting logic.
-   `app.js`: High-level application controller and autonomous behavior.
-   `index.html`: Interactive simulation viewer.
-   `alice.html`: Autonomous generative art mode.

## 🤖 AI Development Assistance

This project was developed with the assistance of **Gemini**, which helped in:
- Porting complex graph and octree logic from JavaScript to high-performance WebAssembly (C).
- Architecting and optimizing the dual-tree Barnes-Hut algorithm for $O(N \log N)$ force calculations.
- Refactoring the codebase into a modular, reusable architecture.

## 📄 License

This project is licensed under the **MIT License**. See the `LICENSE` file for details.
