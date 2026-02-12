# Frontend (Vite + TypeScript)

This folder contains a minimal Vite + TypeScript frontend used for development and served in Docker.

How to run locally (without Docker):

1. cd frontend
2. npm install
3. npm run dev

How to build (Docker):

docker-compose up --build

The frontend will be available at http://localhost:3000 and calls the backend at http://localhost:443/media/mystream when you click the "Connect to backend" button.

---
<br>
<br>
<br>
<br>
<br>
<br>


# Frontend Developer Guide & Architecture

This repository follows a strict modular architecture to separate concerns between UI management, Canvas logic (Fabric.js), Hardware control, and Application State.

**Quick Rule of Thumb:**
* **Need a DOM element?** Add it to `UIRegistry.ts`.
* **Need to store data?** Add it to `AppState.ts`.
* **Drawing pixels/vectors?** Edit `CanvasManager.ts`.
* **Handling a button click?** Logic goes in a `features/` class, wiring goes in `AppController.ts`.

---

## Where Do I Implement X?

### 1. Adding New UI Elements (Buttons, Inputs, Modals)
**Do not** call `document.getElementById` inside feature classes.
1.  **`src/core/UIRegistry.ts`**: Add the element to the interface and the query selector in the `createUIRegistry` function.
2.  **`src/core/AppController.ts`**: Bind the event listener in `bindEvents()`.
3.  **Feature Class**: The actual logic for the interaction should live in the relevant feature class (e.g., `ToolHandler`, `SettingsManager`), not in the AppController.

### 2. Adding New Application State
**Do not** create local state variables inside classes unless they are strictly private to that class's internal calculation.
1.  **`src/core/AppState.ts`**: Define the property in the interface and initialize it in `createAppState`.
2.  **Usage**: This state object is injected into every controller. Access it via `this.state.propertyName`.

### 3. Canvas & Drawing Logic (Fabric.js)
All direct interaction with `fabric.js` happens here. No other file should import `fabric`.
* **`src/ui/CanvasManager.ts`**:
    * Adding new shapes (Square, Circle, etc.).
    * Handling mouse/touch interactions on the canvas.
    * Generating images/blobs for backend export.
    * Handling Thermal Markers and Fixture masks.

### 4. Hardware Interaction (Laser / Robot)
* **`src/features/hardware/HardwareController.ts`**:
    * Logic for toggling hardware on/off.
    * **Safety Interlocks**: Logic regarding mutual exclusion (e.g., turning off Robot if Laser is disabled) lives here.
    * Handling server ACKs and UI timeouts/locking.

### 5. Tooling & Buttons (Pens, Brushes, Modes)
* **`src/features/drawing/ToolHandler.ts`**:
    * Logic for what happens when a tool button is clicked.
    * Managing "Exclusive" tools (e.g., can't have Pen and Eraser active at once).
    * Updating button states (Disabled/Active) based on current canvas state.

### 6. Modes & Tabs
* **`src/features/settings/ModeManager.ts`**:
    * Switching between Drawing, Thermal, and Fixtures tabs.
    * Toggling between Real-Time and Batch processing.
    * **Cleanup**: Responsible for tearing down the old mode (hiding UI, disabling canvas interactions) before setting up the new one.

---

## Architecture Overview

### The "Thin" AppController
`AppController.ts` is the orchestrator. It should **not** contain business logic.
* **Responsibility**: Initialize sub-systems, wire dependencies, bind DOM events, and manage the WebSocket connection.
* **Dependency Injection**: It creates `state` and `ui` once and passes them into every sub-system.

### The Rendering Loop
We use a specific rendering strategy to avoid flickering:
1.  **HTML Video**: Sits at the bottom of the stack (handled by the browser).
2.  **HTML Canvas**: Sits on top (transparent).
3.  **The Loop**: `AppController` runs the loop. It **does not** draw the video to the canvas (which causes desync). It only tells `CanvasManager` to render the overlay shapes.

### Real-Time vs. Batch
* **Batch Mode**: Uses `CanvasManager.ts`. You draw a shape, it stays there. You click execute, the manager rasterizes it and sends it.
* **Real-Time Mode**: Uses `RealTimeDrawing.ts`. It bypasses Fabric.js for the input stream to reduce latency. It grabs raw pointer coordinates and transmits them over WebSocket immediately.

---

## Important Guidelines

### 1. The Persistence Rule
To ensure data isn't lost when switching tabs (modes):
* **Paths (Drawing Mode)**: Persist across all tabs until explicitly cleared via the "Clear" button or "Execute".
* **Fixtures**: Persist only if `Applied`. Once applied, they persist until cleared.
* **Heat Markers**: Persist until explicitly removed via the UI.
* **Implementation Note**: When switching modes in `ModeManager`, use `disableDrawing()` (which locks shapes and prevents interaction) rather than `clearDrawing()` (which deletes them).

### 2. Coordinate Systems
* **Client Space**: Mouse coordinates on the screen.
* **Canvas Space**: Internal Fabric coordinates.
* **Video Space**: The actual resolution of the camera stream.
* **Rule**: Always normalize coordinates before sending to the backend. Use `video.videoWidth` / `canvas.width` ratios.

### 3. Z-Index / Layering
* Heat Markers must always be on top of drawing paths.
* `CanvasManager.ensureMarkersOnTop()` is called automatically after every shape creation to enforce this.

---

## Common Tasks Cheat Sheet

**"I want to add a new shape tool (e.g., Hexagon)"**
1.  **HTML**: Add `<button id="hexBtn">` to HTML.
2.  **Registry**: Add `hexBtn` to `UIRegistry.ts`.
3.  **Types**: Add `'hexagon'` to `ShapeType` in `types.ts`.
4.  **CanvasManager**: Add a case for `hexagon` in `onMouseDown` to instantiate the Fabric object.
5.  **ToolHandler**: Add logic to `handleShapeSelection` and highlight the button.
6.  **AppController**: Bind the click event: `this.ui.hexBtn.addEventListener('click', ...)`

**"I want to display a new value from the WebSocket"**
1.  **AppController**: Inside `syncUiToState`, read the new property from `state` (the raw message).
2.  **UI**: Update the DOM element (retrieved via `this.ui`).
3.  **Logic**: If the value requires complex logic (e.g., a safety check), pass it to `HardwareController` or `AppState`.