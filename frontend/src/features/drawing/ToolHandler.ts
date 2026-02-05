//frontend/src/features/drawing/ToolHandler.ts

import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { ShapeType } from '../../ui/types';

/**
 * ToolHandler
 *
 * Manages every "tool" the user can activate:
 *   – Shape drawing tools  (pen, square, circle, triangle, line)
 *   – Thermal tools        (marker, heat-area)
 *   – Fixture brushes      (round, square, eraser)
 *   – Fixture actions      (clear, apply)
 *
 * Also owns the three button-state updaters that keep action buttons
 * enabled/disabled in sync with the current canvas & app state.
 */
export class ToolHandler {
    constructor(
        private readonly ui: UIRegistry,
        private readonly state: AppState,
        private getCanvasManager: () => CanvasManager | null,
    ) { }

    // ===================================================================
    // Shape tool selection
    // ===================================================================

    /**
     * Handles a click on one of the shape-tool buttons.
     * Enforces the constraint that Real-Time mode only allows freehand,
     * and that only one shape can exist on canvas at a time.
     */
    handleShapeSelection(button: HTMLButtonElement, shape: ShapeType | 'marker'): void {
        const cm = this.getCanvasManager();

        // Real-time mode only permits the pen
        if (this.ui.processingModeSwitch.checked && shape !== 'freehand') return;
        if (button.disabled) return;

        if (button.classList.contains('selected')) {
            // --- Deselect ---
            button.classList.remove('selected');
            this.state.selectedShape = null;
            cm?.disableDrawing();
            cm?.disableMarkerMode();
        } else {
            // --- Select ---
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            button.classList.add('selected');
            this.state.selectedShape = shape;

            if (this.ui.processingModeSwitch.checked) {
                // Real-time: clear any previous drawing immediately
                cm?.clearDrawing();
                this.state.drawnShapeType = null;
            } else {
                // Batch mode
                if (cm) {
                    if (shape === 'marker') {
                        cm.enableMarkerMode();
                    } else {
                        cm.setShapeType(shape);
                        cm.enableDrawing();
                    }
                }
            }
        }

        this.updateDrawButtonState();
    }

    // ===================================================================
    // Thermal tools
    // ===================================================================

    /** Activates the marker or heat-area tool, deactivating everything else first. */
    selectThermalTool(tool: 'marker' | 'heat'): void {
        const cm = this.getCanvasManager();

        const isMarkerActive = this.ui.markerBtn.classList.contains('selected');
        const isHeatActive = this.ui.heatAreaBtn.classList.contains('selected');

        if ((tool === 'marker' && isMarkerActive) || (tool === 'heat' && isHeatActive)) {
            // Turn everything off
            this.ui.markerBtn.classList.remove('selected');
            this.ui.heatAreaBtn.classList.remove('selected');
            cm?.disableMarkerMode();
            cm?.disableHeatAreaMode();
            this.state.selectedShape = null;
            return; // Stop here
        }

        this.ui.markerBtn.classList.remove('selected');
        this.ui.heatAreaBtn.classList.remove('selected');
        cm?.disableDrawing();
        cm?.disableMarkerMode();
        cm?.disableHeatAreaMode();

        if (tool === 'marker') {
            this.ui.markerBtn.classList.add('selected');
            cm?.enableMarkerMode();
            this.state.selectedShape = 'marker';
        } else {
            this.ui.heatAreaBtn.classList.add('selected');
            cm?.enableHeatAreaMode();
            this.state.selectedShape = null;
        }
    }

    // ===================================================================
    // Fixture brushes
    // ===================================================================

    handleBrushSelection(brushType: 'round' | 'square'): void {
        const cm = this.getCanvasManager();

        if (brushType === 'round' && this.ui.roundBrushBtn.disabled) return;
        if (brushType === 'square' && this.ui.squareBrushBtn.disabled) return;

        // Toggle off if the same brush is already active
        if (this.state.selectedBrushType === brushType) {
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;
            cm?.disableFixturesBrush();
        } else {
            // Activate the new brush, clear any other selection first
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.ui.eraserBrushBtn.classList.remove('selected');

            if (brushType === 'round') this.ui.roundBrushBtn.classList.add('selected');
            else this.ui.squareBrushBtn.classList.add('selected');

            this.state.selectedBrushType = brushType;
            this.state.isEraserActive = false;
            cm?.setFixturesBrush(brushType, parseInt(this.ui.brushSizeSlider.value), false);
        }

        this.updateFixturesButtonState();
    }

    handleEraserSelection(): void {
        const cm = this.getCanvasManager();
        if (this.ui.eraserBrushBtn.disabled) return;

        if (this.state.isEraserActive) {
            // Toggle off
            this.ui.eraserBrushBtn.classList.remove('selected');
            this.state.selectedBrushType = null;
            this.state.isEraserActive = false;
            cm?.disableFixturesBrush();
        } else {
            // Activate eraser (internally a round brush with the erase flag)
            this.ui.roundBrushBtn.classList.remove('selected');
            this.ui.squareBrushBtn.classList.remove('selected');
            this.ui.eraserBrushBtn.classList.add('selected');

            this.state.selectedBrushType = 'round';
            this.state.isEraserActive = true;
            cm?.setFixturesBrush('round', parseInt(this.ui.brushSizeSlider.value), true);
        }

        this.updateFixturesButtonState();
    }

    /** Forwards brush-size slider changes to CanvasManager while a brush is active. */
    handleBrushSizeChange(): void {
        if (this.state.selectedBrushType) {
            this.getCanvasManager()?.setFixturesBrush(
                this.state.selectedBrushType,
                parseInt(this.ui.brushSizeSlider.value),
                this.state.isEraserActive,
            );
        }
    }

    // ===================================================================
    // Fixture actions (clear / apply)
    // ===================================================================

    async clearFixtures(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        this.ui.clearBoundaryBtn.disabled = true;
        try {
            cm.clearFixtures();
            await cm.clearFixturesOnServer();
        } catch (e) {
            console.error(e);
        }

        this.resetBrushState(cm);
        this.updateFixturesButtonState();
    }

    async applyFixtures(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        this.ui.applyFixturesBtn.disabled = true;
        this.ui.clearBoundaryBtn.disabled = true;

        try {
            await cm.executeFixtures();
            this.resetBrushState(cm);
            this.updateFixturesButtonState();
        } catch (e) {
            console.error(e);
            this.ui.applyFixturesBtn.disabled = false;
            this.ui.clearBoundaryBtn.disabled = false;
        }
    }

    // ===================================================================
    // Button-state updaters  (called after any state change)
    // ===================================================================

    /**
     * Enables/disables drawing action buttons and locks shape-tool buttons
     * when a shape already exists on canvas (one-shape-at-a-time rule).
     */
    updateDrawButtonState(): void {
        const hasShape = this.state.drawnShapeType !== null;

        this.ui.clearBtn.disabled = !hasShape;
        this.ui.prepareBtn.disabled = !hasShape;
        this.ui.executeBtn.disabled = !hasShape;

        if (hasShape) {
            // Lock every tool except the one that produced the current shape
            this.ui.penBtn.disabled = this.state.drawnShapeType !== 'freehand';
            this.ui.squareBtn.disabled = this.state.drawnShapeType !== 'square';
            this.ui.circleBtn.disabled = this.state.drawnShapeType !== 'circle';
            this.ui.triangleBtn.disabled = this.state.drawnShapeType !== 'triangle';
            this.ui.lineBtn.disabled = this.state.drawnShapeType !== 'line';
        } else {
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
        }
    }

    /** Enables/disables fixture action buttons based on CanvasManager state. */
    updateFixturesButtonState(): void {
        const cm = this.getCanvasManager();
        const hasFixtures = cm?.hasFixtures() ?? false;
        const canApply = cm?.canApplyFixtures() ?? false;

        this.ui.clearBoundaryBtn.disabled = !hasFixtures;
        this.ui.applyFixturesBtn.disabled = !canApply;
        this.ui.roundBrushBtn.disabled = false;
        this.ui.squareBrushBtn.disabled = false;
        this.ui.eraserBrushBtn.disabled = !hasFixtures;
    }

    /** Enables/disables the "clear markers" button based on whether any markers exist. */
    updateThermalButtonState(): void {
        const cm = this.getCanvasManager();
        if (!cm) return;
        this.ui.clearMarkersBtn.disabled = !cm.hasMarkers();
    }

    // ===================================================================
    // Private helpers
    // ===================================================================

    /** Resets all brush-related UI and state after a clear or apply. */
    private resetBrushState(cm: CanvasManager): void {
        cm.disableFixturesBrush();
        this.state.selectedBrushType = null;
        this.state.isEraserActive = false;
        this.ui.roundBrushBtn.classList.remove('selected');
        this.ui.squareBrushBtn.classList.remove('selected');
        this.ui.eraserBrushBtn.classList.remove('selected');
    }
}