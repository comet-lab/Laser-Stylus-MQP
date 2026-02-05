//frontend/src/features/drawing/ExecutionManager.ts

import { UIRegistry }    from '../../core/UIRegistry';
import { AppState }      from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';

/**
 * ExecutionManager
 *
 * Owns the lifecycle that begins once the user has finished drawing a shape:
 *   1. onShapeComplete  – records that a shape now exists (triggers button-state update)
 *   2. executePath      – reads speed / fill settings, sends path to backend, cleans up
 *   3. clearDrawing     – removes the shape and re-arms the selected tool
 */
export class ExecutionManager {
    constructor(
        private readonly ui:                  UIRegistry,
        private readonly state:               AppState,
        private getCanvasManager:             () => CanvasManager | null,
        private updateDrawButtonState:        () => void,
    ) {}

    // ---------------------------------------------------------------
    // Shape-complete callback  (wired into CanvasManager construction)
    // ---------------------------------------------------------------

    /**
     * Called by CanvasManager the moment the user lifts the pointer after
     * drawing a shape.  Records the type so that button-state logic knows
     * a shape exists.
     */
    onShapeComplete(): void {
        if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.state.drawnShapeType = this.state.selectedShape;
            this.updateDrawButtonState();
        }
    }

    // ---------------------------------------------------------------
    // Execute
    // ---------------------------------------------------------------

    /**
     * Validates inputs, sends the path to the backend via CanvasManager,
     * and cleans up UI state on success.
     */
    async executePath(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        const speed   = parseFloat(this.ui.speedInput.value);
        const density = this.state.fillEnabled
            ? parseFloat(this.ui.rasterDensityInput.value)
            : 0;

        if (isNaN(speed) || speed <= 0) {
            alert('Invalid speed');
            return;
        }

        this.ui.executeBtn.disabled = true;
        this.ui.prepareBtn.disabled = true;

        try {
            await cm.executePath(
                speed,
                String(this.state.selectedRasterPattern),
                density,
                this.state.fillEnabled,
            );

            // Success – clean up
            cm.clearDrawing();
            this.state.drawnShapeType = null;
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
            this.updateDrawButtonState();

            // Re-arm the drawing tool that was active before execution
            if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
                cm.setShapeType(this.state.selectedShape);
                cm.enableDrawing();
            }

            this.ui.preparePopup.classList.remove('active');
        } catch (e) {
            console.error(e);
            this.ui.executeBtn.disabled = false;
            this.ui.prepareBtn.disabled = false;
        }
    }

    // ---------------------------------------------------------------
    // Clear
    // ---------------------------------------------------------------

    /**
     * Removes the current shape from the canvas and immediately re-arms
     * the selected tool so the user can draw again without extra clicks.
     */
    clearDrawing(): void {
        const cm = this.getCanvasManager();

        cm?.clearDrawing();
        this.state.drawnShapeType = null;
        this.updateDrawButtonState();

        // Re-arm drawing if a shape tool is still selected
        if (cm && this.state.selectedShape && this.state.selectedShape !== 'marker') {
            cm.setShapeType(this.state.selectedShape);
            cm.enableDrawing();
        }
    }
}