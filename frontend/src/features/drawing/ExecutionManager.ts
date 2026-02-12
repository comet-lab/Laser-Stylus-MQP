//frontend/src/core/ExecutionManager.ts

import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';

/**
 * ExecutionManager
 *
 * Owns the lifecycle:
 * 1. onShapeComplete  – records that a shape now exists
 * 2. previewPath      – sends data, opens preview window
 * 3. executePath      – sends execute command only (requires preview first)
 * 4. clearDrawing     – resets everything
 */
export class ExecutionManager {
    private isPathPrepared: boolean = false;

    constructor(
        private readonly ui: UIRegistry,
        private readonly state: AppState,
        private getCanvasManager: () => CanvasManager | null,
        private updateDrawButtonState: () => void,
        private openPreviewWindow: () => void,
    ) { 
        this.updateExecuteButtonState();
    }

    /**
     * Called by CanvasManager when the user finishes drawing a shape.
     */
    onShapeComplete(): void {
        if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.state.drawnShapeType = this.state.selectedShape;

            // New shape drawn -> Invalidate previous path
            this.isPathPrepared = false;

            this.updateExecuteButtonState();
            this.updateDrawButtonState();
        }
    }

    /**
     * "Simulate" Action:
     * 1. Sends path data to backend.
     * 2. Opens the preview window to show the animation.
     * 3. Enables the Execute button upon success.
     */
    async previewPath(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        const speed = parseFloat(this.ui.speedInput.value);
        const density = this.state.fillEnabled
            ? parseFloat(this.ui.rasterDensityInput.value)
            : 0;

        if (isNaN(speed) || speed <= 0) {
            alert('Invalid speed');
            return;
        }

        // Disable controls while calculating
        this.ui.previewBtn.disabled = true;
        this.ui.executeBtn.disabled = true;

        try {
            // Send all data to backend via preview endpoint
            await cm.previewPath(
                speed,
                String(this.state.selectedRasterPattern),
                density,
                this.state.fillEnabled,
            );

            // Success: Path is now safe to execute
            this.isPathPrepared = true;
            this.updateExecuteButtonState();

            this.ui.previewBtn.disabled = false;

            // Open the preview window to show the path animation
            this.openPreviewWindow();

        } catch (e) {
            console.error(e);
            this.ui.previewBtn.disabled = false;
            this.isPathPrepared = false;
            this.updateExecuteButtonState();
        }
    }

    /**
     * "Execute" Action:
     * Sends execute command to start the prepared path.
     * Only works if path has been previewed first (Safety Interlock).
     */
    async executePath(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        if (!this.isPathPrepared) {
            alert('Please simulate the path first');
            return;
        }

        this.ui.executeBtn.disabled = true;
        this.ui.previewBtn.disabled = true;

        try {
            await cm.executeCommand();

            cm.clearDrawing();
            this.state.drawnShapeType = null;
            this.isPathPrepared = false;
            this.updateExecuteButtonState();

            //Re-enable UI
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
            this.updateDrawButtonState();

            //Explicitly re-enable the simulate button so it can be used again
            this.ui.previewBtn.disabled = false;

            //Close the Prepare and Preview popups
            this.ui.preparePopup.classList.remove('active');
            this.ui.previewCloseBtn.click();

            // 4. Re-arm the tool
            if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
                cm.setShapeType(this.state.selectedShape);
                cm.enableDrawing();
            }

        } catch (e) {
            console.error(e);
            this.ui.executeBtn.disabled = false;
            this.ui.previewBtn.disabled = false;
        }
    }

    /**
     * Removes the current shape from the canvas and resets state.
     */
    clearDrawing(): void {
        const cm = this.getCanvasManager();

        cm?.clearDrawing();
        this.state.drawnShapeType = null;
        this.isPathPrepared = false;

        this.updateExecuteButtonState();
        this.updateDrawButtonState();

        // Re-arm drawing if a shape tool is still selected
        if (cm && this.state.selectedShape && this.state.selectedShape !== 'marker') {
            cm.setShapeType(this.state.selectedShape);
            cm.enableDrawing();
        }
    }

    /**
     * Updates the execute button visual state based on preparation.
     */
    private updateExecuteButtonState(): void {
        if (this.isPathPrepared) {
            this.ui.executeBtn.disabled = false;
            this.ui.executeBtn.style.pointerEvents = 'auto'; // Ensure clickable
            this.ui.executeBtn.style.opacity = '1';
        } else {
            this.ui.executeBtn.disabled = true;
            this.ui.executeBtn.style.pointerEvents = 'none'; // Ensure NOT clickable
            this.ui.executeBtn.style.opacity = '0.3';
        }
    }
}