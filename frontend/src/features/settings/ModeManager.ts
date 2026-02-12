//frontend/src/features/settings/ModeManager.ts

import { UIRegistry }    from '../../core/UIRegistry';
import { AppState }      from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { ShapeType }     from '../../ui/types';

/**
 * ModeManager
 *
 * Handles switching between the three top-level application tabs
 * (Drawing ↔ Thermal ↔ Fixtures) and toggling between Real-Time and
 * Batch processing modes.  Coordinates teardown of the old mode and
 * setup of the new one, including CanvasManager method calls and
 * UI visibility toggling.
 */
export class ModeManager {
    constructor(
        private readonly ui:            UIRegistry,
        private readonly state:         AppState,
        private getCanvasManager:       () => CanvasManager | null,
        private updateDrawButtonState:  () => void,
        private updateFixturesButtonState: () => void,
        private updateThermalButtonState:  () => void,
    ) {}

    // ---------------------------------------------------------------
    // Real-time ↔ Batch toggle
    // ---------------------------------------------------------------

    /**
     * Called when the processing-mode switch changes.
     * Hides/shows batch-only UI elements and resets tool selections.
     */
    toggleMode(): void {
        const isRealTime = this.ui.processingModeSwitch.checked;
        const cm = this.getCanvasManager();

        if (isRealTime) {
            this.ui.batchUiElements.forEach(el => el.classList.add('hidden-mode'));
            this.ui.realTimeUIElements.forEach(el => el.classList.remove('hidden-mode'));
            cm?.disableDrawing();
        } else {
            this.ui.batchUiElements.forEach(el => el.classList.remove('hidden-mode'));
            this.ui.realTimeUIElements.forEach(el => el.classList.add('hidden-mode'));
            cm?.disableDrawing();
        }

        // Reset tool selections so state stays consistent
        this.ui.toggleButtons.forEach(btn => {
            btn.classList.remove('selected');
            btn.disabled = false;
        });
        this.state.selectedShape   = null;
        this.state.drawnShapeType  = null;
        this.updateDrawButtonState();
    }

    // ---------------------------------------------------------------
    // Top-level mode switcher (Drawing / Thermal / Fixtures)
    // ---------------------------------------------------------------

    /**
     * Tears down the current mode, activates the requested mode, and
     * reconciles both the UI and CanvasManager state.
     *
     * @param modeId  – the `id` attribute of the mode button that was clicked
     *                  (e.g. 'drawingBtn', 'thermalBtn', 'fixturesBtn').
     */
    switchMode(modeId: string): void {
        console.log('Switching to mode:', modeId);
        const cm = this.getCanvasManager();

        // ==============================================================
        // TEARDOWN – clean up whatever mode we are leaving
        // ==============================================================
        if (this.state.currentMode === 'fixtures' && modeId !== 'fixturesBtn') {
            if (cm && cm.canApplyFixtures()) {
                cm.clearFixtures();
                this.state.selectedBrushType  = null;
                this.state.isEraserActive     = false;
                this.ui.roundBrushBtn.classList.remove('selected');
                this.ui.squareBrushBtn.classList.remove('selected');
                this.ui.eraserBrushBtn.classList.remove('selected');
            }
        }
        if (this.state.currentMode === 'thermal' && modeId !== 'thermalBtn') {
            cm?.disableMarkerMode();
            this.ui.markerBtn.classList.remove('selected');
        }

        // ==============================================================
        // SETUP – activate the new mode
        // ==============================================================
        switch (modeId) {
            case 'fixturesBtn':
                this.activateFixturesMode(cm);
                break;
            case 'thermalBtn':
                this.activateThermalMode(cm);
                break;
            default:                            // 'drawingBtn' or any unknown
                this.activateDrawingMode(cm);
                break;
        }
    }

    // ---------------------------------------------------------------
    // Private – per-mode activation helpers
    // ---------------------------------------------------------------

    private activateFixturesMode(cm: CanvasManager | null): void {
        this.state.currentMode = 'fixtures';

        // UI visibility
        this.ui.drawingTools.classList.add('hidden');
        this.ui.thermalTools.classList.add('hidden');
        this.ui.fixturesTools.classList.remove('hidden');
        this.ui.thermalUiElements.forEach(el  => el.classList.add('hidden'));
        this.ui.drawingUiElements.forEach(el  => el.classList.add('hidden'));
        this.ui.realTimeUIElements.forEach(el => el.classList.add('hidden-mode'))
        this.ui.fixturesUiElements.forEach(el => el.classList.remove('hidden'));

        // CanvasManager state
        cm?.disableDrawing();
        cm?.enableFixturesMode();
        this.updateFixturesButtonState();
    }

    private activateThermalMode(cm: CanvasManager | null): void {
        this.state.currentMode = 'thermal';

        cm?.showMarkers();
        this.updateThermalButtonState();

        // UI visibility
        this.ui.drawingTools.classList.add('hidden');
        this.ui.thermalTools.classList.remove('hidden');
        this.ui.fixturesTools.classList.add('hidden');
        this.ui.drawingUiElements.forEach(el  => el.classList.add('hidden'));
        this.ui.realTimeUIElements.forEach(el => el.classList.add('hidden-mode'))
        this.ui.fixturesUiElements.forEach(el => el.classList.add('hidden'));
        this.ui.thermalUiElements.forEach(el  => el.classList.remove('hidden'));

        // CanvasManager state
        cm?.disableFixturesMode();
        cm?.disableDrawing();
        if (cm?.hasFixtures()) cm.showFixtures();   // keep fixtures visible but non-editable

        // Restore marker tool if it was already selected before we left thermal
        if (this.state.selectedShape === 'marker') {
            this.ui.markerBtn.classList.add('selected');
            cm?.enableMarkerMode();
        }
    }

    private activateDrawingMode(cm: CanvasManager | null): void {
        const isRealTime = this.ui.processingModeSwitch.checked;
        this.state.currentMode = 'drawing';

        // UI visibility
        this.ui.drawingTools.classList.remove('hidden');
        this.ui.thermalTools.classList.add('hidden');
        this.ui.fixturesTools.classList.add('hidden');
        this.ui.heatAreaBtn.classList.remove('selected');
        this.ui.markerBtn.classList.remove('selected');
        this.ui.drawingUiElements.forEach(el  => el.classList.remove('hidden'));
        if (isRealTime) {
            this.ui.realTimeUIElements.forEach(el => el.classList.remove('hidden-mode'))
        }
        this.ui.fixturesUiElements.forEach(el => el.classList.add('hidden'));
        this.ui.thermalUiElements.forEach(el  => el.classList.add('hidden'));

        // CanvasManager state
        cm?.disableMarkerMode();
        cm?.disableHeatAreaMode();
        cm?.disableFixturesMode();
        if (cm?.hasFixtures()) cm.showFixtures();

        // Restore drawing state that was active before we left drawing mode
        if (this.state.drawnShapeType) {
            // A shape already exists on canvas – lock tools, highlight the right button
            cm?.disableDrawing();
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            this.highlightShapeBtn(this.state.drawnShapeType);
        } else if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
            // A tool is selected but nothing drawn yet – re-enable drawing
            cm?.setShapeType(this.state.selectedShape);
            cm?.enableDrawing();
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
            this.highlightShapeBtn(this.state.selectedShape);
        } else {
            cm?.disableDrawing();
            this.ui.toggleButtons.forEach(btn => btn.classList.remove('selected'));
        }

        this.updateDrawButtonState();
    }

    // ---------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------

    /** Visually marks the button that corresponds to the active shape tool. */
    highlightShapeBtn(shape: ShapeType): void {
        const map: Record<string, HTMLButtonElement> = {
            freehand: this.ui.penBtn,
            square:   this.ui.squareBtn,
            circle:   this.ui.circleBtn,
            triangle: this.ui.triangleBtn,
            line:     this.ui.lineBtn,
        };
        map[shape]?.classList.add('selected');
    }
}