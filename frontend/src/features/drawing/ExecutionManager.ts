import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { PreviewManager } from '../drawing/PreviewManager';

/**
 * ExecutionManager
 *
 * Owns the lifecycle:
 * 1. onShapeComplete – records that a shape now exists
 * 2. executePath     – sends execute command (requires preview first)
 * 3. clearDrawing    – resets everything
 */
export class ExecutionManager {
    constructor(
        private readonly ui: UIRegistry,
        private readonly state: AppState,
        private getCanvasManager: () => CanvasManager | null,
        private updateDrawButtonState: () => void,
        private getPreviewManager: () => PreviewManager,
    ) {
        this.updateExecuteButtonState();
    }

    /**
     * Called by CanvasManager when the user finishes drawing a shape.
     */
    onShapeComplete(): void {
        if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
            this.state.drawnShapeType = this.state.selectedShape;

            // New shape drawn -> Invalidate preview state
            const previewMgr = this.getPreviewManager();
            previewMgr.resetPreviewState();

            this.updateExecuteButtonState();
            this.updateDrawButtonState();
        }
    }

    async previewPath(): Promise<void> {
        const previewMgr = this.getPreviewManager();
        previewMgr.togglePreview(true);
    }

    /**
     * Sends execute command to start the prepared path.
     * Only works if path has been previewed first (Safety Interlock).
     */
    async executePath(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        const previewMgr = this.getPreviewManager();
        if (!previewMgr.hasPreviewedCurrent()) {
            alert('Please preview the path first');
            return;
        }

        this.ui.executeBtn.disabled = true;

        try {
            await cm.executeCommand();
            cm.clearDrawing();
            this.state.drawnShapeType = null;

            // Reset preview state on execution
            previewMgr.resetPreviewState();

            this.updateExecuteButtonState();
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
            this.updateDrawButtonState();

            // Close prepare popup
            this.ui.preparePopup.classList.remove('active');

            // Re-arm the tool
            if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
                cm.setShapeType(this.state.selectedShape);
                cm.enableDrawing();
            }
        } catch (e) {
            console.error(e);
            this.ui.executeBtn.disabled = false;
        }
    }

    clearDrawing(): void {
        const cm = this.getCanvasManager();
        cm?.clearDrawing();
        this.state.drawnShapeType = null;

        // Reset preview state on clear
        const previewMgr = this.getPreviewManager();
        previewMgr.resetPreviewState();

        this.updateExecuteButtonState();
        this.updateDrawButtonState();

        if (cm && this.state.selectedShape && this.state.selectedShape !== 'marker') {
            cm.setShapeType(this.state.selectedShape);
            cm.enableDrawing();
        }
    }

    private updateExecuteButtonState(): void {
        const previewMgr = this.getPreviewManager();
        const hasPreview = previewMgr.hasPreviewedCurrent();

        if (hasPreview) {
            this.ui.executeBtn.disabled = false;
            this.ui.executeBtn.style.pointerEvents = 'auto';
            this.ui.executeBtn.style.opacity = '1';
        } else {
            this.ui.executeBtn.disabled = true;
            this.ui.executeBtn.style.pointerEvents = 'none';
            this.ui.executeBtn.style.opacity = '0.3';
        }
    }
}