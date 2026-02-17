//frontend/src/features/drawing/ExecutionManager.ts

import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { PreviewManager } from '../drawing/PreviewManager';
import { ToastManager } from '../../ui/ToastManager';


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
            ToastManager.show('Please preview the path first', 'warning');
            return;
        }

        const isRobotOn = this.ui.robotBtn.classList.contains('active');

        if (!isRobotOn) {
            this.showInlineRobotWarning(cm, previewMgr);
            return;
        }

        await this.performExecution(cm, previewMgr);
    }

    private showInlineRobotWarning(cm: CanvasManager, previewMgr: PreviewManager): void {
        if (document.getElementById('inline-robot-warning')) return;

        // Capture exact pixel width of the Cancel button
        const targetWidth = this.ui.prepareCancelBtn.offsetWidth;

        this.ui.executeBtn.style.display = 'none';

        //Create Container
        const warningDiv = document.createElement('div');
        warningDiv.id = 'inline-robot-warning';
        // Keep only the dynamic width calculation inline
        warningDiv.style.width = `${targetWidth}px`;

        //Create Text
        const text = document.createElement('span');
        text.className = 'inline-warning-text';
        text.textContent = 'Robot is OFF';

        //Create Button
        const turnOnBtn = document.createElement('button');
        turnOnBtn.className = 'inline-warning-btn';
        turnOnBtn.textContent = 'Turn On & Start';

        warningDiv.appendChild(text);
        warningDiv.appendChild(turnOnBtn);

        this.ui.executeBtn.parentNode?.insertBefore(warningDiv, this.ui.executeBtn);

        // --- Cancel Interceptor ---
        const cancelInterceptor = (e: MouseEvent) => {
            e.stopImmediatePropagation();
            e.preventDefault();
            cleanup();
        };

        this.ui.prepareCancelBtn.addEventListener('click', cancelInterceptor, true);

        const closeInterceptor = () => cleanup();
        this.ui.prepareCloseBtn.addEventListener('click', closeInterceptor);

        const cleanup = () => {
            const warningEl = document.getElementById('inline-robot-warning');
            if (warningEl) warningEl.remove();

            this.ui.executeBtn.style.display = '';
            this.updateExecuteButtonState();

            this.ui.prepareCancelBtn.removeEventListener('click', cancelInterceptor, true);
            this.ui.prepareCloseBtn.removeEventListener('click', closeInterceptor);
        };

        turnOnBtn.onclick = async () => {
            cleanup();
            this.ui.robotBtn.click();

            setTimeout(() => {
                this.performExecution(cm, previewMgr);
            }, 500);
        };
    }

    private async performExecution(cm: CanvasManager, previewMgr: PreviewManager): Promise<void> {
        this.ui.executeBtn.disabled = true;

        try {
            await cm.executeCommand();
            cm.clearDrawing();
            this.state.drawnShapeType = null;

            previewMgr.resetPreviewState();

            this.updateExecuteButtonState();
            this.ui.toggleButtons.forEach(btn => btn.disabled = false);
            this.updateDrawButtonState();

            this.ui.preparePopup.classList.remove('active');

            if (this.state.selectedShape && this.state.selectedShape !== 'marker') {
                cm.setShapeType(this.state.selectedShape);
                cm.enableDrawing();
            }
        } catch (e) {
            console.error(e);
            ToastManager.show("Execution command failed.", "error");
            this.ui.executeBtn.disabled = false;
        }
    }

    clearDrawing(): void {
        const cm = this.getCanvasManager();
        cm?.clearDrawing();
        this.state.drawnShapeType = null;

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

        // Also clean up the warning if the execute button state is forcibly updated 
        const warningDiv = document.getElementById('inline-robot-warning');
        if (warningDiv) {
            warningDiv.remove();
            this.ui.executeBtn.style.display = '';
        }

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