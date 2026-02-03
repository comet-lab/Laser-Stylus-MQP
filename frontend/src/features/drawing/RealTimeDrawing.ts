//frontend/src/features/drawing/RealTimeDrawing.ts

import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { WebSocketHandler } from '../../services/WebSocketHandler';

/**
 * RealTimeDrawing
 *
 * Owns the pointer-event handlers (down / move / up / cancel) that are active
 * during Real-Time streaming mode, plus the high-frequency requestAnimationFrame
 * loop that pushes normalised coordinates to the server.
 *
 * The loop is intentionally separate from the video-render loop so that input
 * latency is decoupled from frame-decode latency.
 */
export class RealTimeDrawing {
    constructor(
        private readonly ui: UIRegistry,
        private readonly state: AppState,
        private readonly ws: WebSocketHandler,
    ) { }

    // ---------------------------------------------------------------
    // Pointer handlers – wired up by the main controller
    // ---------------------------------------------------------------

    handleStart(e: PointerEvent): void {
        // Only active when the real-time switch is ON and the pen tool is selected
        if (!this.ui.processingModeSwitch.checked || this.state.selectedShape !== 'freehand') return;

        e.preventDefault();
        this.ui.canvas.setPointerCapture(e.pointerId);   // keep tracking even if pointer leaves canvas

        this.state.isRealTimeDrawing = true;
        this.state.latestRealTimePos = this.getCanvasCoordinates(e.clientX, e.clientY);

        // Notify the server a new stroke is beginning
        this.ws.updateState({ pathEvent: 'start' });

        this.runLoop();
    }

    handleMove(e: PointerEvent): void {
        if (!this.state.isRealTimeDrawing) return;
        e.preventDefault();
        // Store the latest position; the loop will consume it on next tick
        this.state.latestRealTimePos = this.getCanvasCoordinates(e.clientX, e.clientY);
    }

    handleEnd(e: PointerEvent): void {
        if (!this.state.isRealTimeDrawing) return;
        e.preventDefault();
        this.ui.canvas.releasePointerCapture(e.pointerId);

        this.state.isRealTimeDrawing = false;
        this.state.latestRealTimePos = null;

        this.ws.updateState({ pathEvent: 'end' });
    }

    // ---------------------------------------------------------------
    // High-frequency coordinate loop
    // ---------------------------------------------------------------

    /**
     * Runs once per animation frame while the user is drawing.
     * Converts screen-space canvas coordinates → video-space coordinates
     * (the server works in the original video resolution) and pushes them.
     */
    private runLoop(): void {
        if (!this.state.isRealTimeDrawing) return;

        if (this.state.latestRealTimePos) {
            const rect = this.ui.canvas.getBoundingClientRect();

            const normalizedX = this.state.latestRealTimePos.x / rect.width;
            const normalizedY = this.state.latestRealTimePos.y / rect.height;

            const vidX = normalizedX * this.ui.video.videoWidth;
            const vidY = normalizedY * this.ui.video.videoHeight;

            this.ws.updateState({ x: vidX, y: vidY });
        }

        requestAnimationFrame(() => this.runLoop());
    }

    // ---------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------

    /** Converts client (page) coordinates to canvas-local coordinates. */
    private getCanvasCoordinates(clientX: number, clientY: number) {
        const rect = this.ui.canvas.getBoundingClientRect();
        return { x: clientX - rect.left, y: clientY - rect.top };
    }
}