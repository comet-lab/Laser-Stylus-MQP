import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { Position } from '../../ui/types';

export class PreviewManager {
    private isAnimating: boolean = false;
    private startTime: number = 0;
    private pathData: Position[] = [];
    private durationSeconds: number = 0;
    private animationFrameId: number | null = null;
    
    //Exact scaling factor (Video -> Preview)
    private scaleX: number = 1;
    private scaleY: number = 1;

    //Store marker size to perform manual centering in the loop
    private markerSize: number = 24;

    constructor(
        private readonly ui: UIRegistry,
        private readonly state: AppState,
        private getCanvasManager: () => CanvasManager | null,
    ) {
        //Force border-box and remove CSS transform to give JS full control
        this.ui.previewMarker.style.boxSizing = 'border-box';
        this.ui.previewMarker.style.transform = 'none';
        this.ui.previewMarker.style.margin = '0';
    }

    public async openPreview(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        //Activate UI elements FIRST so they have dimensions in the DOM
        this.ui.overlay.classList.add('active'); 
        this.ui.previewPopup.classList.add('active');

        //Set the video source for the background immediately
        if (this.ui.video.srcObject) {
            this.ui.previewVideo.srcObject = this.ui.video.srcObject;
            this.ui.previewVideo.muted = true;
            
            try {
                await this.ui.previewVideo.play();
            } catch (e) {
                console.warn("Preview video play error:", e);
            }
        }

        //Perform layout calculation NOW that elements are visible
        this.updateLayout();

        //Update status and disable button while fetching
        this.ui.previewTimeDisplay.textContent = "Requesting...";
        this.ui.previewBtn.disabled = true;

        //Fetch data and start
        await this.updatePreviewData();
    }

    public closePreview(): void {
        this.ui.previewPopup.classList.remove('active');
        this.ui.overlay.classList.remove('active');
        
        //Stop the video stream to save resources
        this.ui.previewVideo.srcObject = null;
        
        this.stopAnimation();

        // Re-enable the simulate button so the user can try again
        this.ui.previewBtn.disabled = false;
    }

    private updateLayout(): void {
        const videoW = this.ui.video.videoWidth || 1;
        const videoH = this.ui.video.videoHeight || 1;
        const videoRatio = videoW / videoH;

        //Measure actual header height from DOM
        const header = this.ui.previewPopup.querySelector('.preview-header');
        const headerHeight = header ? header.getBoundingClientRect().height : 60;

        //Calculate positioning based on prepare popup
        const prepareRect = this.ui.preparePopup.getBoundingClientRect();
        const sidebarWidth = 65.5; 
        const rightGap = window.innerWidth - prepareRect.right;
        const targetLeft = sidebarWidth + rightGap;

        //Calculate available width for viewport content
        const availableWidth = prepareRect.left - targetLeft - rightGap;
        
        //Start with maximum width, calculate height maintaining aspect ratio
        let viewportW = availableWidth;
        let viewportH = viewportW / videoRatio;

        //Check vertical bounds (must fit in window with header)
        const maxViewportHeight = window.innerHeight - (rightGap * 2) - headerHeight;
        
        if (viewportH > maxViewportHeight) {
            //Height-constrained: recalculate width from max height
            viewportH = maxViewportHeight;
            viewportW = viewportH * videoRatio;
        }

        //Round to integers for pixel-perfect alignment
        viewportW = Math.floor(viewportW);
        viewportH = Math.floor(viewportH);

        //Apply dimensions to popup (viewport + header)
        this.ui.previewPopup.style.left = `${targetLeft}px`;
        this.ui.previewPopup.style.top = `${prepareRect.top}px`;
        this.ui.previewPopup.style.width = `${viewportW}px`;
        this.ui.previewPopup.style.height = `${viewportH + headerHeight}px`;

        //Set canvas internal resolution to match viewport exactly
        this.ui.previewCanvas.width = viewportW;
        this.ui.previewCanvas.height = viewportH;

        //Calculate scale factors for coordinate transformation
        this.scaleX = viewportW / videoW;
        this.scaleY = viewportH / videoH;

        //Update marker size with even integer snapping
        const baseMarkerSize = 24; 
        let scaledSize = Math.max(8, baseMarkerSize * (viewportW / 1920));
        
        //Force even number to prevent sub-pixel centering issues
        scaledSize = Math.round(scaledSize);
        if (scaledSize % 2 !== 0) scaledSize += 1;

        this.markerSize = scaledSize;
        this.ui.previewMarker.style.width = `${this.markerSize}px`;
        this.ui.previewMarker.style.height = `${this.markerSize}px`;
    }

    public async updatePreviewData(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        this.stopAnimation();

        //Ensure layout is valid before processing
        if (this.ui.previewPopup.classList.contains('active')) {
            this.updateLayout();
        }

        const speed = parseFloat(this.ui.speedInput.value) || 10;
        const density = this.state.fillEnabled ? parseFloat(this.ui.rasterDensityInput.value) : 10;
        const rasterType = this.state.selectedRasterPattern || '';
        const isFill = this.state.fillEnabled;

        try {
            //Send request to backend
            //In REAL mode, this returns an empty path. In FAKE mode, it returns the path.
            const response = await cm.previewPath(speed, rasterType, density, isFill);
            
            if (response.path && response.path.length > 0) {
                // --- FAKE MODE DETECTED (Data came back in HTTP response) ---
                this.pathData = response.path;
                this.durationSeconds = response.duration;
                this.ui.previewTimeDisplay.textContent = `Est. Time: ${this.durationSeconds.toFixed(1)}s`;
                this.ui.previewBtn.disabled = false;
                
                this.drawPathOverlay(); 
                this.startAnimation();
            } else {
                // --- REAL MODE (No data in HTTP response) ---
                // We must wait for the WebSocket to deliver 'path_preview'
                this.ui.previewTimeDisplay.textContent = "Computing (Waiting for Robot)...";
                // Keep button disabled until WS data arrives
                this.ui.previewBtn.disabled = true;
            }

        } catch (e) {
            console.error(e);
            this.ui.previewTimeDisplay.textContent = "Error";
            this.ui.previewBtn.disabled = false;
        }
    }

    /**
     * Called by AppController when a 'path_preview' message arrives via WebSocket.
     * format: { x: [1,2...], y: [3,4...] } (Structure of Arrays)
     */
    public handlePathFromWebSocket(previewData: { x: number[], y: number[] }): void {
        if (!this.ui.previewPopup.classList.contains('active')) return;

        // Convert SOA (Structure of Arrays) to AOS (Array of Objects)
        const path: Position[] = [];
        const len = Math.min(previewData.x.length, previewData.y.length);
        
        for (let i = 0; i < len; i++) {
            path.push({ x: previewData.x[i], y: previewData.y[i] });
        }

        if (path.length === 0) return;

        this.pathData = path;
        
        // Estimate duration based on simple distance calc (since robot didn't send duration)
        const speed = parseFloat(this.ui.speedInput.value) || 10;
        let totalDistance = 0;
        for (let i = 1; i < path.length; i++) {
            const dx = path[i].x - path[i-1].x;
            const dy = path[i].y - path[i-1].y;
            totalDistance += Math.sqrt(dx*dx + dy*dy);
        }
        
        // Arbitrary scaling for duration display if speed is abstract
        // If speed is mm/s and pixels are converted, this needs unit math. 
        // For now, assuming pixels and 'speed' factor:
        this.durationSeconds = totalDistance / (speed * 10 || 1); 

        this.ui.previewTimeDisplay.textContent = `Ready`;
        this.ui.previewBtn.disabled = false;

        this.drawPathOverlay();
        this.startAnimation();
    }

    private drawPathOverlay(): void {
        const ctx = this.ui.previewCanvas.getContext('2d');
        if (!ctx) return; 

        // Clear canvas completely so the video element behind it shows through
        ctx.clearRect(0, 0, this.ui.previewCanvas.width, this.ui.previewCanvas.height);
        
        if (this.pathData.length < 2) return;

        ctx.beginPath();
        // Cyan path matching the robot's intent
        ctx.strokeStyle = '#00ffff'; 
        ctx.lineWidth = 2; 
        ctx.lineJoin = 'round';
        ctx.lineCap = 'round';

        const start = this.pathData[0];
        ctx.moveTo(start.x * this.scaleX, start.y * this.scaleY);

        for (let i = 1; i < this.pathData.length; i++) {
            const p = this.pathData[i];
            ctx.lineTo(p.x * this.scaleX, p.y * this.scaleY);
        }
        ctx.stroke();
    }

    private startAnimation(): void {
        this.isAnimating = true;
        this.startTime = performance.now();
        this.ui.previewMarker.style.display = 'block';
        this.loop();
    }

    private stopAnimation(): void {
        this.isAnimating = false;
        if (this.animationFrameId) cancelAnimationFrame(this.animationFrameId);
        this.ui.previewMarker.style.display = 'none';
    }

    private loop(): void {
        if (!this.isAnimating || this.pathData.length === 0) return;

        const now = performance.now();
        const elapsed = (now - this.startTime) / 1000; 

        // Avoid division by zero
        const duration = this.durationSeconds > 0 ? this.durationSeconds : 1;

        let progress = elapsed / duration;
        if (progress >= 1) {
            this.startTime = performance.now();
            progress = 0;
        }

        const totalPoints = this.pathData.length;
        const index = Math.min(Math.floor(progress * totalPoints), totalPoints - 1);
        const point = this.pathData[index];

        const domX = point.x * this.scaleX;
        const domY = point.y * this.scaleY;

        // Manual centering calculation
        const offset = this.markerSize / 2;
        
        this.ui.previewMarker.style.left = `${domX - offset}px`;
        this.ui.previewMarker.style.top = `${domY - offset}px`;

        this.animationFrameId = requestAnimationFrame(() => this.loop());
    }
}