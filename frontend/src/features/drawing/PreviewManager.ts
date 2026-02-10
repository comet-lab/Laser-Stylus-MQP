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
    
    // Exact scaling factor (Video -> Preview)
    private scaleX: number = 1;
    private scaleY: number = 1;

    constructor(
        private readonly ui: UIRegistry,
        private readonly state: AppState,
        private getCanvasManager: () => CanvasManager | null,
    ) {}

    public async openPreview(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        //Activate UI elements
        this.ui.overlay.classList.add('active'); 
        this.ui.previewPopup.classList.add('active');
        this.ui.previewBtn.disabled = true;
        this.ui.previewTimeDisplay.textContent = "Calculating...";

        //Perform layout calculation
        this.updateLayout(cm);

        //Fetch data and start
        await this.updatePreviewData();
        this.ui.previewBtn.disabled = false;
    }

    public closePreview(): void {
        this.ui.previewPopup.classList.remove('active');
        this.ui.overlay.classList.remove('active'); 
        this.stopAnimation();
    }

    /**
     * Resizes and positions the Preview Window.
     * Enforces aspect ratio of the video.
     * Calculates position based on the prepare menu's right-margin.
     */
    private updateLayout(cm: CanvasManager): void {
        const videoW = this.ui.video.videoWidth || 1;
        const videoH = this.ui.video.videoHeight || 1;
        const videoRatio = videoW / videoH;

        //Measure header height
        // e need to query the header inside the popup to know how much extra space to reserve
        const header = this.ui.previewPopup.querySelector('.preview-header');
        const headerHeight = header ? header.getBoundingClientRect().height : 60; // Fallback to 60 if hidden

        //Calculate positioning (Same as before)
        const prepareRect = this.ui.preparePopup.getBoundingClientRect();
        const sidebarWidth = 65.5; 
        const rightGap = window.innerWidth - prepareRect.right;
        const targetLeft = sidebarWidth + rightGap;

        //Calculate available width for the content (Canvas/video)
        const availableWidth = prepareRect.left - targetLeft - rightGap;
        
        //Calculate target dimensions for the viewport only
        let viewportW = availableWidth;
        let viewportH = viewportW / videoRatio;

        //Check vertical bounds (Window height - margins - header height)
        const maxViewportHeight = window.innerHeight - (rightGap * 2) - headerHeight;
        
        if (viewportH > maxViewportHeight) {
            viewportH = maxViewportHeight;
            viewportW = viewportH * videoRatio;
        }

        // 5. Apply dimensions to DOM
        // The popup gets (viewport height + header height)
        this.ui.previewPopup.style.left = `${targetLeft}px`;
        this.ui.previewPopup.style.top = `${prepareRect.top}px`;
        this.ui.previewPopup.style.width = `${viewportW}px`;
        this.ui.previewPopup.style.height = `${viewportH + headerHeight}px`;

        //Update internal canvas resolution
        //Matches the viewport exactly
        this.ui.previewCanvas.width = viewportW;
        this.ui.previewCanvas.height = viewportH;

        //Update background snapshot
        const dataUrl = cm.getCanvasDataURL();
        const container = this.ui.previewCanvas.parentElement;
        if (container) {
            container.style.backgroundImage = `url(${dataUrl})`;
            // Force stretch to match main viewport behavior
            container.style.backgroundSize = '100% 100%'; 
        }

        //Calculate Scale Factors
        //Now caleY maps video -> viewport (excluding header)
        this.scaleX = viewportW / videoW;
        this.scaleY = viewportH / videoH;

        //Update marker size
        const baseMarkerSize = 24; 
        const scaledSize = Math.max(8, baseMarkerSize * (viewportW / 1920)); 
        this.ui.previewMarker.style.width = `${scaledSize}px`;
        this.ui.previewMarker.style.height = `${scaledSize}px`;
    }

    public async updatePreviewData(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        this.stopAnimation();

        if (this.ui.previewPopup.classList.contains('active')) {
            this.updateLayout(cm);
        }

        const speed = parseFloat(this.ui.speedInput.value) || 10;
        const density = this.state.fillEnabled ? parseFloat(this.ui.rasterDensityInput.value) : 10;
        const rasterType = this.state.selectedRasterPattern || '';
        const isFill = this.state.fillEnabled;

        try {
            const response = await cm.getPreviewPath(speed, rasterType, density, isFill);
            
            this.pathData = response.path;
            this.durationSeconds = response.duration;
            this.ui.previewTimeDisplay.textContent = `Est. Time: ${this.durationSeconds.toFixed(1)}s`;

            this.drawPathOverlay(); 
            this.startAnimation();
        } catch (e) {
            console.error(e);
            this.ui.previewTimeDisplay.textContent = "Error";
        }
    }

    private drawPathOverlay(): void {
        const ctx = this.ui.previewCanvas.getContext('2d');
        if (!ctx || this.pathData.length < 2) return;

        ctx.clearRect(0, 0, this.ui.previewCanvas.width, this.ui.previewCanvas.height);
        
        ctx.beginPath();
        //Cyan path matching the robot's intent
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

        let progress = elapsed / this.durationSeconds;
        if (progress >= 1) {
            this.startTime = performance.now();
            progress = 0;
        }

        const totalPoints = this.pathData.length;
        const index = Math.min(Math.floor(progress * totalPoints), totalPoints - 1);
        const point = this.pathData[index];

        const domX = point.x * this.scaleX;
        const domY = point.y * this.scaleY;

        this.ui.previewMarker.style.left = `${domX}px`;
        this.ui.previewMarker.style.top = `${domY}px`;

        this.animationFrameId = requestAnimationFrame(() => this.loop());
    }
}