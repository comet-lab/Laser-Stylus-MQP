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

        //Activate UI elements
        this.ui.overlay.classList.add('active'); 
        this.ui.previewPopup.classList.add('active');
        this.ui.previewBtn.disabled = true;
        this.ui.previewTimeDisplay.textContent = "Calculating...";

        // Set the video source for the background
        if (this.ui.video.srcObject) {
            this.ui.previewVideo.srcObject = this.ui.video.srcObject;
            this.ui.previewVideo.play().catch(e => console.warn("Preview play error", e));
        }

        //Perform layout calculation
        this.updateLayout();

        //Fetch data and start
        await this.updatePreviewData();
        this.ui.previewBtn.disabled = false;
    }

    public closePreview(): void {
        this.ui.previewPopup.classList.remove('active');
        this.ui.overlay.classList.remove('active'); 
        this.ui.previewVideo.srcObject = null;
        this.stopAnimation();
    }

    /**
     * Resizes and positions the Preview Window.
     * Enforces aspect ratio of the video.
     * Calculates position based on the prepare menu's right-margin.
     */
    private updateLayout(): void {
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

        // Round to integers to ensure pixel-perfect canvas alignment
        viewportW = Math.floor(viewportW);
        viewportH = Math.floor(viewportH);

        //Apply dimensions to DOM
        //The popup gets (viewport height + header height)
        this.ui.previewPopup.style.left = `${targetLeft}px`;
        this.ui.previewPopup.style.top = `${prepareRect.top}px`;
        this.ui.previewPopup.style.width = `${viewportW}px`;
        this.ui.previewPopup.style.height = `${viewportH + headerHeight}px`;

        //Update internal canvas resolution
        //Matches the viewport exactly
        this.ui.previewCanvas.width = viewportW;
        this.ui.previewCanvas.height = viewportH;

        //Calculate Scale Factors
        //Now caleY maps video -> viewport (excluding header)
        this.scaleX = viewportW / videoW;
        this.scaleY = viewportH / videoH;

        //Update marker size
        const baseMarkerSize = 24; 
        let scaledSize = Math.max(8, baseMarkerSize * (viewportW / 1920));
        
        //Snap size to an EVEN INTEGER. 
        //An odd size (e.g. 13px) means the center is at 6.5px.
        scaledSize = Math.round(scaledSize);
        if (scaledSize % 2 !== 0) scaledSize += 1;

        //Update local state so the loop knows the exact size
        this.markerSize = scaledSize;

        this.ui.previewMarker.style.width = `${this.markerSize}px`;
        this.ui.previewMarker.style.height = `${this.markerSize}px`;
    }

    public async updatePreviewData(): Promise<void> {
        const cm = this.getCanvasManager();
        if (!cm) return;

        this.stopAnimation();

        if (this.ui.previewPopup.classList.contains('active')) {
            this.updateLayout();
        }

        const speed = parseFloat(this.ui.speedInput.value) || 10;
        const density = this.state.fillEnabled ? parseFloat(this.ui.rasterDensityInput.value) : 10;
        const rasterType = this.state.selectedRasterPattern || '';
        const isFill = this.state.fillEnabled;

        try {
            //Get snapshot of the original drawing (Fabric canvas only)
            const snapshotUrl = cm.getCanvasDataURL();
            const snapshotImg = new Image();
            
            //Load snapshot and fetch simulation path in parallel
            const [_, response] = await Promise.all([
                new Promise(resolve => {
                    snapshotImg.onload = resolve;
                    snapshotImg.src = snapshotUrl;
                }),
                cm.getPreviewPath(speed, rasterType, density, isFill)
            ]);
            
            this.pathData = response.path;
            this.durationSeconds = response.duration;
            this.ui.previewTimeDisplay.textContent = `Est. Time: ${this.durationSeconds.toFixed(1)}s`;

            //Draw Snapshot (Original) + Path (Overlay)
            this.drawPathOverlay(snapshotImg); 
            this.startAnimation();
        } catch (e) {
            console.error(e);
            this.ui.previewTimeDisplay.textContent = "Error";
        }
    }

    private drawPathOverlay(backgroundImage?: HTMLImageElement): void {
        const ctx = this.ui.previewCanvas.getContext('2d');
        if (!ctx) return; 

        ctx.clearRect(0, 0, this.ui.previewCanvas.width, this.ui.previewCanvas.height);
        
        // Draw original user drawing (Scaled to fit viewport)
        if (backgroundImage) {
            ctx.drawImage(backgroundImage, 0, 0, this.ui.previewCanvas.width, this.ui.previewCanvas.height);
        }

        if (this.pathData.length < 2) return;

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

        //Manual centering calculation
        //top/left = center - (size / 2)
        //This avoids CSS transform ambiguity and ensures integer alignment
        const offset = this.markerSize / 2;
        
        this.ui.previewMarker.style.left = `${domX - offset}px`;
        this.ui.previewMarker.style.top = `${domY - offset}px`;

        this.animationFrameId = requestAnimationFrame(() => this.loop());
    }
}