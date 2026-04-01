//frontend/src/features/drawing/PreviewManager.ts

import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { Position } from '../../ui/types';
import { ToastManager } from '../../ui/ToastManager';

/**
 * PreviewManager - CAD-style toolpath overlay with animated follower
 */
export class PreviewManager {
  private ctx: CanvasRenderingContext2D;
  private isPreviewActive: boolean = false;
  private ignoreWebsocketPaths: boolean = false;
  //sourcePath is the raw pixel data sent from robot
  private sourcePath: Position[] = [];
  private pathData: Position[] = [];
  private durationSeconds: number = 0;
  private hasPreviewedCurrentDrawing: boolean = false;

  // Animation state
  private isAnimating: boolean = false;
  private animationStartTime: number = 0;
  private animationFrameId: number | null = null;

  // Visual styling
  private readonly PATH_GLOW_COLOR = 'rgba(255, 215, 0, 0.4)';
  private readonly PATH_GLOW_WIDTH = 8;
  private readonly PATH_OUTLINE_COLOR = '#00000080';
  private readonly PATH_OUTLINE_WIDTH = 5;
  private readonly PATH_COLOR = '#FFFF00';
  private readonly PATH_WIDTH = 2;         
  private readonly DASH_PATTERN = [10, 10];
  
  private dashOffset: number = 0;
  private requiredConfirmations: number = 0;

  constructor(
    private readonly ui: UIRegistry,
    private readonly state: AppState,
    private getCanvasManager: () => CanvasManager | null,
  ) {
    const ctx = this.ui.previewOverlay.getContext('2d');
    if (!ctx) throw new Error('Preview overlay context not available');
    this.ctx = ctx;

    this.updateOverlaySize();

    //Disable execute button immediately when clicked
    const executeBtn = this.getExecuteBtn();
    if (executeBtn) {
      executeBtn.addEventListener('click', () => {
        this.setExecuteButtonState(false);
      });
    }
  }

  private getExecuteBtn(): HTMLButtonElement | null {
    return document.getElementById('executeBtn') as HTMLButtonElement | null;
  }

  /**
   * Handle the complex state of the execute button (Disabled prop + CSS styles)
   */
  private setExecuteButtonState(isEnabled: boolean): void {
    const btn = this.getExecuteBtn();
    if (!btn) return;

    if (isEnabled) {
      btn.classList.remove('locked');
    } else {
      btn.classList.add('locked');
    }
  }

  private tryEnableExecute(): void {
    //Only unlock if we have a real path drawn and the user has acknowledged all warnings
    if (this.sourcePath.length >= 2 && this.requiredConfirmations <= 0) {
      this.setExecuteButtonState(true);
    } else {
      this.setExecuteButtonState(false);
    }
  }

  public togglePreview(enable: boolean): void {
    this.isPreviewActive = enable;

    const btnText = this.ui.executeBtn.querySelector('.btn-text');

    if (enable) {
      this.ui.previewToggleOn.classList.add('active');
      this.ui.previewToggleOff.classList.remove('active');
      this.ui.previewInfoPanel.classList.add('open');
      this.updateOverlaySize();
      this.refreshPreview();
      if (btnText) btnText.textContent = 'EXECUTE';
    } else {
      this.ui.previewToggleOn.classList.remove('active');
      this.ui.previewToggleOff.classList.add('active');
      this.ui.previewInfoPanel.classList.remove('open');
      this.clearOverlay();
      this.stopAnimation();
    }
  }

  public async refreshPreview(): Promise<void> {
    if (!this.isPreviewActive) return;

    const cm = this.getCanvasManager();
    if (!cm) return;

    const speed = parseFloat(this.ui.speedInput.value) || 10;
    const density = this.state.fillEnabled
      ? parseFloat(this.ui.rasterDensityInput.value)
      : 0;
    const rasterType = 'line_raster'; //TODO: Remove entirely, unless we want to add more raster patterns in the future.
    const isFill = this.state.fillEnabled;

    this.ui.previewDuration.textContent = 'Computing...';

    //Disable execute while computing new path
    this.setExecuteButtonState(false);
    this.stopAnimation();
    ToastManager.clearAll();

    this.requiredConfirmations = 0;

    try {
      const response = await cm.previewPath(speed, rasterType, density, isFill);
      const hasValidPath = response.path && response.path.length > 0;

      if (hasValidPath) {
        this.ignoreWebsocketPaths = true;
      }

      //Handle Active Safety Warnings (Now checking the array of warnings)
      if (response.warnings) {
        if (response.warnings.includes("FIXTURE_OVERLAP")) {
          this.requiredConfirmations++;
          ToastManager.show(
            "SAFETY WARNING: Your planned path crosses into a restricted fixture zone. Please confirm to allow execution.",
            {
              type: 'warning',
              requireAck: true,
              ackText: 'CONFIRM',
              onAcknowledge: () => {
                this.requiredConfirmations--;
                this.tryEnableExecute();
              }
            }
          );
        }
        if (response.warnings.includes("PATH_ESCAPES_BOUNDS")) {
          this.requiredConfirmations++;
          ToastManager.show(
            "PRECISION WARNING: The generated path extends outside your originally drawn boundaries. Please confirm to allow execution.",
            {
              type: 'warning',
              requireAck: true,
              ackText: 'CONFIRM',
              onAcknowledge: () => {
                this.requiredConfirmations--;
                this.tryEnableExecute();
              }
            }
          );
        }
      }
      
      //Handle Safe Paths
      if (hasValidPath) {
        this.handlePathData(response.path, response.duration);
      }
      else {
        this.ui.previewDuration.textContent = 'Waiting...';
      }

    } catch (e: any) {
      if (e.message === "OUT_OF_BOUNDS") {
        ToastManager.show(
          "HARD STOP: Part of your shape is off the screen. Please move it fully inside the camera view.",
          { type: 'error', requireAck: true, ackText: 'DISMISS' }
        );
        this.ui.previewDuration.textContent = 'Out of Bounds';
        this.clearOverlay();
        this.setExecuteButtonState(false);
      } else {
        console.error('Preview refresh error:', e);
        this.ui.previewDuration.textContent = 'Error';
        this.clearOverlay();
      }
    }
  }

  public handlePathFromWebSocket(previewData: { x: number[], y: number[] }, serverDuration?: number): void {
    if (!this.isPreviewActive) return;

    if (this.ignoreWebsocketPaths) {
      console.log("Simulated HTTP path is active. Ignoring WS path echo.");
      return;
    }

    const newLength = Math.min(previewData.x.length, previewData.y.length);
    if (serverDuration === undefined && this.sourcePath.length === newLength) {
      console.log("Ignored duplicate path from robot.");
      return;
    }

    const path: Position[] = [];
    const len = Math.min(previewData.x.length, previewData.y.length);
    for (let i = 0; i < len; i++) {
      path.push({ x: previewData.x[i], y: previewData.y[i] });
    }

    const finalDuration = serverDuration || (this.durationSeconds > 0 ? this.durationSeconds : 10);
    const cm = this.getCanvasManager();

    if (cm && cm.checkIfPathEscapes(path)) {
      this.requiredConfirmations++;
      ToastManager.show(
        "SAFETY WARNING: The generated path extends outside your originally drawn boundaries. Please confirm to allow execution.",
        {
          type: 'warning',
          requireAck: true,
          ackText: 'CONFIRM',
          onAcknowledge: () => {
            this.requiredConfirmations--;
            this.tryEnableExecute();
          }
        }
      );
    }

    // Always process the path data, regardless of warnings
    this.handlePathData(path, finalDuration);
  }

  private handlePathData(videoPath: Position[], duration: number): void {
    //Catch empty or single-point paths before they break the renderer
    if (videoPath.length < 2) {
      this.clearOverlay();
      this.setExecuteButtonState(false); //Force lock the execute button
      this.ui.previewDuration.textContent = '--';
      return;
    }

    this.sourcePath = videoPath;
    this.durationSeconds = duration;
    this.hasPreviewedCurrentDrawing = true;

    this.updatePathTransform();

    this.ui.previewDuration.textContent = `${duration.toFixed(1)}s`;

    this.drawPath();
    this.startAnimation();

    this.tryEnableExecute();
  }

  /**
   * Recalculates `this.pathData` (screen pixels) from `this.sourcePath` (video pixels)
   * based on the current canvas dimensions.
   */
  private updatePathTransform(): void {
    if (this.sourcePath.length === 0) return;

    const video = this.ui.video;
    const viewport = this.ui.viewport;

    //Guard against divide by zero if video isn't loaded yet
    const vWidth = video.videoWidth || 1;
    const vHeight = video.videoHeight || 1;

    //Use viewport dimensions (CSS pixels) for calculating scale
    const scaleX = viewport.offsetWidth / vWidth;
    const scaleY = viewport.offsetHeight / vHeight;

    this.pathData = [];
    let lastX = -9999;
    let lastY = -9999;

    for (const p of this.sourcePath) {
      //Snap to half-pixels for crisp canvas stroke rendering
      const x = Math.floor(p.x * scaleX) + 0.5;
      const y = Math.floor(p.y * scaleY) + 0.5;

      //Filter points that are extremely close to each other
      //Dashed lines look terribly noisy if segments are smaller than a pixel
      const distSq = (x - lastX) * (x - lastX) + (y - lastY) * (y - lastY);
      
      if (distSq > 2.0) { //Only record the point if it moved at least ~1.4 pixels
        this.pathData.push({ x, y });
        lastX = x;
        lastY = y;
      }
    }
    
    //Ensure the absolute last point is always included to match the exact end position
    if (this.sourcePath.length > 0) {
      const lastSrc = this.sourcePath[this.sourcePath.length - 1];
      const finalX = Math.floor(lastSrc.x * scaleX) + 0.5;
      const finalY = Math.floor(lastSrc.y * scaleY) + 0.5;
      
      if (this.pathData.length === 0) {
        this.pathData.push({ x: finalX, y: finalY });
      } else {
        this.pathData[this.pathData.length - 1] = { x: finalX, y: finalY };
      }
    }
  }

  private drawPath(): void {
    this.clearOverlay();

    if (this.pathData.length < 2) return;

    const ctx = this.ctx;
    ctx.save();

    //Draw glow
    ctx.strokeStyle = this.PATH_GLOW_COLOR;
    ctx.lineWidth = this.PATH_GLOW_WIDTH;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.setLineDash([]);
    this.drawPathLine(ctx);

    //Draw translucent background line
    ctx.strokeStyle = this.PATH_OUTLINE_COLOR;
    ctx.lineWidth = this.PATH_OUTLINE_WIDTH;
    this.drawPathLine(ctx);

    //Draw dashed bright foreground line
    ctx.strokeStyle = this.PATH_COLOR;
    ctx.lineWidth = this.PATH_WIDTH;
    ctx.setLineDash(this.DASH_PATTERN);
    ctx.lineDashOffset = this.dashOffset;
    this.drawPathLine(ctx);

    ctx.restore();
  }

  private drawPathLine(ctx: CanvasRenderingContext2D): void {
    ctx.beginPath();
    ctx.moveTo(this.pathData[0].x, this.pathData[0].y);
    for (let i = 1; i < this.pathData.length; i++) {
      ctx.lineTo(this.pathData[i].x, this.pathData[i].y);
    }
    ctx.stroke();
  }

  private startAnimation(): void {
    if (this.pathData.length === 0 || this.durationSeconds <= 0) return;

    this.stopAnimation();
    this.isAnimating = true;
    this.animationStartTime = performance.now();
    this.ui.previewMarker.style.display = 'block';
    this.animationLoop();
  }

  private stopAnimation(): void {
    this.isAnimating = false;
    if (this.animationFrameId) {
      cancelAnimationFrame(this.animationFrameId);
      this.animationFrameId = null;
    }
    this.ui.previewMarker.style.display = 'none';
  }

  private animationLoop(): void {
    if (!this.isAnimating || this.pathData.length === 0) return;

    const now = performance.now();
    const elapsed = (now - this.animationStartTime) / 1000;
    let progress = elapsed / this.durationSeconds;

    if (progress >= 1) {
      this.animationStartTime = performance.now();
      progress = 0;
    }

    const totalPoints = this.pathData.length;
    const index = Math.min(
      Math.floor(progress * totalPoints),
      totalPoints - 1
    );

    //Update the DOM marker
    const point = this.pathData[index];
    this.ui.previewMarker.style.left = `${point.x}px`;
    this.ui.previewMarker.style.top = `${point.y}px`;

    //Change offset to modify speed
    this.dashOffset -= 0.1; 
    this.drawPath(); 

    this.animationFrameId = requestAnimationFrame(() => this.animationLoop());
  }

  private clearOverlay(): void {
    this.ctx.save();
    //Reset transform to identity to clear the raw physical pixels
    this.ctx.setTransform(1, 0, 0, 1, 0, 0);
    this.ctx.clearRect(0, 0, this.ui.previewOverlay.width, this.ui.previewOverlay.height);
    this.ctx.restore();
  }

  /**
   * Called on window resize. Updates canvas size and re-projects the path.
   */
  public updateOverlaySize(): void {
    const canvas = this.ui.previewOverlay;
    const viewport = this.ui.viewport;
    
    //Grab the device's pixel ratio (defaults to 1 for standard displays)
    const dpr = window.devicePixelRatio || 1;

    //Match Canvas bitmap size to DOM, accounting for high-DPI displays
    canvas.width = viewport.offsetWidth * dpr;
    canvas.height = viewport.offsetHeight * dpr;
    
    //Scale the internal drawing context to match CSS logical pixels
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    //Re-calculate path positions for new size
    if (this.sourcePath.length > 0) {
      this.updatePathTransform();

      if (this.isPreviewActive) {
        this.drawPath();
      }
    }
  }

  public hasPreviewedCurrent(): boolean {
    return this.hasPreviewedCurrentDrawing;
  }

  public resetPreviewState(): void {
    this.requiredConfirmations = 0;
    this.hasPreviewedCurrentDrawing = false;
    this.ignoreWebsocketPaths = false;
    this.pathData = [];
    this.sourcePath = [];
    this.durationSeconds = 0;
    this.stopAnimation();
    this.clearOverlay();
    this.ui.previewDuration.textContent = '--';

    // Disable execute button
    this.setExecuteButtonState(false);
    this.ui.previewToggleOff.click();
  }

  public dispose(): void {
    this.stopAnimation();
    this.clearOverlay();
  }
}