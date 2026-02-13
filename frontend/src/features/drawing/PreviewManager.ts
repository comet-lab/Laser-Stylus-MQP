import { UIRegistry } from '../../core/UIRegistry';
import { AppState } from '../../core/AppState';
import { CanvasManager } from '../../ui/CanvasManager';
import { Position } from '../../ui/types';

/**
 * PreviewManager - CAD-style toolpath overlay with animated follower
 */
export class PreviewManager {
  private ctx: CanvasRenderingContext2D;
  private isPreviewActive: boolean = false;
  // "Source of Truth": The path in raw video coordinates
  private sourcePath: Position[] = [];
  private pathData: Position[] = [];
  private durationSeconds: number = 0;
  private hasPreviewedCurrentDrawing: boolean = false;

  // Animation state
  private isAnimating: boolean = false;
  private animationStartTime: number = 0;
  private animationFrameId: number | null = null;

  // Visual styling
  private readonly PATH_COLOR = '#ffff00'; // Neon Yellow
  private readonly PATH_WIDTH = 4;
  private readonly DASH_PATTERN = [12, 6];
  private readonly GLOW_COLOR = 'rgba(255, 255, 0, 0.4)';
  private readonly GLOW_WIDTH = 10;

  constructor(
    private readonly ui: UIRegistry,
    private readonly state: AppState,
    private getCanvasManager: () => CanvasManager | null,
  ) {
    const ctx = this.ui.previewOverlay.getContext('2d');
    if (!ctx) throw new Error('Preview overlay context not available');
    this.ctx = ctx;

    this.updateOverlaySize();

    // Disable execute button immediately when clicked
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
      btn.disabled = false;
      // MUST override the inline styles set by ExecutionManager
      btn.style.pointerEvents = 'auto';
      btn.style.opacity = '1';
    } else {
      btn.disabled = true;
      btn.style.pointerEvents = 'none';
      btn.style.opacity = '0.3';
    }
  }

  public togglePreview(enable: boolean): void {
    this.isPreviewActive = enable;

    if (enable) {
      this.ui.previewToggleOn.classList.add('active');
      this.ui.previewToggleOff.classList.remove('active');
      this.ui.previewInfoPanel.classList.add('open');
      this.updateOverlaySize();
      this.refreshPreview();
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
    const rasterType = this.state.selectedRasterPattern || '';
    const isFill = this.state.fillEnabled;

    this.ui.previewDuration.textContent = 'Computing...';

    // Disable execute while computing new path
    this.setExecuteButtonState(false);
    this.stopAnimation();

    try {
      const response = await cm.previewPath(speed, rasterType, density, isFill);

      if (response.path && response.path.length > 0) {
        this.handlePathData(response.path, response.duration);
      } else {
        this.ui.previewDuration.textContent = 'Waiting...';
      }
    } catch (e) {
      console.error('Preview refresh error:', e);
      this.ui.previewDuration.textContent = 'Error';
      this.clearOverlay();
    }
  }

  public handlePathFromWebSocket(previewData: { x: number[], y: number[] }): void {
    if (!this.isPreviewActive) return;

    const path: Position[] = [];
    const len = Math.min(previewData.x.length, previewData.y.length);
    for (let i = 0; i < len; i++) {
      path.push({ x: previewData.x[i], y: previewData.y[i] });
    }

    // Estimate duration
    const speed = parseFloat(this.ui.speedInput.value) || 10;
    let totalDistance = 0;
    for (let i = 1; i < path.length; i++) {
      const dx = path[i].x - path[i - 1].x;
      const dy = path[i].y - path[i - 1].y;
      totalDistance += Math.sqrt(dx * dx + dy * dy);
    }
    const duration = totalDistance / (speed * 10 || 1);

    this.handlePathData(path, duration);
  }

  private handlePathData(videoPath: Position[], duration: number): void {
    if (videoPath.length === 0) {
      this.clearOverlay();
      return;
    }

    // 1. STORE SOURCE OF TRUTH (Video Coordinates)
    this.sourcePath = videoPath;
    this.durationSeconds = duration;
    this.hasPreviewedCurrentDrawing = true;

    // 2. CALCULATE SCREEN COORDINATES
    this.updatePathTransform();

    this.ui.previewDuration.textContent = `${duration.toFixed(1)}s`;
    this.setExecuteButtonState(true);

    this.drawPath();
    this.startAnimation();
  }

  /**
   * Recalculates `this.pathData` (screen pixels) from `this.sourcePath` (video pixels)
   * based on the current canvas dimensions.
   */
  private updatePathTransform(): void {
    if (this.sourcePath.length === 0) return;

    const video = this.ui.video;
    const canvas = this.ui.previewOverlay;

    // Guard against divide by zero if video isn't loaded yet
    const vWidth = video.videoWidth || 1;
    const vHeight = video.videoHeight || 1;

    const scaleX = canvas.width / vWidth;
    const scaleY = canvas.height / vHeight;

    this.pathData = this.sourcePath.map(p => ({
      x: p.x * scaleX,
      y: p.y * scaleY
    }));
  }

  private drawPath(): void {
    this.clearOverlay();

    if (this.pathData.length < 2) return;

    const ctx = this.ctx;
    ctx.save();

    // Draw glow
    ctx.strokeStyle = this.GLOW_COLOR;
    ctx.lineWidth = this.GLOW_WIDTH;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.setLineDash([]);
    this.drawPathLine(ctx);

    // Draw main path
    ctx.strokeStyle = this.PATH_COLOR;
    ctx.lineWidth = this.PATH_WIDTH;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.setLineDash(this.DASH_PATTERN);
    this.drawPathLine(ctx);

    this.drawDirectionArrows(ctx);

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

  private drawDirectionArrows(ctx: CanvasRenderingContext2D): void {
    const arrowSpacing = 80;
    let accumulatedDist = 0;

    ctx.fillStyle = this.PATH_COLOR;
    ctx.setLineDash([]);

    for (let i = 1; i < this.pathData.length; i++) {
      const p1 = this.pathData[i - 1];
      const p2 = this.pathData[i];

      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const dist = Math.sqrt(dx * dx + dy * dy);

      accumulatedDist += dist;

      if (accumulatedDist >= arrowSpacing) {
        accumulatedDist = 0;

        const angle = Math.atan2(dy, dx);
        const arrowSize = 10;

        ctx.save();
        ctx.translate(p2.x, p2.y);
        ctx.rotate(angle);

        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(-arrowSize, -arrowSize / 2);
        ctx.lineTo(-arrowSize, arrowSize / 2);
        ctx.closePath();
        ctx.fill();

        ctx.restore();
      }
    }
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
    const point = this.pathData[index];

    // Convert canvas coordinates to DOM viewport coordinates
    const canvasRect = this.ui.previewOverlay.getBoundingClientRect();
    const scaleX = canvasRect.width / this.ui.previewOverlay.width;
    const scaleY = canvasRect.height / this.ui.previewOverlay.height;

    const domX = point.x * scaleX;
    const domY = point.y * scaleY;

    this.ui.previewMarker.style.left = `${domX}px`;
    this.ui.previewMarker.style.top = `${domY}px`;

    this.animationFrameId = requestAnimationFrame(() => this.animationLoop());
  }

  private clearOverlay(): void {
    this.ctx.clearRect(0, 0, this.ui.previewOverlay.width, this.ui.previewOverlay.height);
  }

  /**
   * Called on window resize. Updates canvas size and re-projects the path.
   */
  public updateOverlaySize(): void {
    const canvas = this.ui.previewOverlay;
    const viewport = this.ui.viewport;

    // 1. Match Canvas to DOM
    canvas.width = viewport.offsetWidth;
    canvas.height = viewport.offsetHeight;

    // 2. Re-calculate path positions for new size
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
    this.hasPreviewedCurrentDrawing = false;
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