import { Position, ShapeType, Shape, HandleType, DragOffsets } from './types';
import * as Utils from './utils';

export class DrawingTracker {
    private canvas: HTMLCanvasElement;
    private video: HTMLVideoElement;
    private ctx: CanvasRenderingContext2D;
    private drawnPixels: Set<string>;
    private isDrawing: boolean = false;
    private drawingEnabled: boolean = false;
    
    // State
    private lastPos: Position | null = null;
    private startPos: Position | null = null;
    private currentPos: Position | null = null;
    private shapeType: ShapeType = 'freehand';
    private currentShape: Shape | null = null;
    
    // Edit Mode State
    private showHandles: boolean = false;
    private selectedHandle: HandleType = null;
    private dragOffsets: DragOffsets | null = null;

    // Off-screen canvas
    private drawingCanvas: HTMLCanvasElement;
    private drawingCtx: CanvasRenderingContext2D;

    // Config
    private apiBaseUrl: string;
    private onShapeCompleteCallback: (() => void) | null = null;
    private readonly HANDLE_SIZE = 8;
    private readonly HANDLE_HIT_AREA = 15;

    constructor(canvas: HTMLCanvasElement, video: HTMLVideoElement, apiBaseUrl: string = `http://${window.location.hostname}:443`) {
        this.canvas = canvas;
        this.video = video;
        this.ctx = canvas.getContext('2d')!;
        this.apiBaseUrl = apiBaseUrl;
        this.drawnPixels = new Set<string>();

        // Setup off-screen canvas
        this.drawingCanvas = document.createElement('canvas');
        this.drawingCanvas.width = canvas.width;
        this.drawingCanvas.height = canvas.height;
        this.drawingCtx = this.drawingCanvas.getContext('2d')!;
        this.configureCtx();

        this.setupEventListeners();
    }

    private configureCtx() {
        this.drawingCtx.strokeStyle = '#ff0000';
        this.drawingCtx.lineWidth = 5;
        this.drawingCtx.lineCap = 'round';
        this.drawingCtx.lineJoin = 'round';
    }

    private setupEventListeners(): void {
        this.canvas.addEventListener('pointerdown', this.handlePointerDown.bind(this));
        this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
        this.canvas.addEventListener('pointerup', this.handlePointerUp.bind(this));
        this.canvas.addEventListener('pointercancel', this.handlePointerCancel.bind(this));
    }

    // --- Coordinate Helpers ---

    private getPointerPos(e: PointerEvent): Position {
        const rect = this.canvas.getBoundingClientRect();
        const scaleX = this.canvas.width / rect.width;
        const scaleY = this.canvas.height / rect.height;
        return {
            x: Math.floor((e.clientX - rect.left) * scaleX),
            y: Math.floor((e.clientY - rect.top) * scaleY)
        };
    }

    private getHandleAtPosition(pos: Position): HandleType {
        if (!this.currentShape || !this.showHandles) return null;

        const handles = Utils.getHandlePositions(this.currentShape);
        if (!handles) return null;

        // 1. Check handle dots
        for (const [key, handlePos] of Object.entries(handles)) {
            if (Math.abs(pos.x - handlePos.x) <= this.HANDLE_HIT_AREA && 
                Math.abs(pos.y - handlePos.y) <= this.HANDLE_HIT_AREA) {
                return key as HandleType;
            }
        }

        // 2. Check for 'move' (click inside bounding box)
        const bbox = Utils.getLocalBoundingBox(this.currentShape);
        const center = { x: bbox.centerX, y: bbox.centerY };
        const unrotatedMouse = Utils.rotatePoint(pos, center, -this.currentShape.rotation);

        if (unrotatedMouse.x >= bbox.minX && unrotatedMouse.x <= bbox.maxX && 
            unrotatedMouse.y >= bbox.minY && unrotatedMouse.y <= bbox.maxY) {
            return 'move';
        }

        return null;
    }

    // --- Rendering Logic ---

    private drawShape(shape: Shape): void {
        const { type, startPos, endPos, rotation } = shape;
        this.configureCtx();

        if (type === 'circle') {
            const bbox = Utils.getLocalBoundingBox(shape);
            this.drawingCtx.save();
            this.drawingCtx.translate(bbox.centerX, bbox.centerY);
            this.drawingCtx.rotate(rotation);
            this.drawingCtx.beginPath();
            const radius = Math.min(bbox.width, bbox.height) / 2;
            this.drawingCtx.arc(0, 0, radius, 0, 2 * Math.PI); // Draw at 0,0 relative to translate
            this.drawingCtx.stroke();
            this.drawingCtx.restore();
            return;
        }

        if (type === 'line') {
            this.drawingCtx.beginPath();
            this.drawingCtx.moveTo(startPos.x, startPos.y);
            this.drawingCtx.lineTo(endPos.x, endPos.y);
            this.drawingCtx.stroke();
            return;
        }

        // Polygons (Square/Triangle)
        const vertices = Utils.getShapeVertices(shape);
        if (vertices.length === 0) return;

        this.drawingCtx.beginPath();
        this.drawingCtx.moveTo(vertices[0].x, vertices[0].y);
        for (let i = 1; i < vertices.length; i++) {
            this.drawingCtx.lineTo(vertices[i].x, vertices[i].y);
        }
        this.drawingCtx.closePath();
        this.drawingCtx.stroke();
    }

    private drawSelectionHandles(): void {
        if (!this.currentShape || !this.showHandles) return;
        const handles = Utils.getHandlePositions(this.currentShape);
        if (!handles) return;

        this.drawingCtx.save();
        this.drawingCtx.strokeStyle = '#0066ff';
        this.drawingCtx.lineWidth = 2;

        // Draw bounding box
        this.drawingCtx.setLineDash([5, 5]);
        this.drawingCtx.beginPath();
        this.drawingCtx.moveTo(handles.nw.x, handles.nw.y);
        this.drawingCtx.lineTo(handles.ne.x, handles.ne.y);
        this.drawingCtx.lineTo(handles.se.x, handles.se.y);
        this.drawingCtx.lineTo(handles.sw.x, handles.sw.y);
        this.drawingCtx.closePath();
        this.drawingCtx.stroke();

        // Connector to rotation handle
        if (handles.n && handles.rot) {
            this.drawingCtx.setLineDash([]);
            this.drawingCtx.beginPath();
            this.drawingCtx.moveTo(handles.n.x, handles.n.y);
            this.drawingCtx.lineTo(handles.rot.x, handles.rot.y);
            this.drawingCtx.stroke();
        }

        this.drawingCtx.fillStyle = '#ffffff';
        this.drawingCtx.setLineDash([]);

        // Draw Handles
        for (const [key, pos] of Object.entries(handles)) {
            this.drawingCtx.beginPath();
            if (key === 'rot') {
                this.drawingCtx.arc(pos.x, pos.y, this.HANDLE_SIZE / 2, 0, Math.PI * 2);
                this.drawingCtx.fill();
                this.drawingCtx.stroke();
            } else {
                this.drawingCtx.fillRect(pos.x - this.HANDLE_SIZE / 2, pos.y - this.HANDLE_SIZE / 2, this.HANDLE_SIZE, this.HANDLE_SIZE);
                this.drawingCtx.strokeRect(pos.x - this.HANDLE_SIZE / 2, pos.y - this.HANDLE_SIZE / 2, this.HANDLE_SIZE, this.HANDLE_SIZE);
            }
        }
        this.drawingCtx.restore();
    }

    private redraw(): void {
        this.drawingCtx.clearRect(0, 0, this.drawingCanvas.width, this.drawingCanvas.height);
        if (this.currentShape) {
            this.drawShape(this.currentShape);
            this.drawSelectionHandles();
        }
    }

    private addPixelsToSet(x0: number, y0: number, x1: number, y1: number): void {
        const pixelGenerator = Utils.generateLinePixels(x0, y0, x1, y1, 5);
        for (const pixel of pixelGenerator) {
            this.drawnPixels.add(`${pixel.x},${pixel.y}`);
        }
    }

    private finalizeShape(shape: Shape): void {
        this.drawnPixels.clear();
        const { type, startPos, endPos } = shape;

        if (type === 'line') {
            this.addPixelsToSet(startPos.x, startPos.y, endPos.x, endPos.y);
            return;
        }
        
        if (type === 'circle') {
            const bbox = Utils.getLocalBoundingBox(shape);
            const radius = Math.min(bbox.width, bbox.height) / 2;
            const steps = Math.max(120, Math.floor(radius * 2));
            let prevX = Math.floor(bbox.centerX + radius);
            let prevY = Math.floor(bbox.centerY);
            
            for (let i = 1; i <= steps; i++) {
                const angle = (i / steps) * 2 * Math.PI;
                const x = Math.floor(bbox.centerX + radius * Math.cos(angle));
                const y = Math.floor(bbox.centerY + radius * Math.sin(angle));
                this.addPixelsToSet(prevX, prevY, x, y);
                prevX = x;
                prevY = y;
            }
            return;
        }

        // Square/Triangle
        const vertices = Utils.getShapeVertices(shape);
        for (let i = 0; i < vertices.length; i++) {
            const p1 = vertices[i];
            const p2 = vertices[(i + 1) % vertices.length]; 
            this.addPixelsToSet(p1.x, p1.y, p2.x, p2.y);
        }
    }

    // --- Event Handlers ---

    private handlePointerDown(e: PointerEvent): void {
        if (!this.drawingEnabled) return;
        e.preventDefault();
        (e.target as HTMLElement).setPointerCapture(e.pointerId);

        const pos = this.getPointerPos(e);

        // 1. Try to select/manipulate existing shape
        if (this.showHandles && this.currentShape) {
            const handle = this.getHandleAtPosition(pos);
            if (handle) {
                this.selectedHandle = handle;
                if (handle === 'move') {
                    this.dragOffsets = {
                        start: { x: pos.x - this.currentShape.startPos.x, y: pos.y - this.currentShape.startPos.y },
                        end: { x: pos.x - this.currentShape.endPos.x, y: pos.y - this.currentShape.endPos.y }
                    };
                } 
                return;
            } else {
                // Clicked outside handles, deselect
                this.showHandles = false;
                this.redraw();
                return;
            }
        }

        // 2. Check if we clicked a shape to select it (if not currently selected)
        if (this.currentShape && !this.showHandles) {
            const handle = this.getHandleAtPosition(pos);
            if (handle === 'move') {
                this.showHandles = true;
                this.redraw();
                return;
            }
        }

        if (this.drawnPixels.size > 0) {
            return;
        }

        // 3. Start new drawing
        this.isDrawing = true;
        this.startPos = pos;
        this.lastPos = pos;
        this.currentPos = pos;

        if (this.shapeType === 'freehand') {
            this.drawingCtx.beginPath();
            this.drawingCtx.moveTo(pos.x, pos.y);
            this.addPixelsToSet(pos.x, pos.y, pos.x, pos.y);
            this.currentShape = null;
        }
    }

    private handlePointerMove(e: PointerEvent): void {
        if (!this.drawingEnabled) return;
        e.preventDefault();
        const pos = this.getPointerPos(e);

        // Handle Transformations
        if (this.selectedHandle && this.currentShape) {
            const bbox = Utils.getLocalBoundingBox(this.currentShape);
            const center = { x: bbox.centerX, y: bbox.centerY };

            if (this.selectedHandle === 'move' && this.dragOffsets) {
                this.currentShape.startPos.x = pos.x - this.dragOffsets.start.x;
                this.currentShape.startPos.y = pos.y - this.dragOffsets.start.y;
                this.currentShape.endPos.x = pos.x - this.dragOffsets.end.x;
                this.currentShape.endPos.y = pos.y - this.dragOffsets.end.y;
            } 
            else if (this.selectedHandle === 'rot') {
                const angle = Math.atan2(pos.y - center.y, pos.x - center.x);
                this.currentShape.rotation = angle + Math.PI / 2;
            }
            else {
                // Symmetric Resizing Logic
                const unrotatedPos = Utils.rotatePoint(pos, center, -this.currentShape.rotation);
                const halfWidth = Math.abs(unrotatedPos.x - center.x);
                const halfHeight = Math.abs(unrotatedPos.y - center.y);

                if (this.currentShape.type === 'circle') {
                      const radius = Math.max(halfWidth, halfHeight);
                      this.currentShape.startPos = { x: center.x - radius, y: center.y - radius };
                      this.currentShape.endPos = { x: center.x + radius, y: center.y + radius };
                } else {
                    if (['nw', 'ne', 'sw', 'se', 'e', 'w'].includes(this.selectedHandle)) {
                        this.currentShape.startPos.x = center.x - halfWidth;
                        this.currentShape.endPos.x = center.x + halfWidth;
                    }
                    if (['nw', 'ne', 'sw', 'se', 'n', 's'].includes(this.selectedHandle)) {
                        this.currentShape.startPos.y = center.y - halfHeight;
                        this.currentShape.endPos.y = center.y + halfHeight;
                    }
                }
            }
            this.redraw();
            return;
        }

        // Update Cursors
        if (this.showHandles && this.currentShape) {
            const handle = this.getHandleAtPosition(pos);
            if (handle === 'move') this.canvas.style.cursor = 'move';
            else if (handle === 'rot') this.canvas.style.cursor = 'alias';
            else if (handle) this.canvas.style.cursor = 'pointer';
            else this.canvas.style.cursor = 'default';
        } else {
            this.canvas.style.cursor = 'crosshair';
        }

        // Handle Drawing (Creation)
        if (!this.isDrawing || !this.lastPos || !this.startPos) return;
        this.currentPos = pos;

        if (this.shapeType === 'freehand') {
            this.drawingCtx.lineTo(pos.x, pos.y);
            this.drawingCtx.stroke();
            this.addPixelsToSet(this.lastPos.x, this.lastPos.y, pos.x, pos.y);
            this.lastPos = pos;
        } else {
            const shape: Shape = {
                type: this.shapeType,
                startPos: this.startPos,
                endPos: pos,
                rotation: 0
            };
            this.drawingCtx.clearRect(0, 0, this.drawingCanvas.width, this.drawingCanvas.height);
            this.drawShape(shape);
        }
    }

    private handlePointerUp(e: PointerEvent): void {
        if (!this.drawingEnabled) return;
        e.preventDefault();
        (e.target as HTMLElement).releasePointerCapture(e.pointerId);

        if (this.selectedHandle) {
            this.selectedHandle = null;
            this.dragOffsets = null; 
            if (this.currentShape) this.finalizeShape(this.currentShape);
            return;
        }

        if (this.isDrawing && this.startPos && this.currentPos) {
            if (this.shapeType !== 'freehand') {
                this.currentShape = {
                    type: this.shapeType,
                    startPos: this.startPos,
                    endPos: this.currentPos,
                    rotation: 0
                };
                this.finalizeShape(this.currentShape);
                this.showHandles = true;
                this.redraw();
            }
            if (this.onShapeCompleteCallback) this.onShapeCompleteCallback();
        }
        this.isDrawing = false;
    }

    private handlePointerCancel(e: PointerEvent): void {
        (e.target as HTMLElement).releasePointerCapture(e.pointerId);
        this.isDrawing = false;
        this.selectedHandle = null;
        this.dragOffsets = null;
    }

    // --- Public API ---

    public drawOnMainCanvas(): void {
        this.ctx.drawImage(this.drawingCanvas, 0, 0);
    }

    public clearDrawing(): void {
        this.drawingCtx.clearRect(0, 0, this.drawingCanvas.width, this.drawingCanvas.height);
        this.drawnPixels.clear();
        this.startPos = null;
        this.currentPos = null;
        this.currentShape = null;
        this.showHandles = false;
        this.selectedHandle = null;
        this.dragOffsets = null;
    }

    public updateCanvasSize(width: number, height: number): void {
        this.drawingCanvas.width = width;
        this.drawingCanvas.height = height;
        this.configureCtx();
    }

    public setShapeType(shapeType: ShapeType): void {
        this.shapeType = shapeType;
    }

    public enableDrawing(onShapeComplete?: () => void): void {
        this.drawingEnabled = true;
        this.canvas.style.cursor = 'crosshair';
        this.onShapeCompleteCallback = onShapeComplete || null;
    }

    public disableDrawing(): void {
        this.drawingEnabled = false;
        this.isDrawing = false;
        this.showHandles = false;
        this.canvas.style.cursor = 'default';
    }

    public isDrawingEnabled(): boolean {
        return this.drawingEnabled;
    }

    /**
     * Helper: Creates a Blob of the path (White background, Black line)
     */
    private generatePathImageBlob(): Promise<Blob> {
        return new Promise((resolve, reject) => {
            // 1. Create temporary canvas
            const tempCanvas = document.createElement('canvas');
            tempCanvas.width = this.canvas.width;
            tempCanvas.height = this.canvas.height;
            const tCtx = tempCanvas.getContext('2d')!;

            // 2. Fill Background White
            tCtx.fillStyle = '#FFFFFF';
            tCtx.fillRect(0, 0, tempCanvas.width, tempCanvas.height);

            // 3. Draw Pixels as Black
            tCtx.fillStyle = '#000000';
            this.drawnPixels.forEach(key => {
                const [x, y] = key.split(',').map(Number);
                tCtx.fillRect(x, y, 1, 1);
            });

            // 4. Convert to Blob
            tempCanvas.toBlob(blob => {
                if (blob) resolve(blob);
                else reject(new Error('Failed to create image blob'));
            }, 'image/png');
        });
    }

    /**
     * Sends the pixel coordinates and speed to /api/path
     */
    private async sendCoordinates(speed: number): Promise<any> {
        if (this.video.videoWidth === 0 || this.video.videoHeight === 0) {
            throw new Error("Video dimensions missing.");
        }

        const pixels = Array.from(this.drawnPixels).map(key => {
            const [x, y] = key.split(',').map(Number);
            return { x, y };
        });

        if (pixels.length === 0) {
            console.warn("No pixels to send.");
            return null;
        }

        // Normalize coordinates
        const videoPixels = pixels.map(p => ({
            x: p.x / this.canvas.width * this.video.videoWidth,
            y: p.y / this.canvas.height * this.video.videoHeight
        }));

        const response = await fetch(`${this.apiBaseUrl}/api/path`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 
                speed: speed, 
                pixels: videoPixels 
            })
        });

        if (!response.ok) throw new Error(`JSON path upload failed: ${response.status}`);
        return await response.json();
    }

    /**
     * Sends the generated PNG to /api/raster_mask
     */
    private async uploadPathImage(): Promise<any> {
        if (this.drawnPixels.size === 0) return null;

        const blob = await this.generatePathImageBlob();
        const formData = new FormData();
        formData.append('file', blob, 'path.png');

        const response = await fetch(`${this.apiBaseUrl}/api/raster_mask`, {
            method: 'POST',
            body: formData
        });

        if (!response.ok) throw new Error(`Image upload failed: ${response.status}`);
        return await response.json();
    }

    /**
     * Orchestrator: Sends both JSON data and Image separately
     */
    public async executePath(speed: number): Promise<any> {
        console.log("Starting parallel upload of Path JSON and Path Image...");
        
        // Run both requests in parallel
        const [jsonResult, imageResult] = await Promise.all([
            this.sendCoordinates(speed),
            this.uploadPathImage()
        ]);

        return { jsonResult, imageResult };
    }
}