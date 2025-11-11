interface Position {
    x: number;
    y: number;
}

//Class to track the drawing on the canvas
export class DrawingTracker {
    private canvas: HTMLCanvasElement;
    private video: HTMLVideoElement;
    private ctx: CanvasRenderingContext2D;
    private drawnPixels: Set<string>;
    private isDrawing: boolean;
    private drawingEnabled: boolean; // New flag
    private lastPos: Position | null;
    private drawingCanvas: HTMLCanvasElement;
    private drawingCtx: CanvasRenderingContext2D;
    private apiBaseUrl: string;

    constructor(canvas: HTMLCanvasElement, video: HTMLVideoElement, apiBaseUrl: string = `http://${window.location.hostname}:443`) {
        this.canvas = canvas;
        this.video = video;
        this.ctx = canvas.getContext('2d')!;
        this.apiBaseUrl = apiBaseUrl;
        this.drawnPixels = new Set<string>();
        this.isDrawing = false;
        this.drawingEnabled = false; // Start disabled
        this.lastPos = null;

        // Create a separate canvas for drawing that won't be cleared
        this.drawingCanvas = document.createElement('canvas');
        this.drawingCanvas.width = canvas.width;
        this.drawingCanvas.height = canvas.height;
        this.drawingCtx = this.drawingCanvas.getContext('2d')!;

        // Configure drawing style
        this.drawingCtx.strokeStyle = '#ff0000';
        this.drawingCtx.lineWidth = 5;
        this.drawingCtx.lineCap = 'round';
        this.drawingCtx.lineJoin = 'round';

        this.setupEventListeners();
    }

    // Pointer event listeners for drawing
    private setupEventListeners(): void {
        this.canvas.addEventListener('pointerdown', this.handlePointerDown.bind(this));
        this.canvas.addEventListener('pointermove', this.handlePointerMove.bind(this));
        this.canvas.addEventListener('pointerup', this.handlePointerUp.bind(this));
        this.canvas.addEventListener('pointercancel', this.handlePointerCancel.bind(this));
    }

    //Get the position of the pointer on the canvas
    private getPointerPos(e: PointerEvent): Position {
        const rect = this.canvas.getBoundingClientRect();
        const scaleX = this.canvas.width / rect.width;
        const scaleY = this.canvas.height / rect.height;

        return {
            x: Math.floor((e.clientX - rect.left) * scaleX),
            y: Math.floor((e.clientY - rect.top) * scaleY)
        };
    }

    //Add pixels along a line using Bresenham's algorithm
    //Fills in any gaps when drawing
    private addPixelsAlongLine(x0: number, y0: number, x1: number, y1: number, brushSize: number = 5): void {
        const dx = Math.abs(x1 - x0);
        const dy = Math.abs(y1 - y0);
        const sx = x0 < x1 ? 1 : -1;
        const sy = y0 < y1 ? 1 : -1;
        let err = dx - dy;

        while (true) {
            for (let bx = -brushSize; bx <= brushSize; bx++) {
                for (let by = -brushSize; by <= brushSize; by++) {
                    this.drawnPixels.add(`${x0 + bx},${y0 + by}`);
                }
            }
            if (x0 === x1 && y0 === y1) break;
            const e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x0 += sx; }
            if (e2 < dx) { err += dx; y0 += sy; }
        }
    }

    //When you click, check if drawing is enabled and start drawing
    private handlePointerDown(e: PointerEvent): void {
        if (!this.drawingEnabled) return; // Check if drawing is enabled
        
        e.preventDefault();

        (e.target as HTMLElement).setPointerCapture(e.pointerId);

        const pos = this.getPointerPos(e);
        this.isDrawing = true;
        this.lastPos = pos;

        this.drawingCtx.beginPath();
        this.drawingCtx.moveTo(pos.x, pos.y);

        this.addPixelsAlongLine(pos.x, pos.y, pos.x, pos.y, 5);
    }

    //When you move it, check if drawing is enabled and you're drawing
    private handlePointerMove(e: PointerEvent): void {
        if (!this.drawingEnabled) return; // Check if drawing is enabled
        
        e.preventDefault();
        if (!this.isDrawing || !this.lastPos) return;

        const pos = this.getPointerPos(e);

        this.drawingCtx.lineTo(pos.x, pos.y);
        this.drawingCtx.stroke();

        this.addPixelsAlongLine(this.lastPos.x, this.lastPos.y, pos.x, pos.y, 5);
        this.lastPos = pos;
    }

    //Stop drawing when you release
    private handlePointerUp(e: PointerEvent): void {
        if (!this.drawingEnabled) return;
        
        e.preventDefault();

        (e.target as HTMLElement).releasePointerCapture(e.pointerId);

        this.isDrawing = false;
    }

    //Handle pointer cancel event
    private handlePointerCancel(e: PointerEvent): void {
        (e.target as HTMLElement).releasePointerCapture(e.pointerId);
        this.isDrawing = false;
    }

    //Return list of drawn pixels
    private getDrawnPixelList(): Position[] {
        return Array.from(this.drawnPixels).map(key => {
            const [x, y] = key.split(',').map(Number);
            return { x, y };
        });
    }

    //Draw the overlay on the canvas
    public drawOnMainCanvas(): void {
        // Draw the drawing overlay on top of the video frame
        this.ctx.drawImage(this.drawingCanvas, 0, 0);
    }

    //Clear drawing on canvas
    public clearDrawing(): void {
        this.drawingCtx.clearRect(0, 0, this.drawingCanvas.width, this.drawingCanvas.height);
        this.drawnPixels.clear();
    }

    //Update drawing canvas size on window resize
    public updateCanvasSize(width: number, height: number): void {
        this.drawingCanvas.width = width;
        this.drawingCanvas.height = height;
        // Reconfigure drawing style after resize
        this.drawingCtx.strokeStyle = '#ff0000';
        this.drawingCtx.lineWidth = 5;
        this.drawingCtx.lineCap = 'round';
        this.drawingCtx.lineJoin = 'round';
    }

    //Enable drawing mode
    public enableDrawing(): void {
        this.drawingEnabled = true;
        this.canvas.style.cursor = 'crosshair';
    }

    //Disable drawing mode
    public disableDrawing(): void {
        this.drawingEnabled = false;
        this.isDrawing = false;
        this.canvas.style.cursor = 'default';
    }

    //Check if drawing is enabled
    public isDrawingEnabled(): boolean {
        return this.drawingEnabled;
    }

    //Send coordinates to backend via websocket
    public async sendCoordinates(): Promise<{ status: string; pixel_count: number; pixels: Position[]; message: string } | null> {
        const pixels = this.getDrawnPixelList();
        if (pixels.length === 0) {
            console.log("No pixels to send");
            return null;
        }

        // Convert to video coordinates
        const videoPixels = pixels.map(p => ({
            x: p.x / this.canvas.width * this.video.videoWidth,
            y: p.y / this.canvas.height * this.video.videoHeight
        }));

        try {
            const response = await fetch(`${this.apiBaseUrl}/api/path`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    pixels: videoPixels
                })
            });

            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            const data = await response.json();
            console.log(`Sent ${videoPixels.length} pixels, received response:`, data);
            return data;

        } catch (error) {
            console.error('Error sending coordinates:', error);
            throw error;
        }
    }
}