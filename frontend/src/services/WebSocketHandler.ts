// frontend/src/services/WebSocketHandler.ts

// Define the state interface matching backend RobotSchema (robot.py)
export interface WebSocketMessage {
    //Coordinates
    x?: number;
    y?: number;
    z?: number;
    rx?: number;
    ry?: number;
    rz?: number;

    //Laser and Vision State
    laserX?: number;
    laserY?: number;
    averageHeat?: number;
    maxHeat?: number; 
    beamWaist?: number;

    //Flags and Views
    isLaserOn?: boolean;
    isRobotOn?: boolean;
    isLaserFiring?: boolean;
    isTransformedViewOn?: boolean;
    isThermalViewOn?: boolean;
    request_sync?: boolean;

    //Settings
    speed?: number;
    passes?: number;
    density?: number;
    height?: number;
    current_height?: number;
    raster_type?: string;
    pathEvent?: string | null;

    //Complex Data and Masks
    path?: { x: number, y: number }[];
    heat_markers?: { x: number, y: number }[];
    raster_mask?: string;
    fixtures_mask?: string;
    heat_mask?: string;

    //Execution and Preview
    pathPrepared?: boolean;
    executeCommand?: boolean;
    path_preview?: {
        x: number[];
        y: number[];
    };
    preview_duration?: number;
}

export type PartialWebSocketMessage = Partial<WebSocketMessage>;

export class WebSocketHandler {
    private ws: WebSocket | null = null;
    private url: string;
    private outputElement: HTMLElement | null;
    public onStateUpdate: ((newState: WebSocketMessage) => void) | null = null;
    public onOpen: (() => void) | null = null;

    constructor(outputElement: HTMLElement | null) {
        // Fallback to local 8000 if env variable is missing
        const wsName = import.meta.env.VITE_UI_WEBSOCKET_NAME || 'ui';
        this.url = `ws://${window.location.hostname}:443/ws/${wsName}`;
        this.outputElement = outputElement;
    }

    private log(message: string) {
        if (this.outputElement) {
            this.outputElement.textContent += message + "\n";
        } else {
            // Uncomment to debug raw WS traffic in console
            // console.log(`[WebSocket]: ${message}`);
        }
    }

    public connect() {
        if (this.ws) {
            this.ws.onopen = null;
            this.ws.onmessage = null;
            this.ws.onclose = null;
            this.ws.onerror = null;
            this.ws.close();
        }

        this.log("Connecting to WebSocket...");
        this.ws = new WebSocket(this.url);

        this.ws.onopen = () => {
            console.log('WebSocket Connected');
            if (this.onOpen) this.onOpen();
        };
        
        this.ws.onmessage = (event) => {
            try {
                const newState = JSON.parse(event.data) as WebSocketMessage;
                if (this.onStateUpdate) {
                    this.onStateUpdate(newState);
                }
            } catch (e) {
                this.log(`Error parsing incoming state: ${e instanceof Error ? e.message : String(e)}`);
            }
        };
        
        this.ws.onclose = () => {
            console.warn('WebSocket Disconnected. Reconnecting in 2 seconds...');
            this.log("Disconnected");
            // Optional: Auto-reconnect logic
            setTimeout(() => this.connect(), 2000);
        };
        
        this.ws.onerror = (error) => this.log(`WebSocket error: ${String(error)}`);
    }

    public send(message: string): boolean {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(message);
            return true;
        }
        this.log("Not connected. Cannot send data.");
        return false;
    }

    public updateState(updates: PartialWebSocketMessage): boolean {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
            this.log("Not connected. Cannot update state.");
            return false;
        }

        const jsonPayload = JSON.stringify(updates);
        const sent = this.send(jsonPayload);
        
        if (sent) {
            this.log(`Sent state update: ${jsonPayload}`);
        }
        
        return sent;
    }

    public disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }
}