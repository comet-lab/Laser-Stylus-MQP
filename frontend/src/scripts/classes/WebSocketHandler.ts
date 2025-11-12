// Define the state interface matching your backend example_data
export interface WebSocketMessage {
    x: number;
    y: number;
    z: number;
    rx: number;
    ry: number;
    rz: number;
    laserX: number;
    laserY: number;
    beamWaist: number;
    speed: number;
    isLaserOn: boolean;
}

// Type for partial updates, any subset of WebSocketMessage
export type PartialWebSocketMessage = Partial<WebSocketMessage>;

export class WebSocketHandler {
    private ws: WebSocket | null = null;
    private url: string;
    private outputElement: HTMLElement | null;
    public onStateUpdate: ((newState: WebSocketMessage) => void) | null = null;

    constructor(outputElement: HTMLElement | null) {
        this.url = `ws://${window.location.hostname}:443/ws/${import.meta.env.VITE_UI_WEBSOCKET_NAME}`;
        this.outputElement = outputElement;
    }

    private log(message: string) {
        if (this.outputElement) {
            // If an element was provided, use it
            this.outputElement.textContent += message + "\n";
        } else {
            // Otherwise, log to the console
            console.log(`[WebSocketHandler]: ${message}`);
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
        
        this.ws.onopen = () => this.log("Connected to WebSocket");
        this.ws.onmessage = (event) => {
            this.log(`Received: ${event.data}`);
            try {
                const newState = JSON.parse(event.data) as WebSocketMessage;
                
                if (this.onStateUpdate) {
                    this.onStateUpdate(newState);
                }
            } catch (e) {
                this.log(`Error parsing incoming state: ${e instanceof Error ? e.message : String(e)}`);
            }
        };
        this.ws.onclose = () => this.log("Disconnected");
        this.ws.onerror = (error) => this.log(`WebSocket error: ${String(error)}`);
    }

    public send(message: string): boolean {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(message);
            return true;
        }
        this.log("Not connected. Cannot send data");
        return false;
    }

    // Generic method to update any subset of the state
    public updateState(updates: PartialWebSocketMessage): boolean {
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
            this.log("Not connected. Cannot send data");
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