export class WebSocketHandler {
    private ws: WebSocket | null = null;
    private url: string;
    private outputElement: HTMLElement;

    public onStateUpdate: ((newState: any) => void) | null = null;


    constructor(outputElement: HTMLElement) {
        this.url = `ws://localhost:443/ws/${import.meta.env.VITE_UI_WEBSOCKET_NAME}`;
        this.outputElement = outputElement;
    }

    private log(message: string) {
        this.outputElement.textContent += message + "\n";
    }

    //Connect to websocket
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
            this.log(event.data); // Log the raw data
            try {
                //Any time a message is sent from the backend, parse and update state on frontend
                const newState = JSON.parse(event.data);
                
                // If the callback is set, call it with the new state
                if (this.onStateUpdate) {
                    this.onStateUpdate(newState);
                }

            } catch (e) {
                this.log("Error parsing incoming state: " + String(e));
            }
        };
        this.ws.onclose = () => this.log("Disconnected");
        this.ws.onerror = (error) => this.log("WebSocket error: " + String(error));
    }

    //Send a given message through the websocket
    public send(message: string): boolean {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(message);
            return true;
        }
        else {
            this.log("Not connected. Cannot send data");
            return false;
        }
    }

    //Compile a message to send to the backend through the websocket with state and coord data
    public updateStates(coords: { x: number, y: number, z: number, rx: number, ry: number, rz: number },
                        isLaserOn: boolean
            ): boolean {
        
        //If websocket is connected, log message and return false
        if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
            this.log("Not connected. Cannot send data");
            return false;
        }

        const payload = {
            ...coords,
            isLaserOn: isLaserOn
        };

        const jsonPayload = JSON.stringify(payload);
        const sent = this.send(jsonPayload);

        if (sent) {
            this.log("Sent state update")
        }        

        return sent;
    }
}

