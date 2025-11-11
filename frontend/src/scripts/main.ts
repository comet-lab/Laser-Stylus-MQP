import { WebSocketHandler, WebSocketMessage } from './classes/WebSocketHandler';

document.addEventListener('DOMContentLoaded', () => {
    // --- Get Elements ---
    const wsOutput = document.getElementById("wsOutput") as HTMLElement;
    const httpOutput = document.getElementById("httpOutput") as HTMLElement;
    const x_input = document.getElementById("x") as HTMLInputElement;
    const y_input = document.getElementById("y") as HTMLInputElement;
    const z_input = document.getElementById("z") as HTMLInputElement;
    const rx_input = document.getElementById("rx") as HTMLInputElement;
    const ry_input = document.getElementById("ry") as HTMLInputElement;
    const rz_input = document.getElementById("rz") as HTMLInputElement;
    const laserButton = document.getElementById('laserBtn') as HTMLButtonElement;
    const sendBtn = document.getElementById("sendBtn") as HTMLButtonElement;
    const fetchBtn = document.getElementById("fetchBtn") as HTMLButtonElement;

    // --- Helper Functions ---
    
    /** Reads all coordinate inputs and returns them as an object */
    function getCoordinates() {
        return {
            x: parseFloat(x_input.value) || 0,
            y: parseFloat(y_input.value) || 0,
            z: parseFloat(z_input.value) || 0,
            rx: parseFloat(rx_input.value) || 0,
            ry: parseFloat(ry_input.value) || 0,
            rz: parseFloat(rz_input.value) || 0
        };
    }

    /** Reads the laser's *current state* from the button's class */
    function getLocalLaserState(): boolean {
        return laserButton.classList.contains('laser-on');
    }

    /**
     * This is our single function for updating the UI.
     * It's called by the WebSocket handler and the fetch button.
     */
    function syncUiToState(state: Partial<WebSocketMessage>) {
        // Update sliders (only if values are present)
        if (state.x !== undefined) x_input.value = String(state.x);
        if (state.y !== undefined) y_input.value = String(state.y);
        if (state.z !== undefined) z_input.value = String(state.z);
        if (state.rx !== undefined) rx_input.value = String(state.rx);
        if (state.ry !== undefined) ry_input.value = String(state.ry);
        if (state.rz !== undefined) rz_input.value = String(state.rz);

        // Update laser button (only if laser state is present)
        if (state.isLaserOn !== undefined) {
            const newLaserState = !!state.isLaserOn;
            laserButton.textContent = newLaserState ? "Turn Off Laser" : "Turn On Laser";
            laserButton.classList.toggle('laser-on', newLaserState);
        }
    }

    // --- WebSocket Setup ---
    const wsHandler = new WebSocketHandler(wsOutput);
    // implemented the following line of code so that the console has access to the handler
    // then I can do wsHandler.disconnect() and wsHandler.connect() to test that it properly turns the laser off
    //(window as any).wsHandler = wsHandler;
    
    // Assign the callback *before* connecting.
    // This function will run every time the backend broadcasts a new state.
    wsHandler.onStateUpdate = (newState) => {
        syncUiToState(newState);
    };

    wsHandler.connect();

    // --- Event Listeners ---

    // When clicking laser button, request a *toggle*
    laserButton.addEventListener('click', () => {
        // 1. Read the *current* state from the UI
        const isCurrentlyOn = getLocalLaserState();
        
        // 2. Request the *opposite* state
        const desiredNewState = !isCurrentlyOn;
        
        // 3. Send only the laser state update (coordinates unchanged on backend)
        wsHandler.updateState({ isLaserOn: desiredNewState });
    });

    // When clicking send button, send the current coordinates and laser state
    sendBtn.addEventListener('click', () => {
        // Send a complete update with all current UI values
        wsHandler.updateState({
            ...getCoordinates(),
            isLaserOn: getLocalLaserState()
        });
    });

    // When clicking fetch button, fetch the data and *manually sync* the UI
    fetchBtn.addEventListener('click', async () => {
        httpOutput.textContent = 'Fetching current state...';
        try {
            const res = await fetch(`http://${window.location.hostname}:443/current-coordinates`);
            if (!res.ok) throw new Error(`HTTP Error! Status: ${res.status}`);
            
            const data = await res.json() as WebSocketMessage;
            httpOutput.textContent = JSON.stringify(data, null, 2);
            
            // Manually sync the UI to the state we just fetched
            syncUiToState(data);
        } catch (e) {
            const errorMsg = (e instanceof Error) ? e.message : String(e);
            httpOutput.textContent = 'Error: ' + errorMsg;
        }
    });

    // Optional: Add input event listeners to send updates in real-time
    /*
    [x_input, y_input, z_input, rx_input, ry_input, rz_input].forEach(input => {
        input.addEventListener('input', () => {
            wsHandler.updateCoordinates(getCoordinates());
        });
    });
    */
});