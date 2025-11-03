import { WebSocketHandler } from './classes/WebSocketHandler';

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
    function syncUiToState(state: any) {
        // Update sliders
        x_input.value = state.x;
        y_input.value = state.y;
        z_input.value = state.z;
        rx_input.value = state.rx;
        ry_input.value = state.ry;
        rz_input.value = state.rz;

        // Update laser button
        const newLaserState = !!state.isLaserOn;
        laserButton.textContent = newLaserState ? "Turn Off Laser" : "Turn On Laser";
        laserButton.classList.toggle('laser-on', newLaserState);
    }

    // --- WebSocket Setup ---
    const wsHandler = new WebSocketHandler(wsOutput);
    
    // Assign the callback *before* connecting.
    // This function will run every time the backend broadcasts a new state.
    wsHandler.onStateUpdate = (newState) => {
        syncUiToState(newState);
    };

    wsHandler.connect();

    // --- Event Listeners ---

    //When clicking laser button, request a *toggle*
    laserButton.addEventListener('click', async () => {
        // 1. Read the *current* state from the UI
        const isCurrentlyOn = getLocalLaserState();
        
        // 2. Request the *opposite* state
        const desiredNewState = !isCurrentlyOn;
        
        // 3. Send the request
        wsHandler.updateStates(getCoordinates(), desiredNewState);
    });

    //When clicking send button, send the current coordinates and laser state
    sendBtn.addEventListener('click', async () => {
        // Send a "sync" request with the current UI values
        wsHandler.updateStates(getCoordinates(), getLocalLaserState());
    });

    //When clicking fetch button, fetch the data and *manually sync* the UI
    fetchBtn.addEventListener('click', async () => {
        httpOutput.textContent = 'Fetching current state...';
        try {
            const res = await fetch('http://localhost:443/current-coordinates');
            if (!res.ok) throw new Error(`HTTP Error! Status: ${res.status}`);

            const data = await res.json();
            httpOutput.textContent = JSON.stringify(data, null, 2);

            // Manually sync the UI to the state we just fetched
            syncUiToState(data);

        } catch (e) {
            const errorMsg = (e instanceof Error) ? e.message : String(e);
            httpOutput.textContent = 'Error: ' + errorMsg;
        }
    });
});