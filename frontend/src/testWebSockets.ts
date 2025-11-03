export {}

document.addEventListener('DOMContentLoaded', () => {

    const wsOutput = document.getElementById("wsOutput") as HTMLElement
    const httpOutput = document.getElementById("httpOutput") as HTMLElement
    const x_input = document.getElementById("x") as HTMLInputElement
    const y_input = document.getElementById("y") as HTMLInputElement
    const z_input = document.getElementById("z") as HTMLInputElement
    const rx_input = document.getElementById("rx") as HTMLInputElement
    const ry_input = document.getElementById("ry") as HTMLInputElement
    const rz_input = document.getElementById("rz") as HTMLInputElement
    const sendBtn = document.getElementById("sendBtn") as HTMLButtonElement
    const fetchBtn = document.getElementById("fetchBtn") as HTMLButtonElement

    let ws: WebSocket | null = null;
    connectWebSocket();

    sendBtn.addEventListener('click', async () => {
        sendCoordinates();
    })
    fetchBtn.addEventListener('click', async () => {
        fetchCurrentCoordinates();
    })

    function connectWebSocket() {
        if (ws) {
            ws.onopen = null;
            ws.onmessage = null;
            ws.onclose = null;
            ws.onerror = null;
            
            ws.close();
        }

        wsOutput.textContent = "Connecting to WebSocket...\n";

        ws = new WebSocket(`ws://localhost:443/ws/${import.meta.env.VITE_UI_WEBSOCKET_NAME}`);
        
        ws.onopen = () => wsOutput.textContent += "Connected to WebSocket\n";
        ws.onmessage = (event) => wsOutput.textContent += event.data + "\n";
        ws.onclose = () => wsOutput.textContent += "Disconnected\n";
        ws.onerror = (error) => wsOutput.textContent += "WebSocket error: " + error + "\n";
    }

    // sends cordinates as a json string through the websocket
    function sendCoordinates() {
        if (ws && ws.readyState == WebSocket.OPEN) {
            const data = {
                x: parseFloat(x_input.value),
                y: parseFloat(y_input.value),
                z: parseFloat(z_input.value),
                rx: parseFloat(rx_input.value),
                ry: parseFloat(ry_input.value),
                rz: parseFloat(rz_input.value)
            };

            ws.send(JSON.stringify(data));
        }
        else {
            wsOutput.textContent += "Not connected to WebSocket\n";
        }
    }

    // fetches the current coordintates from the backend using the http endpoint
    async function fetchCurrentCoordinates() {
        try {
            const response = await fetch("http://localhost:443/current-coordinates");
            const json = await response.json();
            httpOutput.textContent = JSON.stringify(json, null, 2);
        } catch (err) {
            httpOutput.textContent = "Error fetching coordinates: " + err;
        }
    }
});