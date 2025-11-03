export {}

document.addEventListener('DOMContentLoaded', () => {

    const testBtn = document.getElementById('connectBtn') as HTMLButtonElement
    const testOut = document.getElementById('connectOutput') as HTMLElement

    testBtn.addEventListener('click', async () => {
        testOut.textContent = 'Connecting...'
        try {
            // Call the backend which is accessible on the host at port 443 in this compose setup
            const res = await fetch('http://localhost:443/media/mystream')
            const data = await res.json()
            testOut.textContent = JSON.stringify(data, null, 2)
        } catch (e) {
            testOut.textContent = 'Error: ' + String(e)
        }
    })

    const laserButton = document.getElementById('laserBtn') as HTMLButtonElement
    const laserButtonOut = document.getElementById('laserOutput') as HTMLElement
    const connectWs = document.getElementById('connectWs') as HTMLButtonElement

    let ws: WebSocket | null = null;
    let isLaserOn: boolean = false;

    laserButton.addEventListener('click', async () => {
        sendLaserStatus();
    })
    connectWs.addEventListener('click', async () => {
        connectWebSocket();
    })

    function connectWebSocket() {
        if (ws) {
            ws.onopen = null;
            ws.onmessage = null;
            ws.onclose = null;
            ws.onerror = null;
            
            ws.close();
        }

        laserButtonOut.textContent = "Connecting to WebSocket...\n";

        ws = new WebSocket("ws://localhost:443/ws/coordinates");
        
        ws.onopen = () => laserButtonOut.textContent += "Connected to WebSocket\n";
        ws.onmessage = (event) => laserButtonOut.textContent += event.data + "\n";
        ws.onclose = () => laserButtonOut.textContent += "Disconnected\n";
        ws.onerror = (error) => laserButtonOut.textContent += "WebSocket error: " + error + "\n";
    }

    function sendLaserStatus() {
        if (ws && ws.readyState === WebSocket.OPEN) {
            isLaserOn = !isLaserOn;

            ws.send(JSON.stringify({isLaserOn}));

            laserButtonOut.textContent += `Sent command: ${isLaserOn ? 'On' : 'Off'}\n`;
            laserButton.textContent = isLaserOn ? "Turn off Laser" : "Turn on Laser";
            laserButton.classList.toggle('laser-on', isLaserOn);
        }
        else {
            laserButtonOut.textContent += "Not connected. Please 'Connect to websocket' first.\n";
        }
    }
});

