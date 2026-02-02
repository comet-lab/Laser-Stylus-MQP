import { WebSocketHandler, WebSocketMessage } from './api/WebSocketHandler';

class ControlPanel {
    private ui = {
        wsOutput: document.getElementById("wsOutput") as HTMLElement,
        httpOutput: document.getElementById("httpOutput") as HTMLElement,
        inputs: {
            x: document.getElementById("x") as HTMLInputElement,
            y: document.getElementById("y") as HTMLInputElement,
            z: document.getElementById("z") as HTMLInputElement,
            rx: document.getElementById("rx") as HTMLInputElement,
            ry: document.getElementById("ry") as HTMLInputElement,
            rz: document.getElementById("rz") as HTMLInputElement,
        },
        laserButton: document.getElementById('laserBtn') as HTMLButtonElement,
        sendBtn: document.getElementById("sendCoordsBtn") as HTMLButtonElement,
        fetchBtn: document.getElementById("fetchBtn") as HTMLButtonElement,
    };

    private wsHandler: WebSocketHandler;

    constructor() {
        this.wsHandler = new WebSocketHandler(this.ui.wsOutput);
        this.init();
    }

    private init() {
        this.setupWebSocket();
        this.bindEvents();
    }

    private setupWebSocket() {
        this.wsHandler.onStateUpdate = (newState) => {
            this.syncUiToState(newState);
        };
        this.wsHandler.connect();
    }

    private bindEvents() {
        this.ui.laserButton.addEventListener('click', () => this.toggleLaser());
        this.ui.sendBtn.addEventListener('click', () => this.sendCoordinates());
        this.ui.fetchBtn.addEventListener('click', () => this.fetchCurrentState());
    }

    private getCoordinates() {
        return {
            x: parseFloat(this.ui.inputs.x.value) || 0,
            y: parseFloat(this.ui.inputs.y.value) || 0,
            z: parseFloat(this.ui.inputs.z.value) || 0,
            rx: parseFloat(this.ui.inputs.rx.value) || 0,
            ry: parseFloat(this.ui.inputs.ry.value) || 0,
            rz: parseFloat(this.ui.inputs.rz.value) || 0
        };
    }

    private getLocalLaserState(): boolean {
        return this.ui.laserButton.classList.contains('laser-on');
    }

    private syncUiToState(state: Partial<WebSocketMessage>) {
        if (state.isLaserOn !== undefined) {
            const newLaserState = !!state.isLaserOn;
            this.ui.laserButton.textContent = newLaserState ? "Turn Off Laser" : "Turn On Laser";
            this.ui.laserButton.classList.toggle('laser-on', newLaserState);
        }
    }

    private toggleLaser() {
        const isCurrentlyOn = this.getLocalLaserState();
        const desiredNewState = !isCurrentlyOn;
        this.wsHandler.updateState({ isLaserOn: desiredNewState });
    }

    private sendCoordinates() {
        this.wsHandler.updateState({
            ...this.getCoordinates(),
            isLaserOn: this.getLocalLaserState()
        });
    }

    private async fetchCurrentState() {
        this.ui.httpOutput.textContent = 'Fetching current state...';
        try {
            const res = await fetch(`http://${window.location.hostname}:443/current-coordinates`);
            if (!res.ok) throw new Error(`HTTP Error! Status: ${res.status}`);
            
            const data = await res.json() as WebSocketMessage;
            this.ui.httpOutput.textContent = JSON.stringify(data, null, 2);
            this.syncUiToState(data);
        } catch (e) {
            const errorMsg = (e instanceof Error) ? e.message : String(e);
            this.ui.httpOutput.textContent = 'Error: ' + errorMsg;
        }
    }
}

document.addEventListener('DOMContentLoaded', () => {
    new ControlPanel();
});